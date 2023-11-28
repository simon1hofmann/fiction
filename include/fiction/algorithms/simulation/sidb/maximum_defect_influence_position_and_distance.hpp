//
// Created by Jan Drewniok on 21.06.23.
//

#ifndef FICTION_MAXIMUM_DEFECT_INFLUENCE_POSITION_AND_DISTANCE_HPP
#define FICTION_MAXIMUM_DEFECT_INFLUENCE_POSITION_AND_DISTANCE_HPP

#include "fiction/algorithms/simulation/sidb/critical_temperature.hpp"
#include "fiction/algorithms/simulation/sidb/quickexact.hpp"
#include "fiction/layouts/bounding_box.hpp"
#include "fiction/technology/sidb_defects.hpp"
#include "fiction/technology/sidb_surface.hpp"
#include "fiction/types.hpp"
#include "fiction/utils/execution_utils.hpp"
#include "fiction/utils/layout_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace fiction
{
/**
 * This struct stores the parameters for the maximum_defect_influence_position_and_distance algorithm.
 */
struct maximum_defect_influence_distance_params
{
    /**
     * The defect to calculate the maximum defect influence distance for.
     */
    sidb_defect defect{};
    /**
     * Physical simulation parameters.
     */
    sidb_simulation_parameters physical_params{};
    /**
     * The pair describes the width and height of the area around the gate, which is
     * also used to place defects.
     *
     * @note If SiQAD coordinates are used, the second entry describes the number of dimer rows.
     */
    std::pair<int32_t, int32_t> additional_scanning_area{50, 6};
};

namespace detail
{
/**
 * A class for simulating the maximum influence distance of defects within an SiDB layout.
 *
 * This class is responsible for simulating the distance at which defects placed within an SiDB
 * layout still influence the ground state of the layout. It conducts simulations at various defect positions,
 * identifying the position that maximally impacts the layout and calculating the associated influence distance.
 *
 * The class provides a `run` method to initiate the simulation and compute the maximum influence
 * distance and corresponding defect position. It utilizes multithreading for efficient defect
 * position simulations.
 */
template <typename Lyt>
class maximum_defect_influence_position_and_distance_impl
{
  public:
    maximum_defect_influence_position_and_distance_impl(const Lyt&                                      lyt,
                                                        const maximum_defect_influence_distance_params& sim_params) :
            layout{lyt},
            params{sim_params}
    {
        collect_all_defect_cells();
    }

    std::pair<typename Lyt::cell, double> run() noexcept
    {
        const quickexact_params<sidb_surface<Lyt>> params_defect{
            params.physical_params, quickexact_params<sidb_surface<Lyt>>::automatic_base_number_detection::OFF};

        double          avoidance_distance{0};
        coordinate<Lyt> max_defect_position{};

        const auto simulation_results =
            quickexact(layout, quickexact_params<Lyt>{params.physical_params,
                                                      quickexact_params<Lyt>::automatic_base_number_detection::OFF});

        const auto min_energy          = minimum_energy(simulation_results.charge_distributions);
        uint64_t   charge_index_layout = 0;

        for (auto& lyt_result : simulation_results.charge_distributions)
        {
            if (std::fabs(round_to_n_decimal_places(lyt_result.get_system_energy(), 6) -
                          round_to_n_decimal_places(min_energy, 6)) < std::numeric_limits<double>::epsilon())
            {
                lyt_result.charge_distribution_to_index_general();
                charge_index_layout = lyt_result.get_charge_index_and_base().first;
            }
        }

        // simulate the impact of the defect at a given position on the ground state of the SiDB layout
        const auto process_defect = [&](const auto& defect) noexcept
        {
            if (layout.get_cell_type(defect) == Lyt::technology::cell_type::EMPTY)
            {
                sidb_surface<Lyt> lyt_defect{};

                layout.foreach_cell([this, &lyt_defect](const auto& cell)
                                    { lyt_defect.assign_cell_type(cell, layout.get_cell_type(cell)); });

                // assign defect to layout
                lyt_defect.assign_sidb_defect(defect, params.defect);
                // conduct simulation with defect
                auto simulation_result_defect = quickexact(lyt_defect, params_defect);

                const auto min_energy_defect          = minimum_energy(simulation_result_defect.charge_distributions);
                uint64_t   charge_index_defect_layout = 0;

                // get the charge index of the ground state
                for (const auto& lyt_simulation_with_defect : simulation_result_defect.charge_distributions)
                {
                    if (std::fabs(round_to_n_decimal_places(lyt_simulation_with_defect.get_system_energy(), 6) -
                                  round_to_n_decimal_places(min_energy_defect, 6)) <
                        std::numeric_limits<double>::epsilon())
                    {
                        lyt_simulation_with_defect.charge_distribution_to_index_general();
                        charge_index_defect_layout = lyt_simulation_with_defect.get_charge_index_and_base().first;
                    }
                }

                // defect changes the ground state, i.e., the charge index is changed compared to the charge
                // distribution without placed defect.
                if (charge_index_defect_layout != charge_index_layout)
                {
                    auto distance = std::numeric_limits<double>::max();
                    layout.foreach_cell(
                        [this, &defect, &distance](const auto& cell)
                        {
                            if (sidb_nanometer_distance<Lyt>(layout, cell, defect) < distance)
                            {
                                distance = sidb_nanometer_distance<Lyt>(layout, cell, defect);
                            }
                        });

                    // the distance is larger than the current maximum one.
                    if (distance > avoidance_distance)
                    {
                        max_defect_position = defect;
                        avoidance_distance  = distance;
                    }
                }
            }
        };

        // Apply the process_defect function to each defect using std::for_each
        std::for_each(FICTION_EXECUTION_POLICY_PAR_UNSEQ defect_cells.cbegin(), defect_cells.cend(), process_defect);

        return {max_defect_position, avoidance_distance};
    }

  private:
    /**
     * SiDB cell-level layout to simulate.
     */
    Lyt layout;
    /**
     * Parameters used for the simulation.
     */
    maximum_defect_influence_distance_params params{};
    /**
     * All allowed defect positions.
     */
    std::vector<typename Lyt::cell> defect_cells{};
    /**
     * Collects all possible defect cell positions within a given layout while avoiding SiDB cells.
     *
     * This function calculates a bounding box around the provided layout, encompassing the area
     * where defect cells can be placed. It then iterates through this bounding box, scanning from
     * top to bottom and left to right, and identifies all valid positions for defect cells. A defect
     * cell can only be placed in locations where there are no SiDB cells.
     */
    void collect_all_defect_cells() noexcept
    {
        // bounding box around the given layout to have north-west and south-east cells.
        bounding_box_2d<Lyt> bb{layout};

        auto nw = bb.get_min();  // north-west cell
        auto se = bb.get_max();  // south-east cell

        // shift nw and se cell by the additional scanning area to cover an area that is larger than the gate area.
        nw.x = nw.x - params.additional_scanning_area.first;
        nw.y = nw.y - params.additional_scanning_area.second;

        se.x = se.x + params.additional_scanning_area.first;
        se.y = se.y + params.additional_scanning_area.second;

        defect_cells = all_coordinates_in_spanned_area(nw, se);
    }
};

}  // namespace detail

/**
 * Calculates the maximum distance at which a given defect can influence the layout's ground state.
 *
 * This function simulates the influence of defects on a SiDB cell-level layout. It computes the
 * maximum influence distance, defined as the minimum distance between any SiDB cell and the given defect, at which the
 * defect can still affect the layout's ground state, potentially altering its behavior, such as gate functionality.
 *
 * @param lyt The SiDB cell-level layout for which the influence distance is being determined.
 * @param sim_params Parameters used to calculate the defect's maximum influence distance.
 * @return Pair with the first element describing the position with maximum distance to the layout where a placed defect
 * can still affect the ground state of the layout. The second entry describes the distance of the defect from the
 * layout.
 */
template <typename Lyt>
std::pair<typename Lyt::cell, double>
maximum_defect_influence_position_and_distance(const Lyt&                                      lyt,
                                               const maximum_defect_influence_distance_params& sim_params = {})
{
    static_assert(is_cell_level_layout_v<Lyt>, "Lyt is not a cell-level layout");
    static_assert(has_sidb_technology_v<Lyt>, "Lyt is not an SiDB layout");
    static_assert(!has_offset_ucoord_v<Lyt>, "Lyt cannot be based on offset coordinates");
    static_assert(!is_charge_distribution_surface_v<Lyt>, "Lyt cannot be a charge distribution surface");

    detail::maximum_defect_influence_position_and_distance_impl<Lyt> p{lyt, sim_params};

    return p.run();
}

}  // namespace fiction

#endif  // FICTION_MAXIMUM_DEFECT_INFLUENCE_POSITION_AND_DISTANCE_HPP
