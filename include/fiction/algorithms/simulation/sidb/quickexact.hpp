//
// Created by Jan Drewniok on 18.12.22.
//

#ifndef FICTION_QUICKEXACT_HPP
#define FICTION_QUICKEXACT_HPP

#include "fiction/algorithms/iter/gray_code_iterator.hpp"
#include "fiction/algorithms/simulation/sidb/energy_distribution.hpp"
#include "fiction/algorithms/simulation/sidb/minimum_energy.hpp"
#include "fiction/algorithms/simulation/sidb/sidb_simulation_engine.hpp"
#include "fiction/algorithms/simulation/sidb/sidb_simulation_parameters.hpp"
#include "fiction/algorithms/simulation/sidb/sidb_simulation_result.hpp"
#include "fiction/technology/charge_distribution_surface.hpp"
#include "fiction/traits.hpp"

#include <fmt/format.h>
#include <mockturtle/utils/stopwatch.hpp>

#include <algorithm>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace fiction
{
/**
 * This struct stores the parameters for the *QuickExact* algorithm.
 */
template <typename Lyt>
struct quickexact_params
{
    /**
     * Modes to use for the *QuickExact* algorithm.
     */
    enum class automatic_base_number_detection
    {
        /**
         * Simulation is conducted with the required base number (i.e, if positively charged SiDBs can occur, three
         * state simulation is conducted).
         */
        ON,
        /**
         * The base number from the physical parameter is used for the simulation.
         */
        OFF
    };
    /**
     * All parameters for physical SiDB simulations.
     */
    sidb_simulation_parameters physical_parameters{};
    /**
     * If `ON`, *QuickExact* checks which base number is required for the simulation, i.e., whether 3-state is
     * necessary or 2-state simulation is sufficient.
     */
    automatic_base_number_detection base_number_detection = automatic_base_number_detection::ON;
    /**
     * Local external electrostatic potentials (e.g locally applied electrodes).
     */
    std::unordered_map<cell<Lyt>, double> local_external_potential = {};
    /**
     * Global external electrostatic potential. Value is applied on each cell in the layout.
     */
    double global_potential = 0;
};

namespace detail
{

template <typename Lyt>
class quickexact_impl
{
  public:
    quickexact_impl(const Lyt& lyt, const quickexact_params<Lyt>& parameter) :
            layout{lyt},
            charge_lyt{lyt, parameter.physical_parameters, sidb_charge_state::NEGATIVE},
            params{parameter}
    {}

    sidb_simulation_result<Lyt> run() noexcept
    {
        result.algorithm_name      = "QuickExact";
        result.physical_parameters = params.physical_parameters;

        mockturtle::stopwatch<>::duration time_counter{};
        {
            const mockturtle::stopwatch stop{time_counter};

            initialize_charge_layout();

            // Determine if three state simulation (i.e., positively charged SiDBs can occur) is required.
            const bool three_state_simulation_required =
                (params.base_number_detection == quickexact_params<Lyt>::automatic_base_number_detection::ON &&
                 charge_lyt.is_three_state_simulation_required()) ||
                (params.base_number_detection == quickexact_params<Lyt>::automatic_base_number_detection::OFF &&
                 params.physical_parameters.base == 3);

            // If layout has at least two SiDBs, all SiDBs that have to be negatively charged are erased from the
            // layout.
            if (number_of_sidbs > 1)
            {
                generate_layout_without_negative_sidbs();

                // If the layout consists of SiDBs that do not need to be negatively charged.
                if (!all_sidbs_in_lyt_without_negative_preassigned_ones.empty())
                {
                    // The first cell from all_sidbs_in_lyt_without_negative_preassigned_ones is chosen as the
                    // dependent-cell to initialize the layout (pre-assigned negatively charged SiDBs were erased with
                    // generate_layout_without_negative_sidbs). All SiDBs are set to neutrally charged.
                    charge_distribution_surface charge_lyt_with_assigned_dependent_cell{
                        layout, params.physical_parameters, sidb_charge_state::NEUTRAL,
                        all_sidbs_in_lyt_without_negative_preassigned_ones[0]};

                    charge_lyt_with_assigned_dependent_cell.assign_local_external_potential(
                        params.local_external_potential);
                    charge_lyt_with_assigned_dependent_cell.assign_global_external_potential(params.global_potential);

                    if constexpr (has_get_sidb_defect_v<Lyt>)
                    {
                        for (const auto& [cell, defect] : real_placed_defects)
                        {
                            charge_lyt_with_assigned_dependent_cell.add_sidb_defect_to_potential_landscape(cell,
                                                                                                           defect);
                        }
                    }

                    // IMPORTANT: The pre-assigned negatively charged SiDBs (they have to be negatively charged to
                    // fulfill the population stability) are considered as negatively charged defects in the layout.
                    // Hence, there are no "real" defects assigned, but in order to set some SiDBs with a fixed negative
                    // charge, this way of implementation is chosen.
                    for (const auto& cell : preassigned_negative_sidbs)
                    {
                        charge_lyt_with_assigned_dependent_cell.add_sidb_defect_to_potential_landscape(
                            cell, sidb_defect{sidb_defect_type::UNKNOWN, -1,
                                              charge_lyt_with_assigned_dependent_cell.get_phys_params().epsilon_r,
                                              charge_lyt_with_assigned_dependent_cell.get_phys_params().lambda_tf});
                    }

                    // Update all local potentials, system energy and physically validity. The Flag is set to "Variable"
                    // to allow dependent cell to change its charge state based on the N-1 SiDBs to fulfill the local
                    // population stability in its position.
                    charge_lyt_with_assigned_dependent_cell.update_after_charge_change(dependent_cell_mode::VARIABLE);

                    // If no positively charged SiDB can occur in the layout.
                    if (!three_state_simulation_required)
                    {
                        result.additional_simulation_parameters.emplace_back("base_number", uint64_t{2});
                        two_state_simulation(charge_lyt_with_assigned_dependent_cell);
                    }
                    // If positively charged SiDBs can occur in the layout, 3-state simulation is conducted.
                    else
                    {
                        result.additional_simulation_parameters.emplace_back("base_number", uint64_t{3});
                        three_state_simulation(charge_lyt_with_assigned_dependent_cell);
                    }
                }

                // If the layout consists of only pre-assigned negatively-charged SiDBs
                // (i.e., only SiDBs that are far away from each other).
                else if (all_sidbs_in_lyt_without_negative_preassigned_ones.empty())
                {
                    charge_distribution_surface<Lyt> charge_lyt_copy{charge_lyt};
                    if constexpr (has_get_sidb_defect_v<Lyt>)
                    {
                        for (const auto& [cell, defect] : real_placed_defects)
                        {
                            charge_lyt_copy.assign_sidb_defect(cell, defect);
                        }
                    }
                    result.charge_distributions.push_back(charge_lyt_copy);
                }
            }
            // If there is only one SiDB in the layout, this single SiDB can be neutrally or even positively charged due
            // to external potentials or defects.
            else if (number_of_sidbs == 1)
            {
                if (three_state_simulation_required)
                {
                    charge_lyt.assign_base_number(3);
                }
                else
                {
                    charge_lyt.assign_base_number(2);
                }

                // A check is performed to see if the charge index is still below the maximum charge index. If not, the
                // charge index is increased and the corresponding charge distribution is checked for physical validity.
                while (charge_lyt.get_charge_index_and_base().first < charge_lyt.get_max_charge_index())
                {
                    if (charge_lyt.is_physically_valid())
                    {
                        charge_distribution_surface<Lyt> charge_lyt_copy{charge_lyt};
                        if constexpr (has_get_sidb_defect_v<Lyt>)
                        {
                            for (const auto& [cell, defect] : real_placed_defects)
                            {
                                charge_lyt_copy.assign_sidb_defect(cell, defect);
                            }
                        }
                        result.charge_distributions.push_back(charge_lyt_copy);
                    }

                    charge_lyt.increase_charge_index_by_one(
                        dependent_cell_mode::VARIABLE);  // "Variable" allows that the charge state of the dependent
                                                         // cell is automatically changed based on the new charge
                                                         // distribution.
                }

                if (charge_lyt.is_physically_valid())
                {
                    charge_distribution_surface<Lyt> charge_lyt_copy{charge_lyt};
                    if constexpr (has_get_sidb_defect_v<Lyt>)
                    {
                        for (const auto& [cell, defect] : real_placed_defects)
                        {
                            charge_lyt_copy.assign_sidb_defect(cell, defect);
                        }
                    }
                    result.charge_distributions.push_back(charge_lyt_copy);
                }
            }

            for (const auto& cell : preassigned_negative_sidbs)
            {
                layout.assign_cell_type(cell, Lyt::cell_type::NORMAL);
            }
        }

        result.simulation_runtime = time_counter;

        return result;
    }

  private:
    /**
     * Layout to simulate.
     */
    Lyt layout;
    /**
     * Charge distribution surface.
     */
    charge_distribution_surface<Lyt> charge_lyt{};
    /**
     * Parameters used for the simulation.
     */
    quickexact_params<Lyt> params{};
    /**
     * Indices of all SiDBs that are pre-assigned to be negatively charged in a physically valid layout.
     */
    std::vector<int64_t> preassigned_negative_sidb_indices{};
    /**
     * All SiDBs that are pre-assigned to be negatively charged in a physically valid layout.
     */
    std::vector<typename Lyt::cell> preassigned_negative_sidbs{};
    /**
     * All SiDBs of the layout but without the negatively-charged SiDBs.
     */
    std::vector<typename Lyt::cell> all_sidbs_in_lyt_without_negative_preassigned_ones{};
    /**
     * Collection of defects that are placed in addition to the SiDBs.
     */
    std::unordered_map<typename Lyt::cell, const sidb_defect> real_placed_defects{};
    /**
     * Number of SiDBs of the input layout.
     */
    uint64_t number_of_sidbs{};
    /**
     * Simulation results.
     */
    sidb_simulation_result<Lyt> result{};

    /**
     * This function conducts 2-state physical simulation (negative, neutral).
     *
     * @param charge_layout Initialized charge layout.
     */
    void two_state_simulation(charge_distribution_surface<Lyt>& charge_layout) noexcept
    {
        charge_layout.assign_base_number(2);
        uint64_t previous_charge_index = 0;

        gray_code_iterator gci{0};

        for (gci = 0; gci <= charge_layout.get_max_charge_index(); ++gci)
        {
            charge_layout.assign_charge_index_by_gray_code(*gci, previous_charge_index, dependent_cell_mode::VARIABLE,
                                                           energy_calculation::KEEP_OLD_ENERGY_VALUE,
                                                           charge_distribution_history::CONSIDER);

            previous_charge_index = *gci;

            if (charge_layout.is_physically_valid())
            {
                charge_distribution_surface<Lyt> charge_lyt_copy{charge_layout};
                charge_lyt_copy.recompute_system_energy();

                // The pre-assigned negatively-charged SiDBs are added to the final layout.
                for (const auto& cell : preassigned_negative_sidbs)
                {
                    charge_lyt_copy.add_sidb(cell, sidb_charge_state::NEGATIVE);
                }

                if constexpr (has_get_sidb_defect_v<Lyt>)
                {
                    for (const auto& [cell, defect] : real_placed_defects)
                    {
                        charge_lyt_copy.assign_sidb_defect(cell, defect);
                    }
                }
                result.charge_distributions.push_back(charge_lyt_copy);
            }
        }

        // The cells of the pre-assigned negatively-charged SiDBs are added to the cell level layout.
        for (const auto& cell : preassigned_negative_sidbs)
        {
            layout.assign_cell_type(cell, Lyt::cell_type::NORMAL);
        }
    }
    /**
     * This function conducts 3-state physical simulation (negative, neutral, positive).
     *
     * @param charge_layout Initialized charge layout.
     */
    void three_state_simulation(charge_distribution_surface<Lyt>& charge_layout) noexcept
    {
        charge_layout.assign_all_charge_states(sidb_charge_state::NEGATIVE);
        charge_layout.update_after_charge_change();
        // Not executed to detect if 3-state simulation is required, but to detect the SiDBs that could be positively
        // charged (important to speed up the simulation).
        charge_layout.is_three_state_simulation_required();
        charge_layout.update_after_charge_change(dependent_cell_mode::VARIABLE);

        while (charge_layout.get_charge_index_and_base().first < charge_layout.get_max_charge_index())
        {
            while (charge_layout.get_charge_index_of_sub_layout() < charge_layout.get_max_charge_index_sub_layout())
            {
                if (charge_layout.is_physically_valid())
                {
                    charge_distribution_surface<Lyt> charge_lyt_copy{charge_layout};
                    charge_lyt_copy.recompute_system_energy();

                    // The pre-assigned negatively-charged SiDBs are added to the final layout.
                    for (const auto& cell : preassigned_negative_sidbs)
                    {
                        charge_lyt_copy.add_sidb(cell, sidb_charge_state::NEGATIVE);
                    }

                    if constexpr (has_get_sidb_defect_v<Lyt>)
                    {
                        for (const auto& [cell, defect] : real_placed_defects)
                        {
                            charge_lyt_copy.assign_sidb_defect(cell, defect);
                        }
                    }

                    result.charge_distributions.push_back(charge_lyt_copy);
                }

                charge_layout.increase_charge_index_of_sub_layout_by_one(
                    dependent_cell_mode::VARIABLE, energy_calculation::KEEP_OLD_ENERGY_VALUE,
                    charge_distribution_history::CONSIDER,
                    exhaustive_sidb_simulation_engine::QUICKEXACT);  // "false" allows that the charge state of the
                                                                     // dependent cell is automatically changed based on
                                                                     // the new charge distribution.
            }

            if (charge_layout.is_physically_valid())
            {
                charge_distribution_surface<Lyt> charge_lyt_copy{charge_layout};
                charge_lyt_copy.recompute_system_energy();

                for (const auto& cell : preassigned_negative_sidbs)
                {
                    charge_lyt_copy.add_sidb(cell, sidb_charge_state::NEGATIVE);
                }

                if constexpr (has_get_sidb_defect_v<Lyt>)
                {
                    for (const auto& [cell, defect] : real_placed_defects)
                    {
                        charge_lyt_copy.assign_sidb_defect(cell, defect);
                    }
                }

                result.charge_distributions.push_back(charge_lyt_copy);
            }

            if (charge_layout.get_max_charge_index_sub_layout() != 0)
            {
                charge_layout.reset_charge_index_sub_layout();
            }

            charge_layout.increase_charge_index_by_one(
                dependent_cell_mode::VARIABLE, energy_calculation::KEEP_OLD_ENERGY_VALUE,
                charge_distribution_history::CONSIDER,
                exhaustive_sidb_simulation_engine::QUICKEXACT);  // "false" allows that the charge state of the
                                                                 // dependent cell is automatically changed based on the
                                                                 // new charge distribution.
        }

        // charge configurations of the sublayout are iterated
        while (charge_layout.get_charge_index_of_sub_layout() < charge_layout.get_max_charge_index_sub_layout())
        {
            if (charge_layout.is_physically_valid())
            {
                charge_distribution_surface<Lyt> charge_lyt_copy{charge_layout};
                charge_lyt_copy.recompute_system_energy();

                // The pre-assigned negatively-charged SiDBs are added to the final layout.
                for (const auto& cell : preassigned_negative_sidbs)
                {
                    charge_lyt_copy.add_sidb(cell, sidb_charge_state::NEGATIVE);
                }

                if constexpr (has_get_sidb_defect_v<Lyt>)
                {
                    for (const auto& [cell, defect] : real_placed_defects)
                    {
                        charge_lyt_copy.assign_sidb_defect(cell, defect);
                    }
                }

                result.charge_distributions.push_back(charge_lyt_copy);
            }

            charge_layout.increase_charge_index_of_sub_layout_by_one(
                dependent_cell_mode::VARIABLE, energy_calculation::KEEP_OLD_ENERGY_VALUE,
                charge_distribution_history::CONSIDER, exhaustive_sidb_simulation_engine::QUICKEXACT);
        }

        if (charge_layout.is_physically_valid())
        {
            charge_distribution_surface<Lyt> charge_lyt_copy{charge_layout};

            for (const auto& cell : preassigned_negative_sidbs)
            {
                charge_lyt_copy.add_sidb(cell, sidb_charge_state::NEGATIVE);
            }

            if constexpr (has_get_sidb_defect_v<Lyt>)
            {
                for (const auto& [cell, defect] : real_placed_defects)
                {
                    charge_lyt_copy.assign_sidb_defect(cell, defect);
                }
            }

            result.charge_distributions.push_back(charge_lyt_copy);
        }

        for (const auto& cell : preassigned_negative_sidbs)
        {
            layout.assign_cell_type(cell, Lyt::cell_type::NORMAL);
        }
    }
    /**
     * This function is responsible for preparing the charge layout and relevant data structures for the simulation.
     *
     * This function initializes the charge layout within the context of the current simulation. It performs the
     * following tasks:
     *
     * - If the provided layout type `Lyt` supports a `foreach_sidb_defect` method, it iterates through each
     *   defect in the layout.
     *   - If a defect is found, it adds the SiDB defect to the potential landscape.
     * - It assigns the local external potential from the `params.local_external_potential` configuration to the charge
     * layout.
     * - It assigns the global external potential from `params.global_potential` to the charge layout.
     *
     */
    void initialize_charge_layout() noexcept
    {
        if constexpr (has_foreach_sidb_defect_v<Lyt>)
        {
            layout.foreach_sidb_defect(
                [this](const auto& cd)
                {
                    const auto& [cell, defect] = cd;

                    if (defect.type != sidb_defect_type::NONE)
                    {
                        charge_lyt.add_sidb_defect_to_potential_landscape(cell, layout.get_sidb_defect(cell));
                    }
                });
        }

        charge_lyt.assign_local_external_potential(params.local_external_potential);
        charge_lyt.assign_global_external_potential(params.global_potential, dependent_cell_mode::VARIABLE);

        preassigned_negative_sidb_indices = charge_lyt.negative_sidb_detection();
        preassigned_negative_sidbs.reserve(preassigned_negative_sidb_indices.size());

        all_sidbs_in_lyt_without_negative_preassigned_ones = charge_lyt.get_sidb_order();
        real_placed_defects                                = charge_lyt.get_defects();
        // store the number of SiDBs, since the number of active cells changes during simulation.
        number_of_sidbs = charge_lyt.num_cells();
    }
    /**
     * This function is used to generate a layout without the SiDBs that are pre-assigned to be negatively charged in a
     * physically-valid layout.
     */
    void generate_layout_without_negative_sidbs() noexcept
    {
        for (const auto& index : preassigned_negative_sidb_indices)
        {
            const auto cell = charge_lyt.index_to_cell(static_cast<uint64_t>(index));
            preassigned_negative_sidbs.push_back(cell);
            layout.assign_cell_type(cell, Lyt::cell_type::EMPTY);
        }

        // All pre-assigned negatively-charged SiDBs are erased from the
        // all_sidbs_in_lyt_without_negative_preassigned_ones vector.
        all_sidbs_in_lyt_without_negative_preassigned_ones.erase(
            std::remove_if(all_sidbs_in_lyt_without_negative_preassigned_ones.begin(),
                           all_sidbs_in_lyt_without_negative_preassigned_ones.end(),
                           [this](const auto& n)
                           {
                               return std::find(preassigned_negative_sidbs.cbegin(), preassigned_negative_sidbs.cend(),
                                                n) != preassigned_negative_sidbs.cend();
                           }),
            all_sidbs_in_lyt_without_negative_preassigned_ones.cend());
    }
};

}  // namespace detail

/**
 * *QuickExact* is a quick and exact physical simulation algorithm designed specifically for SiDB layouts. It is
 * proposed in \"The Need for Speed: Efficient Exact Simulation of Silicon Dangling Bond Logic\" by J. Drewniok, M.
 * Walter, and R. Wille (https://arxiv.org/abs/2308.04487). It determines all physically valid charge configurations of
 * a given layout, providing a significant performance advantage of more than three orders of magnitude over *ExGS*
 * (`exhaustive_ground_state_simulation`).
 *
 * The performance improvement of *QuickExact* can be attributed to the incorporation of three key ideas:
 *
 * 1. Advanced Negative SiDB Detection: *QuickExact* efficiently identifies SiDBs that require negative charges
 *    in a physically valid charge distribution. By pre-assigned them in advance, the search space is pruned
 *    by a factor of \f$2^k\f$, where k is the number of found SiDBs.
 *
 * 2. Dependent SiDB Selection: The algorithm selects a dependent SiDB, whose charge state is always derived
 *    from its n-1 neighbors. This dependency simplifies the computation process and contributes to the overall
 *    efficiency of *QuickExact*.
 *
 * 3. Gray Code Representation: *QuickExact* employs Gray code to represent and traverse through all charge
 *    configurations. By using Gray code, only one charge state changes at a time, making the computation
 *    of the local electrostatic potential easier.
 *
 * Additionally, *QuickExact* also considers global and local electrostatic potentials, as well as existing defects.
 * This holistic approach ensures an accurate representation of the physical behavior of the SiDB layout.
 *
 * In summary, *QuickExact* combines advanced SiDB charge detection, dependent SiDB selection, and the use of Gray code
 * to achieve outstanding performance and enable efficient simulations of SiDB layouts, even in scenarios where
 * positively-charged SiDBs occur due to small spacing.
 *
 * @tparam Lyt SiDB cell-level layout type.
 * @param lyt Layout to simulate.
 * @param params Parameter required for the simulation.
 * @return Simulation Results.
 */
template <typename Lyt>
[[nodiscard]] sidb_simulation_result<Lyt> quickexact(const Lyt& lyt, const quickexact_params<Lyt>& params = {}) noexcept
{
    static_assert(is_cell_level_layout_v<Lyt>, "Lyt is not a cell-level layout");
    static_assert(has_sidb_technology_v<Lyt>, "Lyt is not an SiDB layout");
    static_assert(has_siqad_coord_v<Lyt>, "Lyt is not based on SiQAD coordinates");

    detail::quickexact_impl<Lyt> p{lyt, params};

    return p.run();
}

}  // namespace fiction

#endif  // FICTION_QUICKEXACT_HPP