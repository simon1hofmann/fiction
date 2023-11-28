//
// Created by Jan Drewniok on 11.09.23.
//

#ifndef FICTION_IS_OPERATIONAL_HPP
#define FICTION_IS_OPERATIONAL_HPP

#include "fiction/algorithms/iter/bdl_input_iterator.hpp"
#include "fiction/algorithms/simulation/sidb/can_positive_charges_occur.hpp"
#include "fiction/algorithms/simulation/sidb/detect_bdl_pairs.hpp"
#include "fiction/algorithms/simulation/sidb/exhaustive_ground_state_simulation.hpp"
#include "fiction/algorithms/simulation/sidb/quickexact.hpp"
#include "fiction/algorithms/simulation/sidb/quicksim.hpp"
#include "fiction/algorithms/simulation/sidb/sidb_simulation_engine.hpp"
#include "fiction/algorithms/simulation/sidb/sidb_simulation_parameters.hpp"
#include "fiction/traits.hpp"
#include "fiction/types.hpp"
#include "fiction/utils/truth_table_utils.hpp"

#include <kitty/bit_operations.hpp>
#include <kitty/traits.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace fiction
{
/**
 * Possible operational status of a layout.
 */
enum class operational_status
{
    /**
     * The layout is operational.
     */
    OPERATIONAL,
    /**
     * The layout is non-operational.
     */
    NON_OPERATIONAL
};
/**
 * Parameters for the `is_operational` algorithm.
 */
struct is_operational_params
{
    /**
     * The simulation parameters for the physical simulation of the ground state.
     */
    sidb_simulation_parameters simulation_parameter{};
    /**
     * The simulation engine to be used for the operational domain computation.
     */
    sidb_simulation_engine sim_engine{sidb_simulation_engine::QUICKEXACT};
    /**
     * Parameters for the BDL pair detection algorithms.
     */
    detect_bdl_pairs_params bdl_params{};
};

namespace detail
{
/**
 * Implementation of the `is_operational` algorithm for a given gate layout.
 *
 * This class provides an implementation of the `is_operational` algorithm for
 * a specified gate layout and parameters. It checks whether the gate layout is operational
 * by simulating its behavior for different input combinations and comparing the results
 * to expected outputs from a truth table.
 *
 * @tparam Lyt SiDB cell-level layout type.
 * @tparam TT The type of the truth table specifying the gate behavior.
 */
template <typename Lyt, typename TT>
class is_operational_impl
{
  public:
    /**
     * Constructor to initialize the algorithm with a layout and parameters.
     *
     * @param lyt The SiDB cell-level layout to be checked.
     * @param spec Expected Boolean function of the layout given as a multi-output truth table.
     * @param params Parameters for the `is_operational` algorithm.
     */
    is_operational_impl(const Lyt& lyt, const std::vector<TT>& tt, const is_operational_params& params) :
            layout{lyt},
            truth_table{tt},
            parameters{params},
            output_bdl_pairs(detect_bdl_pairs(layout, sidb_technology::cell_type::OUTPUT, parameters.bdl_params)),
            bii(bdl_input_iterator<Lyt>{layout, parameters.bdl_params})
    {}

    /**
     * Run the `is_operational` algorithm.
     *
     * This function executes the operational status checking algorithm for the gate layout
     * and parameters provided during initialization.
     *
     * @return The operational status of the gate layout (either `OPERATIONAL` or `NON_OPERATIONAL`).
     */
    [[nodiscard]] operational_status run() noexcept
    {
        assert(!output_bdl_pairs.empty() && "No output cell provided.");
        assert((truth_table.size() == output_bdl_pairs.size()) &&
               "Number of truth tables and output BDL pairs does not match");

        // number of different input combinations
        for (auto i = 0u; i < truth_table.front().num_bits(); ++i, ++bii)
        {
            ++simulator_invocations;

            // if positively charged SiDBs can occur, the SiDB layout is considered as non-operational
            if (can_positive_charges_occur(*bii, parameters.simulation_parameter))
            {
                return operational_status::NON_OPERATIONAL;
            }

            // performs physical simulation of a given SiDB layout at a given input combination
            const auto simulation_results = physical_simulation_of_layout(bii);

            // if no physically valid charge distributions were found, the layout is non-operational
            if (simulation_results.charge_distributions.empty())
            {
                return operational_status::NON_OPERATIONAL;
            }

            // find the ground state, which is the charge distribution with the lowest energy
            const auto ground_state = std::min_element(
                simulation_results.charge_distributions.cbegin(), simulation_results.charge_distributions.cend(),
                [](const auto& lhs, const auto& rhs) { return lhs.get_system_energy() < rhs.get_system_energy(); });

            // ground state is degenerate
            if ((energy_distribution(simulation_results.charge_distributions).begin()->second) > 1)
            {
                return operational_status::NON_OPERATIONAL;
            }

            // fetch the charge states of the output BDL pair
            for (auto output = 0u; output < output_bdl_pairs.size(); output++)
            {
                const auto charge_state_output_upper = ground_state->get_charge_state(output_bdl_pairs[output].upper);
                const auto charge_state_output_lower = ground_state->get_charge_state(output_bdl_pairs[output].lower);

                // if the output charge states are equal, the layout is not operational
                if (charge_state_output_lower == charge_state_output_upper)
                {
                    return operational_status::NON_OPERATIONAL;
                }

                // if the expected output is 1, the expected charge states are (upper, lower) = (0, -1)
                if (kitty::get_bit(truth_table[output], i))
                {
                    if (charge_state_output_upper != sidb_charge_state::NEUTRAL ||
                        charge_state_output_lower != sidb_charge_state::NEGATIVE)
                    {
                        return operational_status::NON_OPERATIONAL;
                    }
                }
                // if the expected output is 0, the expected charge states are (upper, lower) = (-1, 0)
                else
                {
                    if (charge_state_output_upper != sidb_charge_state::NEGATIVE ||
                        charge_state_output_lower != sidb_charge_state::NEUTRAL)
                    {
                        return operational_status::NON_OPERATIONAL;
                    }
                }
            }
        }

        // if we made it here, the layout is operational
        return operational_status::OPERATIONAL;
    }
    /**
     * Returns the total number of simulator invocations.
     *
     * @return The number of simulator invocations.
     */
    [[nodiscard]] std::size_t get_number_of_simulator_invocations() const noexcept
    {
        return simulator_invocations;
    }

  private:
    /**
     * SiDB cell-level layout.
     */
    const Lyt& layout;
    /**
     * The specification of the layout.
     */
    const std::vector<TT>&
        truth_table;  // TODO implement the matching of multi-input truth table inputs and BDL pair ordering
    /**
     * Parameters for the `is_operational` algorithm.
     */
    is_operational_params parameters;
    /**
     * Output BDL pairs.
     */
    std::vector<bdl_pair<Lyt>> output_bdl_pairs;
    /**
     * Iterator that iterates over all possible input states.
     */
    bdl_input_iterator<Lyt> bii;
    /**
     * Number of simulator invocations.
     */
    std::size_t simulator_invocations{0};
    /**
     * This function conducts physical simulation of the given layout (gate layout with certain input combination). The
     * simulation results are stored in the `sim_result` variable.
     *
     * @param bdl_iterator A reference to a BDL input iterator representing the gate layout at a given input
     * combination. The simulation is performed based on the configuration represented by the iterator.
     * @return Simulation results.
     */
    [[nodiscard]] sidb_simulation_result<Lyt>
    physical_simulation_of_layout(const bdl_input_iterator<Lyt>& bdl_iterator) noexcept
    {
        assert(parameters.simulation_parameter.base == 2 && "base number is set to 3");
        if (parameters.sim_engine == sidb_simulation_engine::EXGS)
        {
            // perform an exhaustive ground state simulation
            return exhaustive_ground_state_simulation(*bdl_iterator, parameters.simulation_parameter);
        }
        if (parameters.sim_engine == sidb_simulation_engine::QUICKSIM)
        {
            // perform a heuristic simulation
            const quicksim_params qs_params{parameters.simulation_parameter, 500, 0.6};
            return quicksim(*bdl_iterator, qs_params);
        }
        if (parameters.sim_engine == sidb_simulation_engine::QUICKEXACT)
        {
            // perform exact simulation
            const quickexact_params<Lyt> quickexact_params{
                parameters.simulation_parameter, fiction::quickexact_params<Lyt>::automatic_base_number_detection::OFF};
            return quickexact(*bdl_iterator, quickexact_params);
        }

        assert(false && "unsupported simulation engine");

        return sidb_simulation_result<Lyt>{};
    }
};

}  // namespace detail

/**
 * Determine the operational status of an SiDB layout.
 *
 * This function checks the operational status of a given gate layout using the `is_operational` algorithm. It
 * determines whether the gate layout is operational and returns the correct result for all \f$ 2^n \f$ input
 * combinations.
 *
 * @tparam Lyt SiDB cell-level layout type.
 * @tparam TT The type of the truth table specifying the layout behavior.
 * @param lyt The SiDB cell-level layout to be checked.
 * @param spec Expected Boolean function of the layout given as a multi-output truth table.
 * @param params Parameters for the `is_operational` algorithm.
 * @return A pair containing the operational status of the gate layout (either `OPERATIONAL` or `NON_OPERATIONAL`) and
 * the number of input combinations tested.
 */
template <typename Lyt, typename TT>
[[nodiscard]] std::pair<operational_status, std::size_t>
is_operational(const Lyt& lyt, const std::vector<TT>& spec, const is_operational_params& params = {}) noexcept
{
    static_assert(is_cell_level_layout_v<Lyt>, "Lyt is not a cell-level layout");
    static_assert(has_sidb_technology_v<Lyt>, "Lyt is not an SiDB layout");
    static_assert(kitty::is_truth_table<TT>::value, "TT is not a truth table");

    assert(lyt.num_pis() > 0 && "skeleton needs input cells");
    assert(lyt.num_pos() > 0 && "skeleton needs output cells");

    assert(!spec.empty());
    // all elements in tts must have the same number of variables
    assert(std::adjacent_find(spec.begin(), spec.end(),
                              [](const auto& a, const auto& b) { return a.num_vars() != b.num_vars(); }) == spec.end());

    detail::is_operational_impl<Lyt, TT> p{lyt, spec, params};

    return {p.run(), p.get_number_of_simulator_invocations()};
}

}  // namespace fiction

#endif  // FICTION_IS_OPERATIONAL_HPP
