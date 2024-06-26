//
// Created by marcel on 07.12.23.
//

#ifndef FICTION_CMD_QUICKSIM_HPP
#define FICTION_CMD_QUICKSIM_HPP

#include <fiction/algorithms/simulation/sidb/minimum_energy.hpp>
#include <fiction/algorithms/simulation/sidb/quicksim.hpp>
#include <fiction/algorithms/simulation/sidb/sidb_simulation_result.hpp>
#include <fiction/traits.hpp>
#include <fiction/types.hpp>
#include <fiction/utils/name_utils.hpp>

#include <alice/alice.hpp>
#include <nlohmann/json.hpp>

#include <any>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <variant>

namespace alice
{
/**
 *
 */
class quicksim_command : public command
{
  public:
    /**
     * Standard constructor. Adds descriptive information, options, and flags.
     *
     * @param e alice::environment that specifies stores etc.
     */
    explicit quicksim_command(const environment::ptr& e) :
            command(e,
                    "The QuickSim algorithm is a heuristic electrostatic ground state simulation algorithm for SiDB "
                    "layouts. It determines physically valid charge configurations (with minimal energy). Depending on "
                    "the simulation parameters, the ground state is found with a certain probability after one run.")
    {
        add_option("--epsilon_r,-e", physical_params.epsilon_r, "Electric permittivity of the substrate (unit-less)",
                   true);
        add_option("--lambda_tf,-l", physical_params.lambda_tf, "Thomas-Fermi screening distance (unit: nm)", true);
        add_option("--mu_minus,-m", physical_params.mu_minus, "Energy transition level (0/-) (unit: eV)", true);
        add_option("--iterations,-i", params.iteration_steps, "Number of iterations to run the simulation for", true);
        add_option("--alpha,-a", params.alpha,
                   "alpha parameter (should be reduced if not charge distribution can be determined)", true);
    }

  protected:
    /**
     * Function to perform the simulation call.
     */
    void execute() override
    {
        // reset sim result
        sim_result_100      = {};
        sim_result_111      = {};
        min_energy          = std::numeric_limits<double>::infinity();
        is_sidb_100_lattice = true;

        if (physical_params.epsilon_r <= 0)
        {
            env->out() << "[e] epsilon_r must be positive" << std::endl;
            reset_params();
            return;
        }
        if (physical_params.lambda_tf <= 0)
        {
            env->out() << "[e] lambda_tf must be positive" << std::endl;
            reset_params();
            return;
        }
        if (params.alpha <= 0)
        {
            env->out() << "[e] alpha must be positive" << std::endl;
            reset_params();
            return;
        }

        auto& s = store<fiction::cell_layout_t>();

        // error case: empty cell layout store
        if (s.empty())
        {
            env->out() << "[w] no cell layout in store" << std::endl;
            reset_params();
            return;
        }

        const auto get_name = [](auto&& lyt_ptr) -> std::string { return fiction::get_name(*lyt_ptr); };

        const auto quicksim = [this, &get_name](auto&& lyt_ptr)
        {
            using Lyt = typename std::decay_t<decltype(lyt_ptr)>::element_type;

            if constexpr (fiction::has_sidb_technology_v<Lyt>)
            {
                if constexpr (fiction::is_charge_distribution_surface_v<Lyt>)
                {
                    env->out() << fmt::format(
                                      "[w] {} already possesses a charge distribution; no simulation is conducted",
                                      get_name(lyt_ptr))
                               << std::endl;
                }
                else
                {
                    params.simulation_parameters = physical_params;

                    if constexpr (fiction::is_sidb_lattice_100_v<Lyt>)
                    {
                        is_sidb_100_lattice = true;
                        sim_result_100      = fiction::quicksim(*lyt_ptr, params);
                    }
                    else if constexpr (fiction::is_sidb_lattice_111_v<Lyt>)
                    {
                        is_sidb_100_lattice = false;
                        sim_result_111      = fiction::quicksim(*lyt_ptr, params);
                    }

                    else
                    {
                        env->out() << "[e] no valid lattice orientation" << std::endl;
                        return;
                    }

                    if (sim_result_100.charge_distributions.empty() && sim_result_111.charge_distributions.empty())
                    {
                        env->out() << fmt::format("[e] no stable charge distribution could be determined for {}",
                                                  get_name(lyt_ptr))
                                   << std::endl;
                    }
                    else
                    {
                        if constexpr (fiction::is_sidb_lattice_100_v<Lyt>)
                        {
                            const auto min_energy_distr =
                                fiction::minimum_energy_distribution(sim_result_100.charge_distributions.cbegin(),
                                                                     sim_result_100.charge_distributions.cend());

                            min_energy = min_energy_distr->get_system_energy();
                            store<fiction::cell_layout_t>().extend() =
                                std::make_shared<fiction::cds_sidb_100_cell_clk_lyt>(*min_energy_distr);
                        }
                        else if constexpr (fiction::is_sidb_lattice_111_v<Lyt>)
                        {
                            const auto min_energy_distr =
                                fiction::minimum_energy_distribution(sim_result_111.charge_distributions.cbegin(),
                                                                     sim_result_111.charge_distributions.cend());

                            min_energy = min_energy_distr->get_system_energy();
                            store<fiction::cell_layout_t>().extend() =
                                std::make_shared<fiction::cds_sidb_111_cell_clk_lyt>(*min_energy_distr);
                        }
                    }
                }
            }
            else
            {
                env->out() << fmt::format("[e] {} is not an SiDB layout", get_name(lyt_ptr)) << std::endl;
            }
        };

        std::visit(quicksim, s.current());

        reset_params();
    }

  private:
    /**
     * Physical parameters for the simulation.
     */
    fiction::sidb_simulation_parameters physical_params{2, -0.32, 5.6, 5.0};
    /**
     * QuickSim parameters.
     */
    fiction::quicksim_params params{};
    /**
     * Simulation result for H-Si(100)-2x1 surface.
     */
    fiction::sidb_simulation_result<fiction::sidb_100_cell_clk_lyt> sim_result_100{};
    /**
     * Simulation result for H-Si(111)-1x1 surface.
     */
    fiction::sidb_simulation_result<fiction::sidb_111_cell_clk_lyt> sim_result_111{};
    /**
     * Minimum energy.
     */
    double min_energy{std::numeric_limits<double>::infinity()};

    bool is_sidb_100_lattice = true;

    /**
     * Logs the resulting information in a log file.
     *
     * @return JSON object containing details about the simulation.
     */
    [[nodiscard]] nlohmann::json log() const override
    {
        try
        {
            if (is_sidb_100_lattice)
            {
                return nlohmann::json{
                    {"Algorithm name", sim_result_100.algorithm_name},
                    {"Simulation runtime", sim_result_100.simulation_runtime.count()},
                    {"Physical parameters",
                     {{"epsilon_r", sim_result_100.simulation_parameters.epsilon_r},
                      {"lambda_tf", sim_result_100.simulation_parameters.lambda_tf},
                      {"mu_minus", sim_result_100.simulation_parameters.mu_minus}}},
                    {"Lowest state energy (eV)", min_energy},
                    {"Number of stable states", sim_result_100.charge_distributions.size()},
                    {"Iteration steps",
                     std::any_cast<uint64_t>(sim_result_100.additional_simulation_parameters.at("iteration_steps"))},
                    {"alpha", std::any_cast<double>(sim_result_100.additional_simulation_parameters.at("alpha"))}};
            }
            return nlohmann::json{
                {"Algorithm name", sim_result_111.algorithm_name},
                {"Simulation runtime", sim_result_111.simulation_runtime.count()},
                {"Physical parameters",
                 {{"epsilon_r", sim_result_111.simulation_parameters.epsilon_r},
                  {"lambda_tf", sim_result_111.simulation_parameters.lambda_tf},
                  {"mu_minus", sim_result_111.simulation_parameters.mu_minus}}},
                {"Lowest state energy (eV)", min_energy},
                {"Number of stable states", sim_result_111.charge_distributions.size()},
                {"Iteration steps",
                 std::any_cast<uint64_t>(sim_result_111.additional_simulation_parameters.at("iteration_steps"))},
                {"alpha", std::any_cast<double>(sim_result_111.additional_simulation_parameters.at("alpha"))}};
        }
        catch (...)
        {
            return nlohmann::json{};
        }
    }
    /**
     * Resets the parameters to their default values.
     */
    void reset_params()
    {
        physical_params = fiction::sidb_simulation_parameters{2, -0.32, 5.6, 5.0};
        params          = {};
    }
};

ALICE_ADD_COMMAND(quicksim, "Simulation")

}  // namespace alice

#endif  // FICTION_CMD_QUICKSIM_HPP
