//
// Created by simon on 17.12.23.
//

#if (FICTION_Z3_SOLVER)

#include "fiction_experiments.hpp"

#include <fiction/algorithms/physical_design/apply_gate_library.hpp>  // layout conversion to cell-level
#include <fiction/algorithms/physical_design/exact.hpp>               // SMT-based physical design of FCN layouts
#include <fiction/algorithms/properties/critical_path_length_and_throughput.hpp>  // critical path and throughput calculations
#include <fiction/algorithms/verification/equivalence_checking.hpp>
#include <fiction/io/network_reader.hpp>  // read networks from files
#include <fiction/io/print_layout.hpp>
#include <fiction/io/write_fgl_layout.hpp>  // writer for SiQAD files (physical simulation)
#include <fiction/layouts/clocked_layout.hpp>
#include <fiction/layouts/clocking_scheme.hpp>
#include <fiction/networks/technology_network.hpp>            // technology-mapped network type
#include <fiction/technology/area.hpp>                        // area requirement calculations
#include <fiction/technology/technology_mapping_library.hpp>  // pre-defined gate types for technology mapping
#include <fiction/traits.hpp>                                 // traits for type-checking
#include <fiction/types.hpp>                                  // pre-defined types suitable for the FCN domain
#include <fiction/utils/name_utils.hpp>

#include <fmt/format.h>                                        // output formatting
#include <lorina/genlib.hpp>                                   // Genlib file parsing
#include <lorina/lorina.hpp>                                   // Verilog/BLIF/AIGER/... file parsing
#include <mockturtle/algorithms/cut_rewriting.hpp>             // logic optimization with cut rewriting
#include <mockturtle/algorithms/mapper.hpp>                    // Technology mapping on the logic level
#include <mockturtle/algorithms/node_resynthesis/xag_npn.hpp>  // NPN databases for cut rewriting of XAGs and AIGs
#include <mockturtle/io/genlib_reader.hpp>                     // call-backs to read Genlib files into gate libraries
#include <mockturtle/io/verilog_reader.hpp>                    // call-backs to read Verilog files into networks
#include <mockturtle/io/write_blif.hpp>
#include <mockturtle/networks/aig.hpp>        // XOR-AND-inverter graphs
#include <mockturtle/networks/klut.hpp>       // k-LUT network
#include <mockturtle/networks/xag.hpp>        // XOR-AND-inverter graphs
#include <mockturtle/utils/tech_library.hpp>  // technology library utils
#include <mockturtle/views/depth_view.hpp>    // to determine network levels

#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
#include <iomanip>



using clk_lyt = fiction::clocked_layout<fiction::cartesian_layout<fiction::offset::ucoord_t>>;


int main()  // NOLINT
{
    using gate_lyt = fiction::cart_gate_clk_lyt;

    experiments::experiment<std::string, uint32_t, uint32_t, uint32_t, uint32_t, uint64_t, uint64_t, uint64_t, uint32_t,
                            uint32_t, uint64_t, uint64_t, double, std::string>
        clk_scheme_exp{"clk_scheme",
                       "benchmark",
                       "inputs",
                       "outputs",
                       "initial nodes",
                       "initial depth",
                       "layout width (in tiles)",
                       "layout height (in tiles)",
                       "layout area (in tiles)",
                       "gates",
                       "wires",
                       "critical path",
                       "throughput",
                       "runtime (in sec)",
                       "equivalent"};

    auto new_clocking_scheme =
        fiction::clocking_scheme<fiction::tile<gate_lyt>>("new", [](auto) { return 0; }, 2, 2, 4, true);

    // parameters for SMT-based physical design
    fiction::exact_physical_design_params<gate_lyt> exact_params{};
    exact_params.scheme    = std::make_shared<fiction::clocking_scheme<fiction::tile<gate_lyt>>>(new_clocking_scheme);
    exact_params.crossings = true;
    exact_params.border_io = true;
    exact_params.desynchronize = true;
    exact_params.timeout       = 3'600'000;  // 1h in ms
    //exact_params.upper_bound_area = 12;

    int max_iter = 1;
    // exact_params.fixed_size    = true;
    fiction::exact_physical_design_stats exact_stats{};

    static constexpr const uint64_t bench_select = fiction_experiments::mux21;

    // Create an array of 16 numbers
    //std::array<int, 16> numbers = {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3};
    std::array<int, 16> numbers = {0, 1, 2, 3, 1, 2, 3, 0, 2, 3, 0, 1, 3, 0, 1, 2};


    auto start_time = std::chrono::high_resolution_clock::now();
    for (const auto& benchmark : fiction_experiments::all_benchmarks(bench_select))
    {
        fmt::print("[i] processing {}\n", benchmark);
        std::ostringstream                        os{};
        fiction::network_reader<fiction::tec_ptr> reader{fiction_experiments::benchmark_path(benchmark), os};
        const auto                                nets = reader.get_networks();
        const auto net = fiction::convert_network<fiction::tec_nt, fiction::tec_nt>(*nets.front());

        // compute depth
        mockturtle::depth_view depth_net{net};

        int it = 0;

        // Use the next_permutation function to generate all permutations
        do {
            int i = 0;
            // Print the current permutation
            for (int y = 0; y < 4; ++y)
            {
                for (int x = 0; x < 4; ++x)
                {
                    new_clocking_scheme.override_clock_number({x, y}, numbers[i]);
                    i++;
                }
            }

            // perform layout generation with an SMT-based exact algorithm
            const auto gate_level_layout = fiction::exact<gate_lyt>(net, exact_params, &exact_stats);

            if (gate_level_layout.has_value())
            {
                fiction::write_fgl_layout(*gate_level_layout, "test.fgl");
                // check equivalence
                fiction::equivalence_checking_stats eq_stats{};
                fiction::equivalence_checking<fiction::tec_nt, gate_lyt>(net, *gate_level_layout, &eq_stats);

                const std::string eq_result = eq_stats.eq == fiction::eq_type::STRONG ? "STRONG" :
                                              eq_stats.eq == fiction::eq_type::WEAK   ? "WEAK" :
                                                                                        "NO";

                // compute critical path and throughput
                fiction::critical_path_length_and_throughput_stats cp_tp_stats{};
                fiction::critical_path_length_and_throughput(*gate_level_layout, &cp_tp_stats);

                fiction::restore_names(net, *gate_level_layout);

                // log results
                clk_scheme_exp(benchmark, net.num_pis(), net.num_pos(), net.num_gates(), depth_net.depth(),
                               gate_level_layout->x() + 1, gate_level_layout->y() + 1,
                               (gate_level_layout->x() + 1) * (gate_level_layout->y() + 1),
                               gate_level_layout->num_gates(), gate_level_layout->num_wires(),
                               cp_tp_stats.critical_path_length, cp_tp_stats.throughput,
                               mockturtle::to_seconds(exact_stats.time_total), eq_result);
                clk_scheme_exp.table();
                clk_scheme_exp.save();
            }

            it++;
        } while (std::next_permutation(numbers.begin(), numbers.end()) && (numbers[0] == 0) && (it <= max_iter));
    }
    // Record the end time
    auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate the duration
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    // Print the execution time
    std::cout << "Execution Time: " << duration.count() << " milliseconds\n";

    return EXIT_SUCCESS;
}

#else  // FICTION_Z3_SOLVER

#include <cstdlib>
#include <iostream>

int main()  // NOLINT
{
    std::cerr << "[e] Z3 solver is not available, please install Z3 and recompile the code" << std::endl;

    return EXIT_FAILURE;
}

#endif  // FICTION_Z3_SOLVER
