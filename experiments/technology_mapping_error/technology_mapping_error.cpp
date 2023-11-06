#include "fiction_experiments.hpp"

#include <fiction/algorithms/physical_design/orthogonal.hpp>  // scalable heuristic for physical design of FCN layouts
#include <fiction/algorithms/properties/critical_path_length_and_throughput.hpp>  // critical path and throughput calculations
#include <fiction/algorithms/verification/equivalence_checking.hpp>               // SAT-based equivalence checking
#include <fiction/io/network_reader.hpp>                                          // read networks from files

#include <fiction/technology/technology_mapping_library.hpp>  // pre-defined gate types for technology mapping
#include <fiction/traits.hpp>                                 // traits for type-checking
#include <fiction/types.hpp>                                  // pre-defined types suitable for the FCN domain

#include <fmt/format.h>                                        // output formatting
#include <lorina/genlib.hpp>                                   // Genlib file parsing
#include <lorina/lorina.hpp>                                   // Verilog/BLIF/AIGER/... file parsing
#include <mockturtle/algorithms/cut_rewriting.hpp>             // logic optimization with cut rewriting
#include <mockturtle/algorithms/mapper.hpp>                    // Technology mapping on the logic level
#include <mockturtle/algorithms/node_resynthesis/xag_npn.hpp>  // NPN databases for cut rewriting of XAGs and AIGs
#include <mockturtle/io/genlib_reader.hpp>                     // call-backs to read Genlib files into gate libraries
#include <mockturtle/utils/tech_library.hpp>                   // technology library utils
#include <mockturtle/io/write_dot.hpp>

#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

int main()  // NOLINT
{
    using gate_lyt = fiction::gate_level_layout<
        fiction::clocked_layout<fiction::tile_based_layout<fiction::cartesian_layout<fiction::offset::ucoord_t>>>>;

    experiments::experiment<std::string, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t,
                            uint64_t, uint64_t, uint64_t, uint32_t, uint32_t, uint64_t, uint64_t, double, std::string>
        technology_mapping_error_exp{"technology_mapping_error_exp",
                                     "benchmark",
                                     "inputs",
                                     "outputs",
                                     "initial nodes",
                                     "initial depth",
                                     "nodes after optimization",
                                     "depth after optimization",
                                     "nodes after mapping",
                                     "depth after mapping",
                                     "ortho layout width (in tiles)",
                                     "ortho layout height (in tiles)",
                                     "ortho layout area (in tiles)",
                                     "gates",
                                     "wires",
                                     "critical path",
                                     "throughput",
                                     "runtime ortho (in sec)",
                                     "equivalent"};

    // instantiate a complete XAG NPN database for node re-synthesis
    const mockturtle::xag_npn_resynthesis<mockturtle::aig_network,                    // the input network type
                                          mockturtle::aig_network,                    // the database network type
                                          mockturtle::xag_npn_db_kind::aig_complete>  // the kind of database to use
        resynthesis_function{};

    // parameters for cut rewriting
    mockturtle::cut_rewriting_params cut_params{};
    cut_params.cut_enumeration_ps.cut_size = 4;

    // instantiate a technology mapping library
    std::stringstream library_stream{};
    library_stream << fiction::GATE_ZERO << fiction::GATE_ONE << fiction::GATE_BUF << fiction::GATE_INV
                   << fiction::GATE_AND2 << fiction::GATE_OR2;

    std::vector<mockturtle::gate> gates{};

    // parameters for technology mapping
    const mockturtle::map_params map_params{};

    const auto read_genlib_result = lorina::read_genlib(library_stream, mockturtle::genlib_reader{gates});  // NOLINT
    assert(read_genlib_result == lorina::return_code::success);
    const mockturtle::tech_library<2> gate_lib{gates};

    // stats for ortho
    fiction::orthogonal_physical_design_stats orthogonal_stats{};

    static constexpr const uint64_t bench_select = fiction_experiments::mux21;

    for (const auto& benchmark : fiction_experiments::all_benchmarks(bench_select))
    {
        fmt::print("[i] processing {}\n", benchmark);
        std::ostringstream                        os{};
        fiction::network_reader<fiction::aig_ptr> reader{fiction_experiments::benchmark_path(benchmark), os};
        const auto                                nets    = reader.get_networks();
        const auto                                aig =*nets.front();

        // compute depth
        const mockturtle::depth_view depth_aig{aig};

        // rewrite network cuts using the given re-synthesis function
        const auto cut_aig = mockturtle::cut_rewriting(aig, resynthesis_function, cut_params);
        // compute depth
        const mockturtle::depth_view depth_cut_aig{cut_aig};

        // perform technology mapping
        const auto mapped_network = mockturtle::map(cut_aig, gate_lib, map_params);
        // compute depth
        const mockturtle::depth_view depth_mapped_network{mapped_network};
        //const auto mapped_network = mockturtle::map(cut_aig, gate_lib, map_params);
        mockturtle::write_dot(mapped_network, "mux21.dot");

        // perform layout generation with an SMT-based exact algorithm
        const auto gate_level_layout = fiction::orthogonal<gate_lyt>(mapped_network, {}, &orthogonal_stats);
        std::ofstream fout("mux21.txt");

        print_gate_level_layout(fout, gate_level_layout, false, false);

        // compute critical path and throughput
        fiction::critical_path_length_and_throughput_stats cp_tp_stats{};
        fiction::critical_path_length_and_throughput(gate_level_layout, &cp_tp_stats);

        // check equivalence
        fiction::equivalence_checking_stats eq_stats{};
        fiction::equivalence_checking(mapped_network, gate_level_layout, &eq_stats);

        const std::string eq_result = eq_stats.eq == fiction::eq_type::STRONG ? "STRONG" :
                                      eq_stats.eq == fiction::eq_type::WEAK   ? "WEAK" :
                                                                                "NO";

        // log results
        technology_mapping_error_exp(
            benchmark, aig.num_pis(), aig.num_pos(), aig.num_gates(), depth_aig.depth(), cut_aig.num_gates(),
            depth_cut_aig.depth(), mapped_network.num_gates(), depth_mapped_network.depth(), gate_level_layout.x() + 1,
            gate_level_layout.y() + 1, (gate_level_layout.x() + 1) * (gate_level_layout.y() + 1),
            gate_level_layout.num_gates(), gate_level_layout.num_wires(), cp_tp_stats.critical_path_length,
            cp_tp_stats.throughput, mockturtle::to_seconds(orthogonal_stats.time_total), eq_result);

        technology_mapping_error_exp.save();
        technology_mapping_error_exp.table();
    }

    return EXIT_SUCCESS;
}
