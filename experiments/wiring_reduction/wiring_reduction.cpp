#include "fiction_experiments.hpp"

#include <fiction/algorithms/physical_design/orthogonal.hpp>  // scalable heuristic for physical design of FCN layouts
#include <fiction/algorithms/physical_design/wiring_reduction.hpp>  // wiring reduction algorithm
#include <fiction/algorithms/properties/critical_path_length_and_throughput.hpp>  // critical path and throughput calculations
#include <fiction/algorithms/verification/equivalence_checking.hpp>               // SAT-based equivalence checking

#include <fmt/format.h>                      // output formatting
#include <lorina/lorina.hpp>                 // Verilog/BLIF/AIGER/... file parsing
#include <mockturtle/io/verilog_reader.hpp>  // call-backs to read Verilog files into networks

#include <cassert>
#include <chrono>
#include <cstdlib>
#include <string>

int main()  // NOLINT
{
    using gate_lyt =
        fiction::gate_level_layout<fiction::clocked_layout<fiction::tile_based_layout<fiction::cartesian_layout<>>>>;

    experiments::experiment<std::string, uint32_t, uint32_t, uint32_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t,
                            uint64_t, uint32_t, uint32_t, uint64_t, uint64_t, double, double, float, std::string>
        wiring_reduction_exp{"wiring_reduction",
                         "benchmark",
                         "inputs",
                         "outputs",
                         "initial nodes",
                         "ortho layout width (in tiles)",
                         "ortho layout height (in tiles)",
                         "ortho layout area (in tiles)",
                         "optimized layout width (in tiles)",
                         "optimized layout height (in tiles)",
                         "optimized layout area (in tiles)",
                         "gates",
                         "wires",
                         "critical path",
                         "throughput",
                         "runtime ortho (in sec)",
                         "runtime optimization (in sec)",
                         "improvement (%)",
                         "equivalent"};

    // stats for SMT-based physical design
    fiction::orthogonal_physical_design_stats orthogonal_stats{};
    fiction::wiring_reduction_stats   wiring_reduction_stats{};

    static constexpr const uint64_t bench_select = (fiction_experiments::trindade16 | fiction_experiments::fontes18) & ~fiction_experiments::clpl;

    for (const auto& benchmark : fiction_experiments::all_benchmarks(bench_select))
    {
        fmt::print("[i] processing {}\n", benchmark);

        fiction::technology_network network{};

        const auto read_verilog_result =
            lorina::read_verilog(fiction_experiments::benchmark_path(benchmark), mockturtle::verilog_reader(network));
        assert(read_verilog_result == lorina::return_code::success);

        // perform layout generation with an OGD-based heuristic algorithm
        auto gate_level_layout = fiction::orthogonal<gate_lyt>(network, {}, &orthogonal_stats);

        //  compute critical path and throughput
        fiction::critical_path_length_and_throughput_stats cp_tp_stats{};
        fiction::critical_path_length_and_throughput(gate_level_layout, &cp_tp_stats);

        // calculate bounding box
        const auto bounding_box_before_wiring_reduction = fiction::bounding_box_2d(gate_level_layout);

        const auto width_before_wiring_reduction  = bounding_box_before_wiring_reduction.get_x_size() + 1;
        const auto height_before_wiring_reduction = bounding_box_before_wiring_reduction.get_y_size() + 1;
        const auto area_before_wiring_reduction   = width_before_wiring_reduction * height_before_wiring_reduction;

        // perform post-layout optimization
        fiction::wiring_reduction<gate_lyt>(gate_level_layout, &wiring_reduction_stats);

        // check equivalence
        fiction::equivalence_checking_stats eq_stats{};
        fiction::equivalence_checking<fiction::technology_network, gate_lyt>(network, gate_level_layout, &eq_stats);

        const std::string eq_result = eq_stats.eq == fiction::eq_type::STRONG ? "STRONG" :
                                      eq_stats.eq == fiction::eq_type::WEAK   ? "WEAK" :
                                                                                "NO";

        // calculate bounding box
        const auto bounding_box_after_wiring_reduction = fiction::bounding_box_2d(gate_level_layout);

        const auto width_after_wiring_reduction  = bounding_box_after_wiring_reduction.get_x_size() + 1;
        const auto height_after_wiring_reduction = bounding_box_after_wiring_reduction.get_y_size() + 1;
        const auto area_after_wiring_reduction   = width_after_wiring_reduction * height_after_wiring_reduction;

        const float improv = 100 * static_cast<float>((area_before_wiring_reduction - area_after_wiring_reduction)) /
                             static_cast<float>(area_before_wiring_reduction);
        // log results
        wiring_reduction_exp(benchmark, network.num_pis(), network.num_pos(), network.num_gates(),
                         width_before_wiring_reduction, height_before_wiring_reduction, area_before_wiring_reduction,
                         width_after_wiring_reduction, height_after_wiring_reduction, area_after_wiring_reduction,
                         gate_level_layout.num_gates(), gate_level_layout.num_wires(), cp_tp_stats.critical_path_length,
                         cp_tp_stats.throughput, mockturtle::to_seconds(orthogonal_stats.time_total),
                         mockturtle::to_seconds(wiring_reduction_stats.time_total), improv, eq_result);

        wiring_reduction_exp.save();
        wiring_reduction_exp.table();
    }

    return EXIT_SUCCESS;
}