//
// Created by Simon Hofmann on 30.01.24.
//
#include "fiction_experiments.hpp"

#include <fiction/algorithms/physical_design/a_star_pr.hpp>                       // wiring reduction algorithm
#include <fiction/algorithms/properties/critical_path_length_and_throughput.hpp>  // critical path and throughput calculations
#include <fiction/algorithms/verification/equivalence_checking.hpp>               // SAT-based equivalence checking
#include <fiction/io/network_reader.hpp>                                          // read networks from files
#include <fiction/layouts/bounding_box.hpp>
#include <fiction/layouts/cartesian_layout.hpp>
#include <fiction/layouts/clocked_layout.hpp>
#include <fiction/layouts/gate_level_layout.hpp>
#include <fiction/layouts/tile_based_layout.hpp>
#include <fiction/networks/technology_network.hpp>

#include <fmt/format.h>  // output formatting

#include <cassert>
#include <chrono>
#include <cstdlib>
#include <string>

template <typename Ntk>
Ntk read_ntk(const std::string& name)
{
    fmt::print("[i] processing {}\n", name);

    std::ostringstream                        os{};
    fiction::network_reader<fiction::tec_ptr> reader{fiction_experiments::benchmark_path(name), os};
    const auto                                nets    = reader.get_networks();
    const auto                                network = *nets.front();

    return network;
}

int main()  // NOLINT
{
    using gate_lyt =
        fiction::gate_level_layout<fiction::clocked_layout<fiction::tile_based_layout<fiction::cartesian_layout<>>>>;

    experiments::experiment<std::string, uint32_t, uint32_t, uint32_t, uint64_t, uint64_t,
                            uint64_t, uint32_t, uint32_t, uint64_t, uint64_t, double, std::string>
        a_star_pr_exp{"a_star_pr_exp",
                             "benchmark",
                             "inputs",
                             "outputs",
                             "initial nodes",
                             "layout width (in tiles)",
                             "layout height (in tiles)",
                             "layout area (in tiles)",
                             "gates",
                             "wires",
                             "critical path",
                             "throughput",
                             "runtime a_star_pr (in sec)",
                             "equivalent"};

    // fiction::a_star_pr_stats a_star_pr_stats{};
    fiction::a_star_pr_stats a_star_pr_stats{};

    static constexpr const uint64_t bench_select = fiction_experiments::mux21;

    for (const auto& benchmark : fiction_experiments::all_benchmarks(bench_select))
    {
        const auto network = read_ntk<fiction::tec_nt>(benchmark);

        auto gate_level_layout = fiction::a_star_pr<gate_lyt>(network, &a_star_pr_stats);

        //  compute critical path and throughput
        fiction::critical_path_length_and_throughput_stats cp_tp_stats{};
        fiction::critical_path_length_and_throughput(gate_level_layout, &cp_tp_stats);

        // check equivalence
        fiction::equivalence_checking_stats eq_stats{};
        fiction::equivalence_checking<fiction::technology_network, gate_lyt>(network, gate_level_layout, &eq_stats);

        const std::string eq_result = eq_stats.eq == fiction::eq_type::STRONG ? "STRONG" :
                                      eq_stats.eq == fiction::eq_type::WEAK   ? "WEAK" :
                                                                                "NO";

        // calculate bounding box
        const auto bounding_box = fiction::bounding_box_2d(gate_level_layout);

        const auto width  = bounding_box.get_x_size() + 1;
        const auto height = bounding_box.get_y_size() + 1;
        const auto area   = width * height;

        // log results
        a_star_pr_exp(benchmark, network.num_pis(), network.num_pos(), network.num_gates(),
                             width, height,
                             area, gate_level_layout.num_gates(), gate_level_layout.num_wires(),
                             cp_tp_stats.critical_path_length, cp_tp_stats.throughput,
                             mockturtle::to_seconds(a_star_pr_stats.time_total),
                             eq_result);

        a_star_pr_exp.save();
        a_star_pr_exp.table();
    }

    return EXIT_SUCCESS;
}