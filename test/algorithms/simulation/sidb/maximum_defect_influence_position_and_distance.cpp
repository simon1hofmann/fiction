//
// Created by Jan Drewniok on 26.06.23.
//

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <fiction/algorithms/path_finding/distance.hpp>
#include <fiction/algorithms/simulation/sidb/maximum_defect_influence_position_and_distance.hpp>
#include <fiction/technology/physical_constants.hpp>
#include <fiction/utils/math_utils.hpp>

using namespace fiction;

TEST_CASE("Test influence distance function", "[maximum-defect-influence-position-and-distance]")
{
    SECTION("empty layout")
    {
        const sidb_defect                              defect{sidb_defect_type::UNKNOWN, -1, 10.6, 5.9};
        const maximum_defect_influence_distance_params sim_params{defect, sidb_simulation_parameters{}};
        const sidb_cell_clk_lyt_siqad                  lyt{};

        const auto [defect_pos, distance] = maximum_defect_influence_position_and_distance(lyt, sim_params);
        CHECK(distance == 0);
        CHECK(defect_pos == coordinate<sidb_cell_clk_lyt_siqad>());
    }

    SECTION("layout with one SiDB")
    {
        const sidb_defect defect{sidb_defect_type::UNKNOWN, -1, sidb_simulation_parameters{}.epsilon_r,
                                 sidb_simulation_parameters{}.lambda_tf};
        const maximum_defect_influence_distance_params sim_params{defect, sidb_simulation_parameters{}, {2, 2}};

        sidb_cell_clk_lyt_siqad lyt{};
        lyt.assign_cell_type({0, 0, 0}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);

        const auto [defect_pos, distance] = maximum_defect_influence_position_and_distance(lyt, sim_params);
        CHECK_THAT(round_to_n_decimal_places(distance, 6),
                   Catch::Matchers::WithinAbs(0.665060, physical_constants::POP_STABILITY_ERR));
        CHECK((((defect_pos.x == -1) && (defect_pos.y == -1) && (defect_pos.z == 1)) ||
               ((defect_pos.x == 1) && (defect_pos.y == -1) && (defect_pos.z == 1))));
    }

    SECTION("layout with one SiDB, negative defect, smaller lambda_tf")
    {
        const sidb_defect defect{sidb_defect_type::UNKNOWN, -1, sidb_simulation_parameters{}.epsilon_r, 1};
        const maximum_defect_influence_distance_params sim_params{defect, sidb_simulation_parameters{}};
        sidb_cell_clk_lyt_siqad                        lyt{};
        lyt.assign_cell_type({0, 0, 0}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        const auto [defect_pos, distance] = maximum_defect_influence_position_and_distance(lyt, sim_params);
        CHECK_THAT(round_to_n_decimal_places(distance, 4) -
                       round_to_n_decimal_places(sidb_nanometer_distance(lyt, {0, 0, 0}, {-1, 0, 1}), 4),
                   Catch::Matchers::WithinAbs(0.0, physical_constants::POP_STABILITY_ERR));
    }

    SECTION("layout with one SiDB, negative defect, large lambda_tf")
    {
        const sidb_defect defect{sidb_defect_type::UNKNOWN, -1, sidb_simulation_parameters{}.epsilon_r, 20};
        const maximum_defect_influence_distance_params sim_params{defect, sidb_simulation_parameters{}, {2, 2}};
        sidb_cell_clk_lyt_siqad                        lyt{};
        lyt.assign_cell_type({0, 0, 0}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        const auto [defect_pos, distance] = maximum_defect_influence_position_and_distance(lyt, sim_params);
        CHECK_THAT(round_to_n_decimal_places(distance, 4) -
                       round_to_n_decimal_places(sidb_nanometer_distance(lyt, {0, 0, 0}, {0, 1, 0}), 4),
                   Catch::Matchers::WithinAbs(0.0, physical_constants::POP_STABILITY_ERR));
    }

    SECTION("layout with one pertuber and one DB pair, negative defect")
    {
        const sidb_defect defect{sidb_defect_type::UNKNOWN, -1, sidb_simulation_parameters{}.epsilon_r,
                                 sidb_simulation_parameters{}.lambda_tf};
        const maximum_defect_influence_distance_params sim_params{defect, sidb_simulation_parameters{}};
        sidb_cell_clk_lyt_siqad                        lyt{};
        lyt.assign_cell_type({0, 0, 0}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        lyt.assign_cell_type({4, 0, 0}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        lyt.assign_cell_type({6, 0, 0}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);

        const auto [defect_pos, distance] = maximum_defect_influence_position_and_distance(lyt, sim_params);
        CHECK_THAT(round_to_n_decimal_places(distance, 4) -
                       round_to_n_decimal_places(sidb_nanometer_distance(lyt, {6, 0, 0}, {10, 0, 0}), 4),
                   Catch::Matchers::WithinAbs(0.0, physical_constants::POP_STABILITY_ERR));
    }

    SECTION("QuickExact simulation of a Y-shape SiDB OR gate with input 01")
    {
        const sidb_defect defect{sidb_defect_type::UNKNOWN, -1, sidb_simulation_parameters{}.epsilon_r,
                                 sidb_simulation_parameters{}.lambda_tf};
        const maximum_defect_influence_distance_params sim_params{defect, sidb_simulation_parameters{}};
        sidb_cell_clk_lyt_siqad                        lyt{};

        lyt.assign_cell_type({10, 0, 0}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        lyt.assign_cell_type({0, 1, 0}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        lyt.assign_cell_type({8, 1, 0}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);

        lyt.assign_cell_type({2, 2, 0}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        lyt.assign_cell_type({6, 2, 0}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        lyt.assign_cell_type({4, 4, 0}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        lyt.assign_cell_type({4, 5, 1}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        lyt.assign_cell_type({4, 7, 1}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);

        const auto [defect_pos, distance] = maximum_defect_influence_position_and_distance(lyt, sim_params);
        CHECK(defect_pos.x == 12);
        CHECK(defect_pos.y == 4);
        CHECK(defect_pos.z == 1);

        CHECK_THAT(distance, Catch::Matchers::WithinAbs(2.8999201713, physical_constants::POP_STABILITY_ERR));

        // number of threads given by the hardware
        const sidb_defect high_screening{sidb_defect_type::UNKNOWN, -1, sidb_simulation_parameters{}.epsilon_r, 1};
        const maximum_defect_influence_distance_params sim_params_high_screening{high_screening,
                                                                                 sidb_simulation_parameters{}};

        const auto [defect_pos_high_screeing, distance_high_screeing] =
            maximum_defect_influence_position_and_distance(lyt, sim_params_high_screening);

        CHECK(distance_high_screeing < distance);
    }

    SECTION("QuickExact simulation of a Y-shape SiDB OR gate with input 01, using cube coordinate")
    {
        const sidb_defect defect{sidb_defect_type::UNKNOWN, -1, sidb_simulation_parameters{}.epsilon_r,
                                 sidb_simulation_parameters{}.lambda_tf};
        const maximum_defect_influence_distance_params sim_params{defect, sidb_simulation_parameters{}};
        cell_level_layout<sidb_technology, clocked_layout<cartesian_layout<cube::coord_t>>> lyt{{30, 30}};

        lyt.assign_cell_type(siqad::to_fiction_coord<cube::coord_t>(siqad::coord_t{10, 0, 0}),
                             sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        lyt.assign_cell_type(siqad::to_fiction_coord<cube::coord_t>(siqad::coord_t{0, 1, 0}),
                             sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        lyt.assign_cell_type(siqad::to_fiction_coord<cube::coord_t>(siqad::coord_t{8, 1, 0}),
                             sidb_cell_clk_lyt_siqad::cell_type::NORMAL);

        lyt.assign_cell_type(siqad::to_fiction_coord<cube::coord_t>(siqad::coord_t{2, 2, 0}),
                             sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        lyt.assign_cell_type(siqad::to_fiction_coord<cube::coord_t>(siqad::coord_t{6, 2, 0}),
                             sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        lyt.assign_cell_type(siqad::to_fiction_coord<cube::coord_t>(siqad::coord_t{4, 4, 0}),
                             sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        lyt.assign_cell_type(siqad::to_fiction_coord<cube::coord_t>(siqad::coord_t{4, 5, 1}),
                             sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        lyt.assign_cell_type(siqad::to_fiction_coord<cube::coord_t>(siqad::coord_t{4, 7, 1}),
                             sidb_cell_clk_lyt_siqad::cell_type::NORMAL);

        const auto [defect_pos, distance] = maximum_defect_influence_position_and_distance(lyt, sim_params);
        CHECK(defect_pos.x == 12);
        CHECK(defect_pos.y == 9);
        CHECK(defect_pos.z == 0);

        CHECK_THAT(distance, Catch::Matchers::WithinAbs(2.8999201713, physical_constants::POP_STABILITY_ERR));
    }
}
