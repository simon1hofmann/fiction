//
// Created by Jan Drewniok on 04.05.23.
//

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <fiction/algorithms/path_finding/distance.hpp>
#include <fiction/algorithms/simulation/sidb/random_sidb_layout_generator.hpp>
#include <fiction/layouts/cartesian_layout.hpp>
#include <fiction/layouts/cell_level_layout.hpp>
#include <fiction/layouts/clocked_layout.hpp>
#include <fiction/layouts/coordinates.hpp>
#include <fiction/technology/cell_technologies.hpp>
#include <fiction/technology/sidb_defects.hpp>
#include <fiction/technology/sidb_surface.hpp>

using namespace fiction;

TEST_CASE("Random cube::coord_t layout generation", "[generate-random-sidb-layout]")
{
    using cube_layout = cell_level_layout<sidb_technology, clocked_layout<cartesian_layout<cube::coord_t>>>;

    SECTION("empty parameters")
    {
        const generate_random_sidb_layout_params<cube_layout> params{};

        const auto lyt = generate_random_sidb_layout(cube_layout{}, params);

        CHECK(lyt.num_cells() == 0);
        CHECK(lyt.x() == 0);
        CHECK(lyt.y() == 0);
    }

    SECTION("given corner coordinates, wrong order")
    {
        const generate_random_sidb_layout_params<cube_layout> params{{{5, 7, 2}, {-10, -10, 0}}};

        const auto result_lyt = generate_random_sidb_layout(cube_layout{}, params);

        CHECK(result_lyt.num_cells() == 0);
        result_lyt.foreach_cell(
            [](const auto& cell)
            {
                CHECK(cell.x == 0);
                CHECK(cell.y == 0);
                CHECK(cell.z == 0);
            });
    }

    SECTION("given corner coordinates")
    {
        const generate_random_sidb_layout_params<cube_layout> params{{{-10, -10, 0}, {5, 7, 2}}};

        const auto result_lyt = generate_random_sidb_layout(cube_layout{}, params);

        CHECK(result_lyt.num_cells() == 0);
        result_lyt.foreach_cell(
            [](const auto& cell)
            {
                CHECK(cell.x == 0);
                CHECK(cell.y == 0);
                CHECK(cell.z == 0);
            });
    }

    SECTION("given two identical coordinates")
    {
        const generate_random_sidb_layout_params<cube_layout> params{{{-10, -10, 1}, {-10, -10, 1}}, 1};

        const auto result_lyt = generate_random_sidb_layout(cube_layout{}, params);

        CHECK(result_lyt.num_cells() == 1);
        result_lyt.foreach_cell(
            [](const auto& cell)
            {
                CHECK(cell.x == -10);
                CHECK(cell.y == -10);
                CHECK(cell.z == 1);
            });
    }

    SECTION("given corner coordinates and number of placed SiDBs")
    {
        const generate_random_sidb_layout_params<cube_layout> params{{{-10, -10, 0}, {5, 7, 1}}, 10};

        const auto result_lyt = generate_random_sidb_layout(cube_layout{}, params);

        CHECK(result_lyt.num_cells() == 10);
        result_lyt.foreach_cell(
            [&](const auto& cell)
            {
                CHECK(cell.x < 6);
                CHECK(cell.x > -11);
                CHECK(cell.y < 8);
                CHECK(cell.y > -11);
                CHECK(cell.z < 21);
                CHECK(cell.z > -1);
            });
    }

    SECTION("given corner coordinates and number of placed SiDBs, and forbid positive charges")
    {
        const generate_random_sidb_layout_params<cube_layout> params{
            {{0, 0, 0}, {90, 90, 0}},
            100,
            generate_random_sidb_layout_params<cube_layout>::positive_charges::ALLOWED};

        const auto result_lyt = generate_random_sidb_layout(cube_layout{}, params);

        CHECK(result_lyt.num_cells() == 100);
        result_lyt.foreach_cell(
            [](const auto& cell)
            {
                CHECK(cell.x < 91);
                CHECK(cell.y < 91);
            });
    }

    SECTION("given corner coordinates and number of placed SiDBs, and allow positive charges")
    {
        const generate_random_sidb_layout_params<cube_layout> params{
            {{0, 0, 0}, {90, 90, 0}},
            100,
            generate_random_sidb_layout_params<cube_layout>::positive_charges::FORBIDDEN};

        const auto result_lyt = generate_random_sidb_layout(cube_layout{}, params);

        CHECK(result_lyt.num_cells() == 100);
        result_lyt.foreach_cell(
            [](const auto& cell)
            {
                CHECK(cell.x < 91);
                CHECK(cell.y < 91);
            });
        // check if all cells are not closer than two cells (Euclidean distance).
        result_lyt.foreach_cell(
            [&result_lyt](const auto& cell_one)
            {
                result_lyt.foreach_cell(
                    [&cell_one, &result_lyt](const auto& cell_two)
                    {
                        if (cell_one != cell_two)
                        {
                            CHECK(euclidean_distance<cube_layout>(result_lyt, cell_one, cell_two) >= 2);
                        }
                    });
            });
    }

    SECTION("given previous layouts")
    {
        const generate_random_sidb_layout_params<cube_layout> params{
            {{-5, -2}, {9, 9}},
            10,
            generate_random_sidb_layout_params<cube_layout>::positive_charges::FORBIDDEN,
            2,
            static_cast<uint64_t>(10E6),
            3};
        const auto result_lyts = generate_multiple_random_sidb_layouts<cube_layout>(cube_layout{}, params);
        CHECK(result_lyts.size() == 3);

        for (const auto& lyt : result_lyts)
        {
            lyt.foreach_cell(
                [](const auto& cell)
                {
                    CHECK(cell.x < 10);
                    CHECK(cell.x > -6);
                    CHECK(cell.y < 10);
                    CHECK(cell.y > -3);
                });
        }
    }

    SECTION("Check uniqueness of two layouts")
    {
        const generate_random_sidb_layout_params<cube_layout> params{
            {{0, 0}, {9, 9}},
            10,
            generate_random_sidb_layout_params<cube_layout>::positive_charges::FORBIDDEN,
            2,
            static_cast<uint64_t>(10E6),
            2};
        const auto result_lyts = generate_multiple_random_sidb_layouts(cube_layout{}, params);
        REQUIRE(result_lyts.size() == 2);

        const auto& first_lyt  = result_lyts.front();
        const auto& second_lyt = result_lyts.back();

        uint64_t counter_different_cell = 0;
        first_lyt.foreach_cell(
            [&second_lyt, &counter_different_cell](const auto& cell_first)
            {
                second_lyt.foreach_cell(
                    [&cell_first, &counter_different_cell](const auto& cell_second)
                    {
                        if (cell_first != cell_second)
                        {
                            counter_different_cell += 1;
                        };
                    });
            });
        CHECK(counter_different_cell != 0);
    }
}

TEST_CASE("Random offset::ucoord_t layout generation", "[generate-random-sidb-layout]")
{
    SECTION("empty parameters")
    {
        const generate_random_sidb_layout_params<sidb_cell_clk_lyt> params{};

        const auto lyt = generate_random_sidb_layout(sidb_cell_clk_lyt{}, params);

        CHECK(lyt.num_cells() == 0);
        CHECK(lyt.x() == 0);
        CHECK(lyt.y() == 0);
    }

    SECTION("given corner coordinates")
    {
        const generate_random_sidb_layout_params<sidb_cell_clk_lyt> params{{{1, 1, 0}, {5, 7, 2}}};

        const auto result_lyt = generate_random_sidb_layout(sidb_cell_clk_lyt{}, params);

        CHECK(result_lyt.num_cells() == 0);
        result_lyt.foreach_cell(
            [&](const auto& cell)
            {
                CHECK(cell.x == 0);
                CHECK(cell.y == 0);
                CHECK(cell.z == 0);
            });
    }

    SECTION("given two identical coordinates")
    {
        const generate_random_sidb_layout_params<sidb_cell_clk_lyt> params{{{5, 5, 1}, {5, 5, 1}}, 1};

        const auto result_lyt = generate_random_sidb_layout(sidb_cell_clk_lyt{}, params);

        CHECK(result_lyt.num_cells() == 1);
        result_lyt.foreach_cell(
            [](const auto& cell)
            {
                CHECK(cell.x == 5);
                CHECK(cell.y == 5);
                CHECK(cell.z == 1);
            });
    }

    SECTION("given corner coordinates and number of placed SiDBs")
    {
        const generate_random_sidb_layout_params<sidb_cell_clk_lyt> params{{{1, 1, 0}, {50, 7, 1}}, 10};

        const auto result_lyt = generate_random_sidb_layout(sidb_cell_clk_lyt{}, params);

        CHECK(result_lyt.num_cells() == 10);
        result_lyt.foreach_cell(
            [](const auto& cell)
            {
                CHECK(cell.x < 51);
                CHECK(cell.x > 0);
                CHECK(cell.y < 8);
                CHECK(cell.y > 0);
                CHECK(cell.z < 21);
                CHECK(cell.z >= 0);
            });
    }

    SECTION("given corner coordinates and number of placed SiDBs, and forbid positive charges")
    {
        const generate_random_sidb_layout_params<sidb_cell_clk_lyt> params{
            {{0, 0, 0}, {90, 90, 0}},
            100,
            generate_random_sidb_layout_params<sidb_cell_clk_lyt>::positive_charges::ALLOWED};

        const auto result_lyt = generate_random_sidb_layout(sidb_cell_clk_lyt{}, params);

        CHECK(result_lyt.num_cells() == 100);
        result_lyt.foreach_cell(
            [](const auto& cell)
            {
                CHECK(cell.x < 91);
                CHECK(cell.y < 91);
            });
    }

    SECTION("given corner coordinates and number of placed SiDBs, and allow positive charges")
    {
        const generate_random_sidb_layout_params<sidb_cell_clk_lyt> params{
            {{0, 0, 0}, {90, 90, 0}},
            100,
            generate_random_sidb_layout_params<sidb_cell_clk_lyt>::positive_charges::FORBIDDEN};

        const auto result_lyt = generate_random_sidb_layout(sidb_cell_clk_lyt{}, params);

        CHECK(result_lyt.num_cells() == 100);
        result_lyt.foreach_cell(
            [](const auto& cell)
            {
                CHECK(cell.x < 91);
                CHECK(cell.y < 91);
            });
        // check if all cells are not closer than two cells (Euclidean distance).
        result_lyt.foreach_cell(
            [&result_lyt](const auto& cell_one)
            {
                result_lyt.foreach_cell(
                    [&cell_one, &result_lyt](const auto& cell_two)
                    {
                        if (cell_one != cell_two)
                        {
                            CHECK(euclidean_distance<sidb_cell_clk_lyt>(result_lyt, cell_one, cell_two) >= 2);
                        }
                    });
            });
    }

    SECTION("given previous layouts")
    {
        const generate_random_sidb_layout_params<sidb_cell_clk_lyt> params{
            {{0, 0}, {9, 9, 2}},
            10,
            generate_random_sidb_layout_params<sidb_cell_clk_lyt>::positive_charges::FORBIDDEN,
            2,
            static_cast<uint64_t>(10E6),
            3};
        const auto result_lyts = generate_multiple_random_sidb_layouts(sidb_cell_clk_lyt{}, params);
        CHECK(result_lyts.size() == 3);

        for (const auto& lyt : result_lyts)
        {
            lyt.foreach_cell(
                [](const auto& cell)
                {
                    CHECK(cell.x < 10);
                    CHECK(cell.y < 10);
                });
        }
    }

    SECTION("Check uniqueness of two layouts")
    {
        const generate_random_sidb_layout_params<sidb_cell_clk_lyt> params{
            {{0, 0}, {9, 9}},
            10,
            generate_random_sidb_layout_params<sidb_cell_clk_lyt>::positive_charges::FORBIDDEN,
            2,
            static_cast<uint64_t>(10E6),
            2};
        const auto result_lyts = generate_multiple_random_sidb_layouts(sidb_cell_clk_lyt{}, params);
        REQUIRE(result_lyts.size() == 2);

        const auto& first_lyt  = result_lyts.front();
        const auto& second_lyt = result_lyts.back();

        uint64_t counter_different_cell = 0;
        first_lyt.foreach_cell(
            [&second_lyt, &counter_different_cell](const auto& cell_first)
            {
                second_lyt.foreach_cell(
                    [&cell_first, &counter_different_cell](const auto& cell_second)
                    {
                        if (cell_first != cell_second)
                        {
                            counter_different_cell += 1;
                        };
                    });
            });
        CHECK(counter_different_cell != 0);
    }

    SECTION("Check correct use of skeleton layout when generating only one random layout")
    {
        const generate_random_sidb_layout_params<sidb_cell_clk_lyt> params{{{0, 0}, {9, 9}}, 10};
        sidb_cell_clk_lyt                                           skeleton_layout{};
        skeleton_layout.assign_cell_type({0, 0}, sidb_cell_clk_lyt::technology::NORMAL);
        skeleton_layout.assign_cell_type({9, 1}, sidb_cell_clk_lyt::technology::NORMAL);
        skeleton_layout.assign_cell_type({5, 0}, sidb_cell_clk_lyt::technology::NORMAL);
        const auto result_lyt = generate_random_sidb_layout(skeleton_layout, params);

        CHECK(result_lyt.num_cells() == 13);
    }

    SECTION("Check correct use of skeleton layout when generating multiple random layouts")
    {
        const generate_random_sidb_layout_params<sidb_cell_clk_lyt> params{
            {{0, 0}, {9, 9}},
            10,
            generate_random_sidb_layout_params<sidb_cell_clk_lyt>::positive_charges::FORBIDDEN,
            2,
            static_cast<uint64_t>(10E6),
            2};
        sidb_cell_clk_lyt skeleton_layout{};
        skeleton_layout.assign_cell_type({0, 0}, sidb_cell_clk_lyt::technology::NORMAL);
        skeleton_layout.assign_cell_type({3, 0}, sidb_cell_clk_lyt::technology::NORMAL);
        skeleton_layout.assign_cell_type({9, 1}, sidb_cell_clk_lyt::technology::NORMAL);
        const auto result_lyts = generate_multiple_random_sidb_layouts<sidb_cell_clk_lyt>(skeleton_layout, params);

        REQUIRE(result_lyts.size() == 2);
        CHECK(result_lyts.front().num_cells() == 13);
        CHECK(result_lyts.back().num_cells() == 13);
    }
}

TEST_CASE("Random siqad::coord_t layout generation", "[generate-random-sidb-layout]")
{
    SECTION("empty parameters")
    {
        const generate_random_sidb_layout_params<sidb_cell_clk_lyt_siqad> params{};

        const auto lyt = generate_random_sidb_layout(sidb_cell_clk_lyt_siqad{}, params);

        CHECK(lyt.num_cells() == 0);
        CHECK(lyt.x() == 0);
        CHECK(lyt.y() == 0);
    }

    SECTION("given two identical coordinates")
    {
        const generate_random_sidb_layout_params<sidb_cell_clk_lyt_siqad> params{{{5, 5, 1}, {5, 5, 1}}, 1};

        const auto result_lyt = generate_random_sidb_layout<sidb_cell_clk_lyt_siqad>(sidb_cell_clk_lyt_siqad{}, params);

        CHECK(result_lyt.num_cells() == 1);
        result_lyt.foreach_cell(
            [](const auto& cell)
            {
                CHECK(cell.x == 5);
                CHECK(cell.y == 5);
                CHECK(cell.z == 1);
            });
    }

    SECTION("given corner coordinates")
    {
        const generate_random_sidb_layout_params<sidb_cell_clk_lyt_siqad> params{{{1, 1, 0}, {5, 7, 1}}};

        const auto result_lyt = generate_random_sidb_layout(sidb_cell_clk_lyt_siqad{}, params);

        CHECK(result_lyt.num_cells() == 0);
        result_lyt.foreach_cell(
            [&](const auto& cell)
            {
                CHECK(cell.x == 0);
                CHECK(cell.y == 0);
                CHECK(cell.z == 0);
            });
    }

    SECTION("given corner coordinates and number of placed SiDBs")
    {
        const generate_random_sidb_layout_params<sidb_cell_clk_lyt_siqad> params{{{1, 1, 0}, {50, 7, 1}}, 10};

        const auto result_lyt = generate_random_sidb_layout(sidb_cell_clk_lyt_siqad{}, params);

        CHECK(result_lyt.num_cells() == 10);
        result_lyt.foreach_cell(
            [](const auto& cell)
            {
                CHECK(cell.x < 51);
                CHECK(cell.x > 0);
                CHECK(cell.y < 8);
                CHECK(cell.y > 0);
                CHECK(cell.z <= 1);
            });
    }

    SECTION("given corner coordinates and number of placed SiDBs, and allow positive charges")
    {
        const generate_random_sidb_layout_params<sidb_cell_clk_lyt_siqad> params{
            {{0, 0, 0}, {90, 90, 0}},
            100,
            generate_random_sidb_layout_params<sidb_cell_clk_lyt_siqad>::positive_charges::ALLOWED};

        const auto result_lyt = generate_random_sidb_layout(sidb_cell_clk_lyt_siqad{}, params);

        CHECK(result_lyt.num_cells() == 100);
        result_lyt.foreach_cell(
            [](const auto& cell)
            {
                CHECK(cell.x < 91);
                CHECK(cell.y < 91);
            });
    }

    SECTION("given corner coordinates and number of placed SiDBs, and forbid positive charges")
    {
        const generate_random_sidb_layout_params<sidb_cell_clk_lyt_siqad> params{
            {{0, 0, 0}, {90, 90, 0}},
            10,
            generate_random_sidb_layout_params<sidb_cell_clk_lyt_siqad>::positive_charges::FORBIDDEN};

        const auto result_lyt = generate_random_sidb_layout(sidb_cell_clk_lyt_siqad{}, params);

        CHECK(result_lyt.num_cells() == 10);
        result_lyt.foreach_cell(
            [](const auto& cell)
            {
                CHECK(cell.x < 91);
                CHECK(cell.y < 91);
                CHECK(cell.z <= 1);
            });
        // check if all cells are not closer than two cells (Euclidean distance).
        result_lyt.foreach_cell(
            [&result_lyt](const auto& cell_one)
            {
                result_lyt.foreach_cell(
                    [&cell_one, &result_lyt](const auto& cell_two)
                    {
                        if (cell_one != cell_two)
                        {
                            CHECK(euclidean_distance<sidb_cell_clk_lyt_siqad>(result_lyt, cell_one, cell_two) >= 2);
                        }
                    });
            });
    }

    SECTION("given previous layouts")
    {
        const generate_random_sidb_layout_params<sidb_cell_clk_lyt_siqad> params{
            {{0, 0, 1}, {9, 9, 1}},
            10,
            generate_random_sidb_layout_params<sidb_cell_clk_lyt_siqad>::positive_charges::FORBIDDEN,
            2,
            static_cast<uint64_t>(10E6),
            3};
        const auto result_lyts = generate_multiple_random_sidb_layouts(sidb_cell_clk_lyt_siqad{}, params);
        CHECK(result_lyts.size() == 3);

        for (const auto& lyt : result_lyts)
        {
            lyt.foreach_cell(
                [](const auto& cell)
                {
                    CHECK(cell.x < 10);
                    CHECK(cell.x >= 0);
                    CHECK(cell.y < 10);
                    CHECK(cell.y >= 0);
                    CHECK(cell.z <= 1);
                });
        }
    }
}

TEST_CASE("Random siqad::coord_t layout generation with defects", "[generate-random-sidb-layout]")
{

    SECTION("given two identical coordinates")
    {
        const generate_random_sidb_layout_params<sidb_defect_cell_clk_lyt_siqad> params{{{5, 5, 1}, {5, 5, 1}}, 1};

        const auto result_lyt = generate_random_sidb_layout(sidb_defect_cell_clk_lyt_siqad{}, params);

        CHECK(result_lyt.num_cells() == 1);
        result_lyt.foreach_cell(
            [](const auto& cell)
            {
                CHECK(cell.x == 5);
                CHECK(cell.y == 5);
                CHECK(cell.z == 1);
            });
    }

    SECTION("region including only one cell and there is a defect")
    {
        // it is not possible to generate a random layout since the position where a SiDB could be placed is occupied by
        // a defect.
        const generate_random_sidb_layout_params<sidb_defect_cell_clk_lyt_siqad> params{
            {{2, 1, 1}, {2, 1, 1}},
            1,
            generate_random_sidb_layout_params<sidb_defect_cell_clk_lyt_siqad>::positive_charges::FORBIDDEN,
            2,
            5u};

        auto defect_layout = sidb_defect_cell_clk_lyt_siqad{};
        defect_layout.assign_sidb_defect({2, 1, 1}, sidb_defect{sidb_defect_type::DB, -1, 5.6, 5});

        const auto result_lyt = generate_random_sidb_layout(defect_layout, params);

        CHECK(result_lyt.num_cells() == 0);
        CHECK(result_lyt.num_defects() == 1);
    }

    SECTION("region including only one cell and there is no defect")
    {
        // it is not possible to generate a random layout since the position where a SiDB could be placed is occupied by
        // a defect.
        const generate_random_sidb_layout_params<sidb_defect_cell_clk_lyt_siqad> params{
            {{2, 1, 1}, {2, 1, 1}},
            1,
            generate_random_sidb_layout_params<sidb_defect_cell_clk_lyt_siqad>::positive_charges::FORBIDDEN,
            2,
            5u};

        auto defect_layout = sidb_defect_cell_clk_lyt_siqad{};
        defect_layout.assign_sidb_defect({3, 1, 1}, sidb_defect{sidb_defect_type::DB, -1, 5.6, 5});
        defect_layout.assign_sidb_defect({4, 1, 1}, sidb_defect{sidb_defect_type::SINGLE_DIHYDRIDE, 1, 7.6, 7});

        const auto result_lyt = generate_random_sidb_layout(defect_layout, params);

        CHECK(result_lyt.num_cells() == 1);
        CHECK(result_lyt.num_defects() == 2);

        CHECK(result_lyt.get_cell_type({2, 1, 1}) == sidb_defect_cell_clk_lyt_siqad::technology::cell_type::NORMAL);
        CHECK(result_lyt.get_sidb_defect({3, 1, 1}) == sidb_defect{sidb_defect_type::DB, -1, 5.6, 5});
        CHECK(result_lyt.get_sidb_defect({4, 1, 1}) == sidb_defect{sidb_defect_type::SINGLE_DIHYDRIDE, 1, 7.6, 7});
    }

    SECTION("given corner coordinates and number of placed SiDBs, and allow positive charges")
    {
        const generate_random_sidb_layout_params<sidb_defect_cell_clk_lyt_siqad> params{
            {{0, 0, 0}, {10, 2, 0}},
            10,
            generate_random_sidb_layout_params<sidb_defect_cell_clk_lyt_siqad>::positive_charges::ALLOWED,
            2};

        auto defect_layout = sidb_defect_cell_clk_lyt_siqad{};
        defect_layout.assign_sidb_defect({2, 2, 0}, sidb_defect{sidb_defect_type::DB, -1, 5.6, 5});
        defect_layout.assign_sidb_defect({4, 1, 0}, sidb_defect{sidb_defect_type::SINGLE_DIHYDRIDE, 1, 7.6, 7});
        defect_layout.assign_sidb_defect({5, 1, 0}, sidb_defect{sidb_defect_type::SINGLE_DIHYDRIDE, 1, 7.6, 9});
        defect_layout.assign_sidb_defect({7, 1, 0}, sidb_defect{sidb_defect_type::SINGLE_DIHYDRIDE, 1, 2.6, 7});
        defect_layout.assign_sidb_defect({2, 1, 0}, sidb_defect{sidb_defect_type::SINGLE_DIHYDRIDE, 1, 7.6, 4});

        const auto result_lyt = generate_random_sidb_layout(defect_layout, params);

        CHECK(result_lyt.num_cells() == 10);
        CHECK(result_lyt.num_defects() == 5);

        // check if all cells are not closer than two cells (Euclidean distance).
        result_lyt.foreach_cell(
            [](const auto& cell)
            {
                CHECK(cell != siqad::coord_t{2, 2, 0});
                CHECK(cell != siqad::coord_t{4, 1, 0});
                CHECK(cell != siqad::coord_t{5, 1, 0});
                CHECK(cell != siqad::coord_t{7, 1, 0});
                CHECK(cell != siqad::coord_t{2, 1, 0});
            });
    }
}

TEST_CASE("Random cube::coord_t layout generation with defects", "[generate-random-sidb-layout]")
{
    using lyt = sidb_surface<cell_level_layout<sidb_technology, clocked_layout<cartesian_layout<cube::coord_t>>>>;

    const generate_random_sidb_layout_params<lyt> params{
        {siqad::to_fiction_coord<cube::coord_t>(siqad::coord_t{0, 0, 0}),
         siqad::to_fiction_coord<cube::coord_t>(siqad::coord_t{10, 2, 0})},
        10,
        generate_random_sidb_layout_params<lyt>::positive_charges::ALLOWED,
        2};

    lyt layout{};

    layout.assign_sidb_defect(siqad::to_fiction_coord<cube::coord_t>(siqad::coord_t{2, 2, 0}),
                              sidb_defect{sidb_defect_type::DB, -1, 5.6, 5});
    layout.assign_sidb_defect(siqad::to_fiction_coord<cube::coord_t>(siqad::coord_t{4, 1, 0}),
                              sidb_defect{sidb_defect_type::SINGLE_DIHYDRIDE, 1, 7.6, 7});
    layout.assign_sidb_defect(siqad::to_fiction_coord<cube::coord_t>(siqad::coord_t{5, 1, 0}),
                              sidb_defect{sidb_defect_type::SINGLE_DIHYDRIDE, 1, 7.6, 9});
    layout.assign_sidb_defect(siqad::to_fiction_coord<cube::coord_t>(siqad::coord_t{7, 1, 0}),
                              sidb_defect{sidb_defect_type::SINGLE_DIHYDRIDE, 1, 2.6, 7});
    layout.assign_sidb_defect(siqad::to_fiction_coord<cube::coord_t>(siqad::coord_t{2, 1, 0}),
                              sidb_defect{sidb_defect_type::SINGLE_DIHYDRIDE, 1, 7.6, 4});

    const auto result_lyt = generate_random_sidb_layout(layout, params);

    CHECK(result_lyt.num_cells() == 10);
    CHECK(result_lyt.num_defects() == 5);

    // check if all cells are not closer than two cells (Euclidean distance).
    result_lyt.foreach_cell(
        [](const auto& cell)
        {
            CHECK(cell != siqad::to_fiction_coord<cube::coord_t>(siqad::coord_t{2, 2, 0}));
            CHECK(cell != siqad::to_fiction_coord<cube::coord_t>(siqad::coord_t{4, 1, 0}));
            CHECK(cell != siqad::to_fiction_coord<cube::coord_t>(siqad::coord_t{5, 1, 0}));
            CHECK(cell != siqad::to_fiction_coord<cube::coord_t>(siqad::coord_t{7, 1, 0}));
            CHECK(cell != siqad::to_fiction_coord<cube::coord_t>(siqad::coord_t{2, 1, 0}));
        });
}
