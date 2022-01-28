//
// Created by marcel on 23.09.21.
//

#ifndef FICTION_WRITE_FQCA_LAYOUT_HPP
#define FICTION_WRITE_FQCA_LAYOUT_HPP

#include "fiction/technology/cell_technologies.hpp"
#include "fiction/traits.hpp"
#include "utils/version_info.hpp"

#include <fmt/format.h>

#include <fstream>
#include <ostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

namespace fiction
{

struct write_fqca_layout_params
{
    bool create_inter_layer_via_cells = false;
};

class out_of_cell_names_exception : public std::exception
{
  public:
    out_of_cell_names_exception() noexcept : std::exception() {}

    [[nodiscard]] const char* what() const noexcept override
    {
        return "no more single-character cell designators available";
    }
};

namespace detail
{

namespace qca_stack
{

static constexpr const char* HEADER = "[ {}generated by {} ({}) ]\n\n";

static constexpr const char* COLUMN = "=";

static constexpr const char* CELL_SEPARATOR = " ";
static constexpr const char* EMPTY_CELL     = " ";
static constexpr const char* CONST_0_CELL   = "-";
static constexpr const char* CONST_1_CELL   = "+";

static constexpr const char* SECTION_SEPARATOR = "\n\n$\n\n";

static constexpr const char* ITEM           = "- ";
static constexpr const char* LABEL          = "label = \"{}\"";
static constexpr const char* NUMBER         = "number = {}";
static constexpr const char* CLOCK          = "clock = {}";
static constexpr const char* OFFSET         = "offset = {}";
static constexpr const char* PRIMARY_INPUT  = "input";
static constexpr const char* PRIMARY_OUTPUT = "output";
static constexpr const char* PROPAGATE      = "propagate";

}  // namespace qca_stack

template <typename Lyt>
class write_fqca_layout_impl
{
  public:
    write_fqca_layout_impl(const Lyt& src, std::ostream& s, const write_fqca_layout_params& p) : lyt{src}, os{s}, ps{p}
    {}

    void run()
    {
        write_header();

        write_layout_definition();

        os << qca_stack::SECTION_SEPARATOR;

        write_cell_definition();

        os << std::flush;
    }

  private:
    Lyt lyt;

    std::ostream& os;

    const write_fqca_layout_params ps;

    // used to generate named cells
    char const* alphabet_iterator = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";

    struct labeled_cell
    {
        labeled_cell(const cell<Lyt>& c_, const char d_) noexcept : c{c_}, designator{d_} {}

        cell<Lyt> c;

        char designator;
    };

    std::vector<labeled_cell> labeled_cells{};

    /**
     * Might throw an 'out_of_cell_names_exception'.
     *
     * @return The next cell designator in the alphabet.
     */
    char next_cell_designator()
    {
        if (const auto designator = *alphabet_iterator; designator)
        {
            // increment iterator only after its value has been fetched to include 'a' on first call
            ++alphabet_iterator;

            return designator;
        }
        else
        {
            throw out_of_cell_names_exception();
        }
    }

    void write_header()
    {
        const auto layout_name = lyt.get_layout_name();
        os << fmt::format(qca_stack::HEADER, layout_name.empty() ? "" : fmt::format("{} - ", layout_name),
                          FICTION_VERSION, FICTION_REPO);
    }

    void write_layer_separator()
    {
        for (decltype(lyt.x()) i = 0; i <= lyt.x(); i++) { os << qca_stack::COLUMN << qca_stack::CELL_SEPARATOR; }

        os << '\n';
    }

    void write_cell(const cell<Lyt>& c)
    {
        // empty cell
        if (lyt.is_empty_cell(c))
        {
            os << qca_stack::EMPTY_CELL;
        }
        // named or primary cell
        else if (const auto cell_name = lyt.get_cell_name(c); !cell_name.empty() || lyt.is_pi(c) || lyt.is_po(c))
        {
            // generate next cell designator
            const auto cell_designator = next_cell_designator();

            // save cell for later writing in the label section
            labeled_cells.emplace_back(c, cell_designator);

            // write designator
            os << cell_designator;
        }
        // const-0 cell
        else if (const auto cell_type = lyt.get_cell_type(c); qca_technology::is_const_0_cell(cell_type))
        {
            os << qca_stack::CONST_0_CELL;
        }
        // const-1 cell
        else if (qca_technology::is_const_1_cell(cell_type))
        {
            os << qca_stack::CONST_1_CELL;
        }
        // regular cell
        else
        {
            // write cell clock number
            os << std::to_string(lyt.get_clock_number(c));
        }

        os << qca_stack::CELL_SEPARATOR;
    }

    void write_via_cell(const cell<Lyt>& c, std::stringstream& via_stream) const
    {
        // if cell is marked as a vertical cell
        if (const auto cell_mode = lyt.get_cell_mode(c); qca_technology::is_vertical_cell_mode(cell_mode))
        {
            via_stream << std::to_string(lyt.get_clock_number(c));
        }
        // must be an empty cell
        else
        {
            via_stream << qca_stack::EMPTY_CELL;
        }

        via_stream << qca_stack::CELL_SEPARATOR;
    }

    void write_layout_definition()
    {
        // for each layer
        for (decltype(lyt.z()) z = 0; z <= lyt.z(); ++z)
        {
            // list of inter-layer cells
            std::stringstream via_layer_buffer{};

            write_layer_separator();

            // for each cell
            for (decltype(lyt.y()) y = 0; y <= lyt.y(); ++y)
            {
                for (decltype(lyt.x()) x = 0; x <= lyt.x(); ++x)
                {
                    const cell<Lyt> c{x, y, z};

                    write_cell(c);

                    if (ps.create_inter_layer_via_cells && (z != lyt.z()))
                    {
                        write_via_cell(c, via_layer_buffer);
                    }
                }  // row done

                os << '\n';

                if (ps.create_inter_layer_via_cells && (z != lyt.z()))
                {
                    via_layer_buffer << '\n';
                }

            }  // layer done

            if (ps.create_inter_layer_via_cells && (z != lyt.z()))
            {
                write_layer_separator();

                // write via cells
                os << via_layer_buffer.rdbuf();
            }
        }  // layout done

        // add one final layer separator
        write_layer_separator();
    }

    void write_labeled_cell(const labeled_cell& lc)
    {
        os << fmt::format("{}:\n", lc.designator);

        // if cell is PI
        if (lyt.is_pi(lc.c))
        {
            os << qca_stack::ITEM << qca_stack::PRIMARY_INPUT << '\n';
        }
        // if cell is PO
        if (lyt.is_po(lc.c))
        {
            os << qca_stack::ITEM << qca_stack::PRIMARY_OUTPUT << '\n';
        }
        // if cell has a name
        if (const auto cell_name = lyt.get_cell_name(lc.c); !cell_name.empty())
        {
            os << qca_stack::ITEM << fmt::format(qca_stack::LABEL, cell_name) << '\n';
        }

        // every cell has a clock
        os << qca_stack::ITEM << fmt::format(qca_stack::CLOCK, lyt.get_clock_number(lc.c)) << '\n';

        // 'number', 'propagate', and 'offset' are not supported

        // end block
        os << "\n";
    }

    void write_cell_definition()
    {
        for (const auto& lc : labeled_cells) { write_labeled_cell(lc); }
    }
};

}  // namespace detail

/**
 * Writes a cell-level QCA layout to an fqca file provided as an output stream. The format is used by QCA-STACK by
 * Willem Lambooy (https://github.com/wlambooy/QCA-STACK).
 *
 * Might throw an 'out_of_cell_names_exception' in case there are more I/O cells in the layout than lowercase +
 * uppercase letters in the English alphabet.
 *
 * @tparam Lyt The layout type to be written. Must be a clocked cell-level QCA layout.
 * @param lyt The cell-level QCA layout.
 * @param os The output stream to write into.
 * @param ps Parameters.
 */
template <typename Lyt>
void write_fqca_layout(const Lyt& lyt, std::ostream& os, write_fqca_layout_params ps = {})
{
    static_assert(is_cell_level_layout_v<Lyt>, "Lyt is not a cell-level layout");
    static_assert(std::is_same_v<technology<Lyt>, qca_technology>, "Lyt must be a QCA layout");

    detail::write_fqca_layout_impl p{lyt, os, ps};

    p.run();
}
/**
 * Writes a cell-level QCA layout to an fqca file provided as a file name. The format is used by QCA-STACK by
 * Willem Lambooy (https://github.com/wlambooy/QCA-STACK).
 *
 * Might throw an 'out_of_cell_names_exception' in case there are more I/O cells in the layout than lowercase +
 * uppercase letters in the English alphabet.
 *
 * @tparam Lyt The layout type to be written. Must be a clocked cell-level QCA layout.
 * @param lyt The cell-level QCA layout.
 * @param filename The file name to create and write into. Should preferably use the ".fqca" extension.
 * @param ps Parameters.
 */
template <typename Lyt>
void write_fqca_layout(const Lyt& lyt, const std::string& filename, write_fqca_layout_params ps = {})
{
    std::ofstream os{filename.c_str(), std::ofstream::out};

    if (!os.is_open())
        throw std::ofstream::failure("could not open file");

    write_fqca_layout(lyt, os, ps);
    os.close();
}

}  // namespace fiction

#endif  // FICTION_WRITE_FQCA_LAYOUT_HPP