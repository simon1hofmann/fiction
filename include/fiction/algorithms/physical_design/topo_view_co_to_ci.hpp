/*!
 \file topo_view_co_to_ci.hpp
 \brief Reimplements foreach_node to guarantee topological order

 \author Alessandro Tempia Calvino
 \author Heinz Riener
 \author Mathias Soeken
 \author Max Austin
*/

#pragma once

#include <mockturtle/networks/detail/foreach.hpp>
#include <mockturtle/traits.hpp>
#include <mockturtle/views/immutable_view.hpp>

#include <cassert>
#include <optional>
#include <vector>

namespace fiction
{

/*! \brief Ensures topological order for of all nodes reachable from the outputs.
*
* Overrides the interface methods `foreach_node`, `foreach_gate`,
* `size`, `num_gates`.
*
* This class computes *on construction* a topological order of the nodes which
* are reachable from the outputs.  Constant nodes and primary inputs will also
* be considered even if they are not reachable from the outputs.  Further,
* constant nodes and primary inputs will be visited first before any gate node
* is visited.  Constant nodes precede primary inputs, and primary inputs are
* visited in the same order in which they were created.
*
* Since the topological order is computed only once when creating an instance,
* this view disables changes to the network interface.  Also, since only
* reachable nodes are traversed, not all network nodes may be called in
* `foreach_node` and `foreach_gate`.
*
* **Required network functions:**
* - `get_constant`
* - `foreach_pi`
* - `foreach_po`
* - `foreach_fanin`
* - `incr_trav_id`
* - `set_visited`
* - `trav_id`
* - `visited`
*
* Example
*
  \verbatim embed:rst

  .. code-block:: c++

     // create network somehow; aig may not be in topological order
     aig_network aig = ...;

     // create a topological view on the network
     topo_view_co_to_ci aig_topo{aig};

     // call algorithm that requires topological order
     cut_enumeration( aig_topo );
  \endverbatim
*/
template <class Ntk, bool sorted = mockturtle::is_topologically_sorted_v<Ntk>>
class topo_view_co_to_ci
{};

template <typename Ntk>
class topo_view_co_to_ci<Ntk, false> : public mockturtle::immutable_view<Ntk>
{
  public:
    using storage                                 = typename Ntk::storage;
    using node                                    = typename Ntk::node;
    using signal                                  = typename Ntk::signal;
    static constexpr bool is_topologically_sorted = true;

    /*! \brief Default constructor.
     *
     * Constructs topological view on another network.
     */
    topo_view_co_to_ci(Ntk const& ntk) : mockturtle::immutable_view<Ntk>(ntk)
    {
        static_assert(mockturtle::is_network_type_v<Ntk>, "Ntk is not a network type");
        static_assert(mockturtle::has_size_v<Ntk>, "Ntk does not implement the size method");
        static_assert(mockturtle::has_get_constant_v<Ntk>, "Ntk does not implement the get_constant method");
        static_assert(mockturtle::has_foreach_pi_v<Ntk>, "Ntk does not implement the foreach_pi method");
        static_assert(mockturtle::has_foreach_po_v<Ntk>, "Ntk does not implement the foreach_po method");
        static_assert(mockturtle::has_foreach_fanin_v<Ntk>, "Ntk does not implement the foreach_fanin method");
        static_assert(mockturtle::has_incr_trav_id_v<Ntk>, "Ntk does not implement the incr_trav_id method");
        static_assert(mockturtle::has_set_visited_v<Ntk>, "Ntk does not implement the set_visited method");
        static_assert(mockturtle::has_trav_id_v<Ntk>, "Ntk does not implement the trav_id method");
        static_assert(mockturtle::has_visited_v<Ntk>, "Ntk does not implement the visited method");

        update_topo();
    }

    /*! \brief Reimplementation of `size`. */
    auto size() const
    {
        return static_cast<uint32_t>(topo_order.size());
    }

    /*! \brief Reimplementation of `num_gates`. */
    auto num_gates() const
    {
        uint32_t const offset = 1u + this->num_pis() +
                                (this->get_node(this->get_constant(true)) != this->get_node(this->get_constant(false)));
        return static_cast<uint32_t>(topo_order.size() - offset);
    }

    /*! \brief Reimplementation of `node_to_index`. */
    uint32_t node_to_index(node const& n) const
    {
        return std::distance(std::begin(topo_order), std::find(std::begin(topo_order), std::end(topo_order), n));
    }

    /*! \brief Reimplementation of `index_to_node`. */
    node index_to_node(uint32_t index) const
    {
        return topo_order.at(index);
    }

    /*! \brief Reimplementation of `foreach_node`. */
    template <typename Fn>
    void foreach_node(Fn&& fn) const
    {
        mockturtle::detail::foreach_element(topo_order.begin(), topo_order.end(), fn);
    }

    /*! \brief Implementation of `foreach_node` in reverse topological order. */
    template <typename Fn>
    void foreach_node_reverse(Fn&& fn) const
    {
        mockturtle::detail::foreach_element(topo_order.rbegin(), topo_order.rend(), fn);
    }

    /*! \brief Reimplementation of `foreach_gate`. */
    template <typename Fn>
    void foreach_gate(Fn&& fn) const
    {
        uint32_t const offset = 1u + this->num_pis() +
                                (this->get_node(this->get_constant(true)) != this->get_node(this->get_constant(false)));
        mockturtle::detail::foreach_element(topo_order.begin() + offset, topo_order.end(), fn);
    }

    /*! \brief Implementation of `foreach_gate` in reverse topological order. */
    template <typename Fn>
    void foreach_gate_reverse(Fn&& fn) const
    {
        uint32_t const offset = 1u + this->num_pis() +
                                (this->get_node(this->get_constant(true)) != this->get_node(this->get_constant(false)));
        mockturtle::detail::foreach_element(topo_order.rbegin(), topo_order.rend() - offset, fn);
    }

    void update_topo()
    {
        this->incr_trav_id();
        this->incr_trav_id();
        topo_order.reserve(this->size());

        /* constants and PIs */
        const auto c0 = this->get_node(this->get_constant(false));
        topo_order.push_back(c0);
        this->set_visited(c0, this->trav_id());

        if (const auto c1 = this->get_node(this->get_constant(true)); this->visited(c1) != this->trav_id())
        {
            topo_order.push_back(c1);
            this->set_visited(c1, this->trav_id());
        }

        /*this->foreach_ci( [this]( auto n ) {
                             if ( this->visited( n ) != this->trav_id() )
                             {
                                 topo_order.push_back( n );
                                 this->set_visited( n, this->trav_id() );
                             }
                         } );*/

        Ntk::foreach_co(
            [this](auto f)
            {
                /* node was already visited */
                if (this->visited(this->get_node(f)) == this->trav_id())
                    return;

                create_topo_rec(this->get_node(f));
            });
    }

  private:
    void create_topo_rec(node const& n)
    {
        /* is permanently marked? */
        if (this->visited(n) == this->trav_id())
            return;

        /* ensure that the node is not temporarily marked */
        assert(this->visited(n) != this->trav_id() - 1);

        /* mark node temporarily */
        this->set_visited(n, this->trav_id() - 1);

        /* mark children */
        this->foreach_fanin(n, [this](signal const& f) { create_topo_rec(this->get_node(f)); });

        /* mark node n permanently */
        this->set_visited(n, this->trav_id());

        /* visit node */
        topo_order.push_back(n);
    }

  private:
    std::vector<node> topo_order;
};

template <typename Ntk>
class topo_view_co_to_ci<Ntk, true> : public Ntk
{
  public:
    topo_view_co_to_ci(Ntk const& ntk) : Ntk(ntk) {}
};

template <class T>
topo_view_co_to_ci(T const&) -> topo_view_co_to_ci<T>;

template <class T>
topo_view_co_to_ci(T const&, typename T::signal const&) -> topo_view_co_to_ci<T>;

}  // namespace fiction