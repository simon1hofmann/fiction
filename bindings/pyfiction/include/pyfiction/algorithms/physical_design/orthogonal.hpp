//
// Created by marcel on 08.06.22.
//

#ifndef PYFICTION_ORTHOGONAL_HPP
#define PYFICTION_ORTHOGONAL_HPP

#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "pyfiction/types.hpp"

#include <fiction/algorithms/physical_design/orthogonal.hpp>

#include <sstream>

namespace pyfiction
{

/**
 * OGD-based physical design.
 */
inline void orthogonal(pybind11::module& m)
{
    namespace py = pybind11;
    using namespace pybind11::literals;

    py::class_<fiction::orthogonal_physical_design_params>(m, "orthogonal_params").def(py::init<>())

        ;

    py::class_<fiction::orthogonal_physical_design_stats>(m, "orthogonal_stats")
        .def(py::init<>())
        .def("__repr__",
             [](const fiction::orthogonal_physical_design_stats& stats)
             {
                 std::stringstream stream{};
                 stats.report(stream);
                 return stream.str();
             })

        ;

    m.def("orthogonal", &fiction::orthogonal<py_cartesian_gate_layout, py_logic_network>, "network"_a,
          "parameters"_a = fiction::orthogonal_physical_design_params{}, "statistics"_a = nullptr);
}

}  // namespace pyfiction

#endif  // PYFICTION_ORTHOGONAL_HPP