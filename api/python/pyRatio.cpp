#include <pybind11/pybind11.h>
#include "py_solver.h"
#include "py_core_listener.h"
#include "py_solver_listener.h"
#include "py_rational.h"

namespace py = pybind11;
using namespace ratio;

PYBIND11_MODULE(pyRatio, m)
{
    m.doc() = "The oRatio Python API";

    py::class_<py_core_listener>(m, "CoreListener")
        .def(py::init<>())
        .def("log", &py_core_listener::log)
        .def("read_script", &py_core_listener::read_script)
        .def("read_files", &py_core_listener::read_files)
        .def("state_changed", &py_core_listener::state_changed)
        .def("started_solving", &py_core_listener::started_solving)
        .def("solution_found", &py_core_listener::solution_found)
        .def("inconsistent_problem", &py_core_listener::inconsistent_problem);

    py::class_<py_solver_listener>(m, "SolverListener")
        .def(py::init<>())
        .def("flaw_created", &py_solver_listener::flaw_created)
        .def("flaw_state_changed", &py_solver_listener::flaw_state_changed)
        .def("flaw_cost_changed", &py_solver_listener::flaw_cost_changed)
        .def("flaw_position_changed", &py_solver_listener::flaw_position_changed)
        .def("current_flaw", &py_solver_listener::current_flaw)
        .def("resolver_created", &py_solver_listener::resolver_created)
        .def("resolver_state_changed", &py_solver_listener::resolver_state_changed)
        .def("current_resolver", &py_solver_listener::current_resolver)
        .def("causal_link_added", &py_solver_listener::causal_link_added);

    py::class_<py_solver>(m, "Solver")
        .def(py::init<>())
        .def("read_script", &py_solver::read_script)
        .def("read_files", &py_solver::read_files)
        .def("solve", &py_solver::solve)
        .def("add_core_listener", &py_solver::add_core_listener)
        .def("remove_core_listener", &py_solver::remove_core_listener)
        .def("add_solver_listener", &py_solver::add_solver_listener)
        .def("remove_solver_listener", &py_solver::remove_solver_listener);

    py::class_<py_rational>(m, "Rational")
        .def(py::init<const py::int_ &>())
        .def(py::init<const py::int_ &, const py::int_ &>())
        .def("numerator", &py_rational::numerator)
        .def("denominator", &py_rational::denominator);
}