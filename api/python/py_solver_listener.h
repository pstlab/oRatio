#pragma once

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace ratio
{
  class py_solver_listener
  {
  public:
    py_solver_listener() {}
    ~py_solver_listener() {}

    void flaw_created(const py::object &f) {}
    void flaw_state_changed(const py::object &f) {}
    void flaw_cost_changed(const py::object &f) {}
    void flaw_position_changed(const py::object &f) {}
    void current_flaw(const py::object &f) {}

    void resolver_created(const py::object &r) {}
    void resolver_state_changed(const py::object &r) {}
    void current_resolver(const py::object &r) {}

    void causal_link_added(const py::object &f, const py::object &r) {}
  };
} // namespace ratio
