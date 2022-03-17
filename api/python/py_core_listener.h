#pragma once

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace ratio
{
  class py_core_listener
  {
  public:
    py_core_listener() {}
    ~py_core_listener() {}

    void log(const py::str &) {}
    void read_script(const py::str &) {}
    void read_files(const py::list &) {}

    void state_changed() {}

    void started_solving() {}
    void solution_found() {}
    void inconsistent_problem() {}
  };
} // namespace ratio
