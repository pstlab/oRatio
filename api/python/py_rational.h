#pragma once

#include <pybind11/pybind11.h>
#include "rational.h"

namespace py = pybind11;

namespace ratio
{
  class py_rational
  {
  public:
    py_rational(smt::rational &val) : val(val) {}
    py_rational(const py::int_ &num) : val(num) {}
    py_rational(const py::int_ &num, const py::int_ &den) : val(num, den) {}
    ~py_rational() {}

    py::int_ numerator() const { return val.numerator(); }
    py::int_ denominator() const { return val.denominator(); }

  private:
    smt::rational val;
  };
} // namespace ratio
