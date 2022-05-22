#pragma once

#include <cstddef>

namespace smt
{
  typedef size_t var;
  constexpr var FALSE_var = 0;

  typedef unsigned short int lbool;
  constexpr lbool False = 0;
  constexpr lbool True = 1;
  constexpr lbool Undefined = 2;

  typedef long I;
} // namespace smt