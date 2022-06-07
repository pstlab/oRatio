#pragma once

#include <cstddef>

namespace smt
{
  using var = size_t;
  constexpr var FALSE_var = 0;

  using lbool = unsigned short int;
  constexpr lbool False = 0;
  constexpr lbool True = 1;
  constexpr lbool Undefined = 2;

  using I = long;
} // namespace smt