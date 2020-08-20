#pragma once

namespace smt
{
  typedef size_t var;
  static const var FALSE_var = 0;
  static const var TRUE_var = 1;

  typedef unsigned short int lbool;
  static const lbool False = 0;
  static const lbool True = 1;
  static const lbool Undefined = 2;
} // namespace smt