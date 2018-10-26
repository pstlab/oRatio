#pragma once

#include "scope.h"
#include "env.h"
#include "sat_core.h"
#include "lra_theory.h"
#include "ov_theory.h"

#define BOOL_KEYWORD "bool"
#define INT_KEYWORD "int"
#define REAL_KEYWORD "real"
#define STRING_KEYWORD "string"

namespace ratio
{

class core : public scope, public env
{
  friend class var_item;

public:
  core();
  core(const core &orig) = delete;
  ~core();

  smt::sat_core &get_sat_core() { return sat_cr; }     // returns the sat core..
  smt::lra_theory &get_lra_theory() { return lra_th; } // returns the linear-real-arithmetic theory..
  smt::ov_theory &get_ov_theory() { return ov_th; }    // returns the object-variable theory..

  bool_expr new_bool();
  bool_expr new_bool(const bool &val);
  arith_expr new_int();
  arith_expr new_int(const smt::I &val);
  arith_expr new_real();
  arith_expr new_real(const smt::rational &val);
  string_expr new_string();
  string_expr new_string(const std::string &val);
  virtual expr new_enum(const type &tp, const std::unordered_set<item *> &allowed_vals);

private:
  expr new_enum(const type &tp, const std::vector<smt::var> &vars, const std::vector<item *> &vals);

public:
  field &get_field(const std::string &name) const override; // returns the field having the given name..

  expr get(const std::string &name) const override;

private:
  smt::sat_core sat_cr;   // the sat core..
  smt::lra_theory lra_th; // the linear-real-arithmetic theory..
  smt::ov_theory ov_th;   // the object-variable theory..

  std::map<std::string, type *> types;
};
} // namespace ratio