#pragma once

#include "scope.h"
#include "env.h"
#include "sat_core.h"
#include "lra_theory.h"
#include "ov_theory.h"
#ifdef BUILD_GUI
#include <iostream>

#define LOG(msg) std::cout << __FILE__ << "(" << __LINE__ << "): " << msg << std::endl
#else
#define LOG(msg)
#endif

#define BOOL_KEYWORD "bool"
#define INT_KEYWORD "int"
#define REAL_KEYWORD "real"
#define STRING_KEYWORD "string"

namespace riddle::ast
{
class compilation_unit;
} // namespace riddle::ast

namespace ratio
{

class atom;
class disjunction;
#ifdef BUILD_GUI
class core_listener;
#endif

namespace ast
{
class formula_statement;
class disjunction_statement;
class method_declaration;
class predicate_declaration;
class typedef_declaration;
class enum_declaration;
class class_declaration;
} // namespace ast

class core : public scope, public env
{
  friend class var_item;
  friend class ast::formula_statement;
  friend class ast::disjunction_statement;
  friend class ast::method_declaration;
  friend class ast::predicate_declaration;
  friend class ast::typedef_declaration;
  friend class ast::enum_declaration;
  friend class ast::class_declaration;
#ifdef BUILD_GUI
  friend class core_listener;
  friend class type;
  friend class scope;
#endif

public:
  core();
  core(const core &orig) = delete;
  ~core();

  smt::sat_core &get_sat_core() { return sat_cr; }     // returns the sat core..
  smt::lra_theory &get_lra_theory() { return lra_th; } // returns the linear-real-arithmetic theory..
  smt::ov_theory &get_ov_theory() { return ov_th; }    // returns the object-variable theory..

  void read(const std::string &script);
  void read(const std::vector<std::string> &files);

  bool_expr new_bool();
  bool_expr new_bool(const bool &val);
  arith_expr new_int();
  arith_expr new_int(const smt::I &val);
  arith_expr new_real();
  arith_expr new_real(const smt::rational &val);
  string_expr new_string();
  string_expr new_string(const std::string &val);
  virtual expr new_enum(const type &tp, const std::unordered_set<item *> &allowed_vals);

  bool_expr negate(bool_expr var);
  bool_expr eq(bool_expr left, bool_expr right);
  bool_expr conj(const std::vector<bool_expr> &exprs);
  bool_expr disj(const std::vector<bool_expr> &exprs);
  bool_expr exct_one(const std::vector<bool_expr> &exprs);

  arith_expr add(const std::vector<arith_expr> &exprs);
  arith_expr sub(const std::vector<arith_expr> &exprs);
  arith_expr mult(const std::vector<arith_expr> &exprs);
  arith_expr div(const std::vector<arith_expr> &exprs);
  arith_expr minus(arith_expr ex);

  bool_expr lt(arith_expr left, arith_expr right);
  bool_expr leq(arith_expr left, arith_expr right);
  bool_expr eq(arith_expr left, arith_expr right);
  bool_expr geq(arith_expr left, arith_expr right);
  bool_expr gt(arith_expr left, arith_expr right);

  bool_expr eq(expr i0, expr i1);

  void assert_facts(const std::vector<smt::lit> &facts);

private:
  expr new_enum(const type &tp, const std::vector<smt::var> &vars, const std::vector<item *> &vals);

protected:
  void new_methods(const std::vector<const method *> &ms);
  void new_types(const std::vector<type *> &ts);
  void new_predicates(const std::vector<predicate *> &ps);

public:
  const field &get_field(const std::string &name) const override; // returns the field having the given name..

  const method &get_method(const std::string &name, const std::vector<const type *> &ts) const override;
  std::vector<const method *> get_methods() const noexcept override
  {
    std::vector<const method *> c_methods;
    for (const auto &ms : methods)
      c_methods.insert(c_methods.begin(), ms.second.begin(), ms.second.end());
    return c_methods;
  }

  type &get_type(const std::string &name) const override;
  std::map<std::string, type *> get_types() const noexcept override { return types; }

  predicate &get_predicate(const std::string &name) const override;
  std::map<std::string, predicate *> get_predicates() const noexcept override { return predicates; }

  expr get(const std::string &name) const override;

  smt::lbool bool_value(const bool_expr &x) const noexcept;                          // the current value of the given boolean expression..
  smt::inf_rational arith_lb(const arith_expr &x) const noexcept;                    // the current lower bound of the given arith expression..
  smt::inf_rational arith_ub(const arith_expr &x) const noexcept;                    // the current upper bound of the given arith expression..
  smt::inf_rational arith_value(const arith_expr &x) const noexcept;                 // the current value of the given arith expression..
  std::unordered_set<smt::var_value *> enum_value(const var_expr &x) const noexcept; // the current allowed values of the given enum expression..

  virtual void solve() = 0;

private:
  virtual void new_fact(atom &atm) = 0;
  virtual void new_goal(atom &atm) = 0;
  virtual void new_disjunction(context &ctx, const disjunction &disj) = 0;

protected:
  smt::var get_ni() { return ni; }
  void set_ni(const smt::var &v)
  {
    tmp_ni = ni;
    ni = v;
  }

  void restore_ni() { ni = tmp_ni; }

public:
  std::string to_string() const noexcept;

private:
  std::string to_string(const item *const i) const noexcept;
  std::string to_string(const atom *const i) const noexcept;
  std::string to_string(const std::map<std::string, expr> &items) const noexcept;

private:
  smt::sat_core sat_cr;   // the sat core..
  smt::lra_theory lra_th; // the linear-real-arithmetic theory..
  smt::ov_theory ov_th;   // the object-variable theory..

  std::vector<riddle::ast::compilation_unit *> cus; // the compilation units..

  std::map<std::string, std::vector<const method *>> methods; // the methods, indexed by their name, defined within this type..
  std::map<std::string, type *> types;                        // the types, indexed by their name, defined within this core..
  std::map<std::string, predicate *> predicates;              // the predicates, indexed by their name, defined within this core..

  smt::var tmp_ni;             // the temporary controlling variable, used for restoring the controlling variable..
  smt::var ni = smt::TRUE_var; // the controlling variable..

#ifdef BUILD_GUI
private:
  std::vector<core_listener *> listeners; // the core listeners..

protected:
  void fire_log(const std::string msg) const;
  void fire_read(const std::string &script) const;
  void fire_read(const std::vector<std::string> &files) const;
  void fire_state_changed() const;
#endif
};

class inconsistency_exception : public std::exception
{
  const char *what() const noexcept override { return "an inconsistency has been found.."; }
};

class unsolvable_exception : public std::exception
{
  const char *what() const noexcept override { return "the problem is unsolvable.."; }
};
} // namespace ratio