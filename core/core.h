#pragma once

#include "scope.h"
#include "env.h"
#include "sat_core.h"
#include "lra_theory.h"
#include "ov_theory.h"
#include "idl_theory.h"
#include "rdl_theory.h"
#ifdef VERBOSE_LOG
#include <iostream>

#define LOG(msg) std::cout << __FILE__ << "(" << __LINE__ << "): " << msg << '\n'
#else
#define LOG(msg)
#endif

#if defined(VERBOSE_LOG) || defined(BUILD_LISTENERS)
#define RECOMPUTE_NAMES() recompute_names()
#else
#define RECOMPUTE_NAMES()
#endif

#ifdef BUILD_LISTENERS
#define FIRE_LOG(msg) fire_log(msg)
#define FIRE_READ(rddl) fire_read(rddl)
#define FIRE_STATE_CHANGED() fire_state_changed()
#define FIRE_STARTED_SOLVING() fire_started_solving()
#define FIRE_SOLUTION_FOUND() fire_solution_found()
#define FIRE_INCONSISTENT_PROBLEM() fire_inconsistent_problem()
#else
#define FIRE_LOG(msg)
#define FIRE_READ(rddl)
#define FIRE_STATE_CHANGED()
#define FIRE_STARTED_SOLVING()
#define FIRE_SOLUTION_FOUND()
#define FIRE_INCONSISTENT_PROBLEM()
#endif

namespace riddle::ast
{
  class compilation_unit;
} // namespace riddle::ast

namespace ratio
{
  class atom;
  class conjunction;
#ifdef BUILD_LISTENERS
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
#ifdef BUILD_LISTENERS
    friend class core_listener;
    friend class type;
    friend class scope;
#endif

  public:
    CORE_EXPORT core();
    core(const core &orig) = delete;
    CORE_EXPORT ~core();

    inline smt::sat_core &get_sat_core() noexcept { return sat_cr; }     // returns the sat core..
    inline smt::lra_theory &get_lra_theory() noexcept { return lra_th; } // returns the linear-real-arithmetic theory..
    inline smt::ov_theory &get_ov_theory() noexcept { return ov_th; }    // returns the object-variable theory..
    inline smt::idl_theory &get_idl_theory() noexcept { return idl_th; } // returns the integer difference logic theory..
    inline smt::rdl_theory &get_rdl_theory() noexcept { return rdl_th; } // returns the real difference logic theory..

    CORE_EXPORT virtual void read(const std::string &script);             // parses the given riddle script..
    CORE_EXPORT virtual void read(const std::vector<std::string> &files); // parses the given riddle files..

    CORE_EXPORT virtual bool_expr new_bool() noexcept;                   // creates a new boolean variable..
    CORE_EXPORT bool_expr new_bool(const bool &val) noexcept;            // creates a new boolean literal..
    CORE_EXPORT virtual arith_expr new_int() noexcept;                   // creates a new integer variable..
    CORE_EXPORT arith_expr new_int(const smt::I &val) noexcept;          // creates a new integer literal..
    CORE_EXPORT arith_expr new_real() noexcept;                          // creates a new real variable..
    CORE_EXPORT arith_expr new_real(const smt::rational &val) noexcept;  // creates a new real literal..
    CORE_EXPORT arith_expr new_tp() noexcept;                            // creates a new time-point variable..
    CORE_EXPORT arith_expr new_tp(const smt::rational &val) noexcept;    // creates a new time-point literal..
    CORE_EXPORT string_expr new_string() noexcept;                       // creates a new string variable..
    CORE_EXPORT string_expr new_string(const std::string &val) noexcept; // creates a new string literal..
    CORE_EXPORT virtual expr new_enum(const type &tp, const std::vector<item *> &allowed_vals);

    CORE_EXPORT bool_expr negate(bool_expr var) noexcept;
    CORE_EXPORT bool_expr eq(bool_expr left, bool_expr right) noexcept;
    CORE_EXPORT bool_expr conj(const std::vector<bool_expr> &exprs) noexcept;
    CORE_EXPORT virtual bool_expr disj(const std::vector<bool_expr> &exprs) noexcept;
    CORE_EXPORT bool_expr exct_one(const std::vector<bool_expr> &exprs) noexcept;

    CORE_EXPORT arith_expr add(const std::vector<arith_expr> &exprs) noexcept;
    CORE_EXPORT arith_expr sub(const std::vector<arith_expr> &exprs) noexcept;
    CORE_EXPORT arith_expr mult(const std::vector<arith_expr> &exprs) noexcept;
    CORE_EXPORT arith_expr div(const std::vector<arith_expr> &exprs) noexcept;
    CORE_EXPORT arith_expr minus(arith_expr ex) noexcept;

    CORE_EXPORT bool_expr lt(arith_expr left, arith_expr right) noexcept;
    CORE_EXPORT bool_expr leq(arith_expr left, arith_expr right) noexcept;
    CORE_EXPORT bool_expr eq(arith_expr left, arith_expr right) noexcept;
    CORE_EXPORT bool_expr geq(arith_expr left, arith_expr right) noexcept;
    CORE_EXPORT bool_expr gt(arith_expr left, arith_expr right) noexcept;

    CORE_EXPORT bool_expr eq(expr i0, expr i1) noexcept;

    CORE_EXPORT void assert_facts(const std::vector<smt::lit> &facts);
    CORE_EXPORT void assert_facts(const std::vector<bool_expr> &facts);

  private:
    expr new_enum(const type &tp, const std::vector<smt::lit> &lits, const std::vector<item *> &vals) noexcept;

    const type &get_type(const std::vector<arith_expr> &exprs) const;

  protected:
    void new_methods(const std::vector<const method *> &ms) noexcept;
    CORE_EXPORT void new_types(const std::vector<type *> &ts) noexcept;
    void new_predicates(const std::vector<predicate *> &ps) noexcept;

  public:
    CORE_EXPORT const field &get_field(const std::string &name) const override; // returns the field having the given name..

    CORE_EXPORT const method &get_method(const std::string &name, const std::vector<const type *> &ts) const override;
    std::vector<const method *> get_methods() const noexcept override
    {
      std::vector<const method *> c_methods;
      for (const auto &[mthd_name, mthds] : methods)
        c_methods.insert(c_methods.cbegin(), mthds.cbegin(), mthds.cend());
      return c_methods;
    }

    CORE_EXPORT type &get_type(const std::string &name) const override;
    std::map<std::string, type *> get_types() const noexcept override { return types; }

    CORE_EXPORT predicate &get_predicate(const std::string &name) const override;
    std::map<std::string, predicate *> get_predicates() const noexcept override { return predicates; }

    CORE_EXPORT expr get(const std::string &name) const override;

    CORE_EXPORT smt::lbool bool_value(const bool_expr &x) const noexcept;                                // the current value of the given boolean expression..
    std::pair<smt::inf_rational, smt::inf_rational> arith_bounds(const arith_expr &x) const noexcept;    // the current bounds of the given arith expression..
    CORE_EXPORT smt::inf_rational arith_value(const arith_expr &x) const noexcept;                       // the current value of the given arith expression..
    CORE_EXPORT std::unordered_set<const smt::var_value *> enum_value(const var_expr &x) const noexcept; // the current allowed values of the given enum expression..

    virtual void solve() = 0;

  private:
    virtual void new_atom(atom &atm, const bool &is_fact) = 0;
    virtual void new_disjunction(context &ctx, const std::vector<const conjunction *> &conjs) = 0;

  protected:
    inline smt::lit get_ni() noexcept { return ni; }
    inline void set_ni(const smt::lit &v) noexcept
    {
      tmp_ni = ni;
      ni = v;
    }

    inline void restore_ni() noexcept { ni = tmp_ni; }

  public:
    CORE_EXPORT smt::json to_json() const noexcept override;

    CORE_EXPORT friend std::ostream &operator<<(std::ostream &os, const core &cr);

  private:
    smt::sat_core sat_cr;   // the sat core..
    smt::lra_theory lra_th; // the linear-real-arithmetic theory..
    smt::ov_theory ov_th;   // the object-variable theory..
    smt::idl_theory idl_th; // the integer difference logic theory..
    smt::rdl_theory rdl_th; // the real difference logic theory..

#if defined(VERBOSE_LOG) || defined(BUILD_LISTENERS)
  public:
    const std::string &guess_name(const item &itm) const noexcept { return expr_names.at(&itm); }

  private:
    void recompute_names() noexcept;

    std::unordered_map<const item *, const std::string> expr_names;
#endif

    std::vector<riddle::ast::compilation_unit *> cus; // the compilation units..

    std::map<std::string, std::vector<const method *>> methods; // the methods, indexed by their name, defined within this type..
    std::map<std::string, type *> types;                        // the types, indexed by their name, defined within this core..
    std::map<std::string, predicate *> predicates;              // the predicates, indexed by their name, defined within this core..

    smt::lit tmp_ni;             // the temporary controlling literal, used for restoring the controlling literal..
    smt::lit ni = smt::TRUE_lit; // the controlling literal..

#ifdef BUILD_LISTENERS
  private:
    std::vector<core_listener *> listeners; // the core listeners..

  protected:
    void fire_log(const std::string msg) const noexcept;
    void fire_read(const std::string &script) const noexcept;
    void fire_read(const std::vector<std::string> &files) const noexcept;
    CORE_EXPORT void fire_state_changed() const noexcept;
    CORE_EXPORT void fire_started_solving() const noexcept;
    CORE_EXPORT void fire_solution_found() const noexcept;
    void fire_inconsistent_problem() const noexcept;
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