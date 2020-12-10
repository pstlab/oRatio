#pragma once

#include "scope.h"
#include "env.h"
#include "sat_core.h"
#include "lra_theory.h"
#include "ov_theory.h"
#include "idl_theory.h"
#include "rdl_theory.h"
#ifdef BUILD_GUI
#include <iostream>

#define LOG(msg) std::cout << __FILE__ << "(" << __LINE__ << "): " << msg << std::endl
#else
#define LOG(msg)
#endif

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

    smt::sat_core &get_sat_core() noexcept { return sat_cr; }     // returns the sat core..
    smt::lra_theory &get_lra_theory() noexcept { return lra_th; } // returns the linear-real-arithmetic theory..
    smt::ov_theory &get_ov_theory() noexcept { return ov_th; }    // returns the object-variable theory..
    smt::idl_theory &get_idl_theory() noexcept { return idl_th; } // returns the integer difference logic theory..
    smt::rdl_theory &get_rdl_theory() noexcept { return rdl_th; } // returns the real difference logic theory..

    void read(const std::string &script);             // parses the given riddle script..
    void read(const std::vector<std::string> &files); // parses the given riddle files..

    virtual bool_expr new_bool() noexcept;                   // creates a new boolean variable..
    bool_expr new_bool(const bool &val) noexcept;            // creates a new boolean literal..
    virtual arith_expr new_int() noexcept;                   // creates a new integer variable..
    arith_expr new_int(const smt::I &val) noexcept;          // creates a new integer literal..
    arith_expr new_real() noexcept;                          // creates a new real variable..
    arith_expr new_real(const smt::rational &val) noexcept;  // creates a new real literal..
    arith_expr new_tp() noexcept;                            // creates a new time-point variable..
    arith_expr new_tp(const smt::rational &val) noexcept;    // creates a new time-point literal..
    string_expr new_string() noexcept;                       // creates a new string variable..
    string_expr new_string(const std::string &val) noexcept; // creates a new string literal..
    virtual expr new_enum(const type &tp, const std::vector<item *> &allowed_vals);

    bool_expr negate(bool_expr var) noexcept;
    bool_expr eq(bool_expr left, bool_expr right) noexcept;
    bool_expr conj(const std::vector<bool_expr> &exprs) noexcept;
    virtual bool_expr disj(const std::vector<bool_expr> &exprs) noexcept;
    bool_expr exct_one(const std::vector<bool_expr> &exprs) noexcept;

    arith_expr add(const std::vector<arith_expr> &exprs) noexcept;
    arith_expr sub(const std::vector<arith_expr> &exprs) noexcept;
    arith_expr mult(const std::vector<arith_expr> &exprs) noexcept;
    arith_expr div(const std::vector<arith_expr> &exprs) noexcept;
    arith_expr minus(arith_expr ex) noexcept;

    bool_expr lt(arith_expr left, arith_expr right) noexcept;
    bool_expr leq(arith_expr left, arith_expr right) noexcept;
    bool_expr eq(arith_expr left, arith_expr right) noexcept;
    bool_expr geq(arith_expr left, arith_expr right) noexcept;
    bool_expr gt(arith_expr left, arith_expr right) noexcept;

    bool_expr eq(expr i0, expr i1) noexcept;

    void assert_facts(const std::vector<smt::lit> &facts);
    void assert_facts(const std::vector<bool_expr> &facts);

  private:
    expr new_enum(const type &tp, const std::vector<smt::lit> &lits, const std::vector<item *> &vals) noexcept;

    const type &get_type(const std::vector<arith_expr> &exprs) const;

  protected:
    void new_methods(const std::vector<const method *> &ms) noexcept;
    void new_types(const std::vector<type *> &ts) noexcept;
    void new_predicates(const std::vector<predicate *> &ps) noexcept;

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

    smt::lbool bool_value(const bool_expr &x) const noexcept;                                         // the current value of the given boolean expression..
    std::pair<smt::inf_rational, smt::inf_rational> arith_bounds(const arith_expr &x) const noexcept; // the current bounds of the given arith expression..
    smt::inf_rational arith_value(const arith_expr &x) const noexcept;                                // the current value of the given arith expression..
    std::unordered_set<const smt::var_value *> enum_value(const var_expr &x) const noexcept;          // the current allowed values of the given enum expression..

    virtual void solve() = 0;

  private:
    virtual void new_atom(atom &atm, const bool &is_fact) = 0;
    virtual void new_disjunction(context &ctx, const disjunction &disj) = 0;

  protected:
    smt::lit get_ni() noexcept { return ni; }
    void set_ni(const smt::lit &v) noexcept
    {
      tmp_ni = ni;
      ni = v;
    }

    void restore_ni() noexcept { ni = tmp_ni; }

  public:
    smt::json to_json() const noexcept;

    friend std::ostream &operator<<(std::ostream &os, const core &cr);

  private:
    smt::sat_core sat_cr;   // the sat core..
    smt::lra_theory lra_th; // the linear-real-arithmetic theory..
    smt::ov_theory ov_th;   // the object-variable theory..
    smt::idl_theory idl_th; // the integer difference logic theory..
    smt::rdl_theory rdl_th; // the real difference logic theory..

    std::vector<riddle::ast::compilation_unit *> cus; // the compilation units..

    std::map<std::string, std::vector<const method *>> methods; // the methods, indexed by their name, defined within this type..
    std::map<std::string, type *> types;                        // the types, indexed by their name, defined within this core..
    std::map<std::string, predicate *> predicates;              // the predicates, indexed by their name, defined within this core..

    smt::lit tmp_ni;             // the temporary controlling literal, used for restoring the controlling literal..
    smt::lit ni = smt::TRUE_var; // the controlling literal..

#ifdef BUILD_GUI
  private:
    std::vector<core_listener *> listeners; // the core listeners..

  protected:
    void fire_log(const std::string msg) const noexcept;
    void fire_read(const std::string &script) const noexcept;
    void fire_read(const std::vector<std::string> &files) const noexcept;
    void fire_state_changed() const noexcept;
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