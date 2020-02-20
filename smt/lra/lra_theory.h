#pragma once

#include "sat_core.h"
#include "lin.h"
#include "inf_rational.h"
#include <set>
#include <unordered_set>
#include <unordered_map>

namespace smt
{

class lra_value_listener;
class assertion;
class row;

class lra_theory : public theory
{
  friend class lra_value_listener;
  friend class assertion;
  friend class row;

public:
  lra_theory(sat_core &sat);
  lra_theory(const lra_theory &orig) = delete;
  virtual ~lra_theory();

  var new_var();             // creates and returns a new numeric variable..
  var new_var(const lin &l); // creates and returns a new numeric variable and makes it equal to the given linear expression..

  bool is_basic(const var &v) const { return tableau.count(v); }

  var new_lt(const lin &left, const lin &right);
  var new_leq(const lin &left, const lin &right);
  var new_geq(const lin &left, const lin &right);
  var new_gt(const lin &left, const lin &right);

  bool lt(const lin &left, const lin &right, const var &p = TRUE_var);
  bool leq(const lin &left, const lin &right, const var &p = TRUE_var);
  bool geq(const lin &left, const lin &right, const var &p = TRUE_var);
  bool gt(const lin &left, const lin &right, const var &p = TRUE_var);

  inf_rational lb(const var &v) const { return bounds[lb_index(v)].value; } // the current lower bound of variable 'v'..
  inf_rational ub(const var &v) const { return bounds[ub_index(v)].value; } // the current upper bound of variable 'v'..
  inf_rational value(const var &v) const { return vals[v]; }                // the current value of variable 'v'..

  inf_rational lb(const lin &l) const // returns the current lower bound of linear expression 'l'..
  {
    inf_rational b(l.known_term);
    for (const auto &term : l.vars)
      b += (term.second.is_positive() ? lb(term.first) : ub(term.first)) * term.second;
    return b;
  }
  inf_rational ub(const lin &l) const // returns the current upper bound of linear expression 'l'..
  {
    inf_rational b(l.known_term);
    for (const auto &term : l.vars)
      b += (term.second.is_positive() ? ub(term.first) : lb(term.first)) * term.second;
    return b;
  }
  inf_rational value(const lin &l) const // returns the current value of linear expression 'l'..
  {
    inf_rational v(l.known_term);
    for (const auto &term : l.vars)
      v += value(term.first) * term.second;
    return v;
  }

private:
  bool propagate(const lit &p, std::vector<lit> &cnfl) override;
  bool check(std::vector<lit> &cnfl) override;
  void push() override;
  void pop() override;

  bool assert_lower(const var &x_i, const inf_rational &val, const lit &p, std::vector<lit> &cnfl);
  bool assert_upper(const var &x_i, const inf_rational &val, const lit &p, std::vector<lit> &cnfl);
  void update(const var &x_i, const inf_rational &v);
  void pivot_and_update(const var &x_i, const var &x_j, const inf_rational &v);
  void pivot(const var x_i, const var x_j);
  void new_row(const var &x, const lin &l);

  void listen(const var &v, lra_value_listener *const l) { listening[v].insert(l); }

  static size_t lb_index(const var &v) { return v << 1; }       // the index of the lower bound of the 'v' variable..
  static size_t ub_index(const var &v) { return (v << 1) ^ 1; } // the index of the upper bound of the 'v' variable..

public:
  std::string to_string();

private:
  /**
     * Represents the bound of a variable and the reason for its existence.
     */
  struct bound
  {
    inf_rational value; // the value of the bound..
    lit *reason;        // the reason for the value..
  };

  std::vector<bound> bounds;                                 // the current bounds..
  std::vector<inf_rational> vals;                            // the current values..
  std::map<const var, row *> tableau;                        // the sparse matrix..
  std::unordered_map<std::string, var> exprs;                // the expressions (string to numeric variable) for which already exist slack variables..
  std::unordered_map<std::string, var> s_asrts;              // the assertions (string to propositional variable) used for reducing the number of boolean variables..
  std::unordered_map<var, std::vector<assertion *>> v_asrts; // the assertions (propositional variable to assertions) used for enforcing (negating) assertions..
  std::vector<std::vector<assertion *>> a_watches;           // for each variable 'v', a list of assertions watching 'v'..
  std::vector<std::unordered_set<row *>> t_watches;          // for each variable 'v', a list of tableau rows watching 'v'..
  std::vector<std::unordered_map<size_t, bound>> layers;     // we store the updated bounds..
  std::unordered_map<var, std::set<lra_value_listener *>> listening;

#ifdef OPTIMIZE_FOR_NATIVE_ARCH
  std::vector<mutex_wrapper> t_mutexes;
#endif
};
} // namespace smt