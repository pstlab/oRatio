#pragma once

#include "sat_core.h"
#include "theory.h"
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

    lit new_lt(const lin &left, const lin &right);
    lit new_leq(const lin &left, const lin &right);
    lit new_eq(const lin &left, const lin &right) { return sat.new_conj({new_geq(left, right), new_leq(left, right)}); }
    lit new_geq(const lin &left, const lin &right);
    lit new_gt(const lin &left, const lin &right);

    inf_rational lb(const var &v) const { return c_bounds[lb_index(v)].value; } // the current lower bound of variable 'v'..
    inf_rational ub(const var &v) const { return c_bounds[ub_index(v)].value; } // the current upper bound of variable 'v'..
    inf_rational value(const var &v) const { return vals[v]; }                  // the current value of variable 'v'..

    inf_rational lb(const lin &l) const // returns the current lower bound of linear expression 'l'..
    {
      inf_rational b(l.known_term);
      for (const auto &term : l.vars)
        b += (is_positive(term.second) ? lb(term.first) : ub(term.first)) * term.second;
      return b;
    }
    inf_rational ub(const lin &l) const // returns the current upper bound of linear expression 'l'..
    {
      inf_rational b(l.known_term);
      for (const auto &term : l.vars)
        b += (is_positive(term.second) ? ub(term.first) : lb(term.first)) * term.second;
      return b;
    }
    std::pair<inf_rational, inf_rational> bounds(const lin &l) const // returns the current upper bound of linear expression 'l'..
    {
      inf_rational c_lb(l.known_term);
      inf_rational c_ub(l.known_term);
      for (const auto &term : l.vars)
      {
        c_lb += (is_positive(term.second) ? lb(term.first) : ub(term.first)) * term.second;
        c_ub += (is_positive(term.second) ? ub(term.first) : lb(term.first)) * term.second;
      }
      return std::make_pair(c_lb, c_ub);
    }
    inf_rational value(const lin &l) const // returns the current value of linear expression 'l'..
    {
      inf_rational v(l.known_term);
      for (const auto &term : l.vars)
        v += value(term.first) * term.second;
      return v;
    }

    bool equates(const lin &l0, const lin &l1) const;

  private:
    bool propagate(const lit &p) override;
    bool check() override;
    void push() override;
    void pop() override;

    bool assert_lower(const var &x_i, const inf_rational &val, const lit &p);
    bool assert_upper(const var &x_i, const inf_rational &val, const lit &p);
    void update(const var &x_i, const inf_rational &v);
    void pivot_and_update(const var &x_i, const var &x_j, const inf_rational &v);
    void pivot(const var x_i, const var x_j);
    void new_row(const var &x, const lin &l);

    void listen(const var &v, lra_value_listener *const l) { listening[v].insert(l); }

    static size_t lb_index(const var &v) { return v << 1; }       // the index of the lower bound of the 'v' variable..
    static size_t ub_index(const var &v) { return (v << 1) ^ 1; } // the index of the upper bound of the 'v' variable..

    friend std::string to_string(const lra_theory &rhs);

  private:
    /**
     * Represents the bound of a variable and the reason for its existence.
     */
    struct bound
    {
      inf_rational value; // the value of the bound..
      lit reason;         // the reason for the value..
    };

    std::vector<bound> c_bounds;                               // the current bounds..
    std::vector<inf_rational> vals;                            // the current values..
    std::map<const var, row *> tableau;                        // the sparse matrix..
    std::unordered_map<std::string, var> exprs;                // the expressions (string to numeric variable) for which already exist slack variables..
    std::unordered_map<std::string, lit> s_asrts;              // the assertions (string to literal) used for reducing the number of boolean variables..
    std::unordered_map<var, std::vector<assertion *>> v_asrts; // the assertions (literal to assertions) used for enforcing (negating) assertions..
    std::vector<std::vector<assertion *>> a_watches;           // for each variable 'v', a list of assertions watching 'v'..
    std::vector<std::unordered_set<row *>> t_watches;          // for each variable 'v', a list of tableau rows watching 'v'..
    std::vector<std::unordered_map<size_t, bound>> layers;     // we store the updated bounds..
    std::unordered_map<var, std::set<lra_value_listener *>> listening;
  };
} // namespace smt