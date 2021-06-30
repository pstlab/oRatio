#pragma once

#include "sat_core.h"
#include "theory.h"
#include "lin.h"
#include "inf_rational.h"
#include "json.h"
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <mutex>

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
    SMT_EXPORT lra_theory(sat_core &sat);
    lra_theory(const lra_theory &orig) = delete;
    SMT_EXPORT virtual ~lra_theory();

    SMT_EXPORT var new_var() noexcept;             // creates and returns a new numeric variable..
    SMT_EXPORT var new_var(const lin &l) noexcept; // creates and returns a new numeric variable and makes it equal to the given linear expression..

    inline bool is_basic(const var &v) const noexcept { return tableau.count(v); }

    SMT_EXPORT lit new_lt(const lin &left, const lin &right) noexcept;
    SMT_EXPORT lit new_leq(const lin &left, const lin &right) noexcept;
    SMT_EXPORT lit new_eq(const lin &left, const lin &right) noexcept { return sat.new_conj({new_geq(left, right), new_leq(left, right)}); }
    SMT_EXPORT lit new_geq(const lin &left, const lin &right) noexcept;
    SMT_EXPORT lit new_gt(const lin &left, const lin &right) noexcept;

    inline inf_rational lb(const var &v) const noexcept { return c_bounds[lb_index(v)].value; } // the current lower bound of variable 'v'..
    inline inf_rational ub(const var &v) const noexcept { return c_bounds[ub_index(v)].value; } // the current upper bound of variable 'v'..
    inline inf_rational value(const var &v) const noexcept { return vals[v]; }                  // the current value of variable 'v'..

    inline inf_rational lb(const lin &l) const noexcept // returns the current lower bound of linear expression 'l'..
    {
      inf_rational b(l.known_term);
      for (const auto &[v, c] : l.vars)
        b += (is_positive(c) ? lb(v) : ub(v)) * c;
      return b;
    }
    inline inf_rational ub(const lin &l) const noexcept // returns the current upper bound of linear expression 'l'..
    {
      inf_rational b(l.known_term);
      for (const auto &[v, c] : l.vars)
        b += (is_positive(c) ? ub(v) : lb(v)) * c;
      return b;
    }
    inline std::pair<inf_rational, inf_rational> bounds(const lin &l) const noexcept // returns the current upper bound of linear expression 'l'..
    {
      inf_rational c_lb(l.known_term);
      inf_rational c_ub(l.known_term);
      for (const auto &[v, c] : l.vars)
      {
        c_lb += (is_positive(c) ? lb(v) : ub(v)) * c;
        c_ub += (is_positive(c) ? ub(v) : lb(v)) * c;
      }
      return std::make_pair(c_lb, c_ub);
    }
    inline inf_rational value(const lin &l) const // returns the current value of linear expression 'l'..
    {
      inf_rational val(l.known_term);
      for (const auto &[v, c] : l.vars)
        val += value(v) * c;
      return val;
    }

    SMT_EXPORT bool equates(const lin &l0, const lin &l1) const noexcept;

    SMT_EXPORT bool set_lb(const var &x_i, const inf_rational &val, const lit &p) noexcept { return assert_lower(x_i, val, p); }
    SMT_EXPORT bool set_ub(const var &x_i, const inf_rational &val, const lit &p) noexcept { return assert_upper(x_i, val, p); }
    SMT_EXPORT bool set(const var &x_i, const inf_rational &val, const lit &p) noexcept { return set_lb(x_i, val, p) && set_ub(x_i, val, p); }

  private:
    bool propagate(const lit &p) noexcept override;
    bool check() noexcept override;
    void push() noexcept override;
    void pop() noexcept override;

    bool assert_lower(const var &x_i, const inf_rational &val, const lit &p) noexcept;
    bool assert_upper(const var &x_i, const inf_rational &val, const lit &p) noexcept;
    void update(const var &x_i, const inf_rational &v) noexcept;
    void pivot_and_update(const var &x_i, const var &x_j, const inf_rational &v) noexcept;
    void pivot(const var x_i, const var x_j) noexcept;
    void new_row(const var &x, const lin &l) noexcept;

    inline void listen(const var &v, lra_value_listener *const l) noexcept { listening[v].insert(l); }

    inline static size_t lb_index(const var &v) noexcept { return v << 1; }       // the index of the lower bound of the 'v' variable..
    inline static size_t ub_index(const var &v) noexcept { return (v << 1) ^ 1; } // the index of the upper bound of the 'v' variable..

    json to_json() const noexcept;

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

#ifdef PARALLELIZE
    struct var_mtx : std::mutex
    {
      var_mtx() = default;
      var_mtx(var_mtx const &) noexcept : std::mutex() {}
      bool operator==(var_mtx const &other) noexcept { return this == &other; }
    };
    std::vector<var_mtx> t_mtxs; // for each variable 'v', a list of mutexes parallelizing pivoting..
#endif
  };
} // namespace smt