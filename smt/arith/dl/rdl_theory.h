#pragma once

#include "sat_core.h"
#include "theory.h"
#include "inf_rational.h"
#include "lin.h"
#include <map>

namespace smt
{
  class rdl_value_listener;

  class rdl_theory : public theory
  {
    friend class rdl_value_listener;

  public:
    SMT_EXPORT rdl_theory(sat_core &sat, const size_t &size = 16);
    rdl_theory(const rdl_theory &orig) = delete;
    SMT_EXPORT virtual ~rdl_theory();

    SMT_EXPORT var new_var() noexcept; // creates and returns a new distance logic variable..

    SMT_EXPORT lit new_distance(const var &from, const var &to, const inf_rational &dist) noexcept; // creates and returns a new propositional variable for controlling the constraint 'to - from <= dist'..
    SMT_EXPORT lit new_distance(const var &from, const var &to, const inf_rational &min, const inf_rational &max) noexcept { return sat.new_conj({new_distance(to, from, -min), new_distance(from, to, max)}); }

    SMT_EXPORT lit new_lt(const lin &left, const lin &right);
    SMT_EXPORT lit new_leq(const lin &left, const lin &right);
    SMT_EXPORT lit new_eq(const lin &left, const lin &right);
    SMT_EXPORT lit new_geq(const lin &left, const lin &right);
    SMT_EXPORT lit new_gt(const lin &left, const lin &right);

    SMT_EXPORT inline inf_rational lb(const var &v) const noexcept { return -_dists[v][0]; }
    SMT_EXPORT inline inf_rational ub(const var &v) const noexcept { return _dists[0][v]; }
    SMT_EXPORT inline std::pair<inf_rational, inf_rational> bounds(const var &v) const noexcept { return std::make_pair(-_dists[v][0], _dists[0][v]); }
    SMT_EXPORT inline std::pair<inf_rational, inf_rational> distance(const var &from, const var &to) const noexcept { return std::make_pair(-_dists[to][from], _dists[from][to]); }

    SMT_EXPORT std::pair<inf_rational, inf_rational> bounds(const lin &l) const;
    SMT_EXPORT std::pair<inf_rational, inf_rational> distance(const lin &from, const lin &to) const;

    SMT_EXPORT bool equates(const lin &l0, const lin &l1) const;

    size_t size() const noexcept { return n_vars; }

  private:
    bool propagate(const lit &p) noexcept override;
    bool check() noexcept override;
    void push() noexcept override;
    void pop() noexcept override;

    void propagate(const var &from, const var &to, const inf_rational &dist) noexcept;
    void set_dist(const var &from, const var &to, const inf_rational &dist) noexcept;
    void set_pred(const var &from, const var &to, const var &pred) noexcept;

    void resize(const size_t &size) noexcept;

    inline void listen(const var &v, rdl_value_listener *const l) noexcept { listening[v].insert(l); }

  private:
    class rdl_distance
    {
      friend class rdl_theory;

    public:
      rdl_distance(const lit &b, const var &from, const var &to, const inf_rational &dist) : b(b), from(from), to(to), dist(dist) {}
      rdl_distance(const rdl_distance &orig) = delete;
      ~rdl_distance() {}

    private:
      const lit b; // the propositional literal associated to the distance constraint..
      const var from;
      const var to;
      const inf_rational dist;
    };

    struct layer
    {
      std::map<std::pair<var, var>, inf_rational> old_dists;     // the updated distances..
      std::map<std::pair<var, var>, var> old_preds;              // the updated predecessors..
      std::map<std::pair<var, var>, rdl_distance *> old_constrs; // the updated constraints..
    };

    size_t n_vars = 1;
    std::vector<std::vector<inf_rational>> _dists;                           // the distance matrix..
    std::vector<std::vector<var>> _preds;                                    // the predecessor matrix..
    std::map<std::pair<var, var>, rdl_distance *> dist_constr;               // the currently enforced constraints..
    std::unordered_map<var, std::vector<rdl_distance *>> var_dists;          // the constraints controlled by a propositional variable (for propagation purposes)..
    std::map<std::pair<var, var>, std::vector<rdl_distance *>> dist_constrs; // the constraints between two temporal points (for propagation purposes)..
    std::vector<layer> layers;                                               // we store the updates..
    std::unordered_map<var, std::set<rdl_value_listener *>> listening;
  };
} // namespace smt