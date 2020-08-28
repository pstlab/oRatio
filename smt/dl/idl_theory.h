#pragma once

#include "sat_core.h"
#include "theory.h"
#include "lin.h"
#include <limits>
#include <map>

namespace smt
{

  class idl_value_listener;

  class idl_theory : public theory
  {
    friend class idl_value_listener;

  public:
    idl_theory(sat_core &sat, const size_t &size = 16);
    idl_theory(const idl_theory &orig) = delete;
    virtual ~idl_theory();

    var new_var(); // creates and returns a new distance logic variable..

    lit new_distance(const var &from, const var &to, const I &dist); // creates and returns a new propositional variable for controlling the constraint 'to - from <= dist'..
    lit new_distance(const var &from, const var &to, const I &min, const I &max) { return sat.new_conj({new_distance(to, from, -min), new_distance(from, to, max)}); }

    lit new_lt(const lin &left, const lin &right);
    lit new_leq(const lin &left, const lin &right);
    lit new_eq(const lin &left, const lin &right);
    lit new_geq(const lin &left, const lin &right);
    lit new_gt(const lin &left, const lin &right);

    I lb(const var &v) const { return -_dists[v][0]; }
    I ub(const var &v) const { return _dists[0][v]; }
    std::pair<I, I> bounds(const var &v) const { return std::make_pair(-_dists[v][0], _dists[0][v]); }
    std::pair<I, I> distance(const var &from, const var &to) const { return std::make_pair(-_dists[to][from], _dists[from][to]); }

    std::pair<I, I> bounds(const lin &l) const;
    std::pair<I, I> distance(const lin &from, const lin &to) const;

    bool equates(const lin &l0, const lin &l1) const;

    size_t size() const { return n_vars; }

  public:
    static constexpr I inf() { return std::numeric_limits<I>::max() / 2 - 1; }

  private:
    bool propagate(const lit &p) override;
    bool check() override;
    void push() override;
    void pop() override;

    void propagate(const var &from, const var &to, const I &dist);
    void set_dist(const var &from, const var &to, const I &dist);
    void set_pred(const var &from, const var &to, const var &pred);

    void resize(const size_t &size);

    void listen(const var &v, idl_value_listener *const l) { listening[v].insert(l); }

  private:
    class idl_distance
    {
      friend class idl_theory;

    public:
      idl_distance(const var &b, const var &from, const var &to, const I &dist) : b(b), from(from), to(to), dist(dist) {}
      idl_distance(const idl_distance &orig) = delete;
      ~idl_distance() {}

    private:
      const var b; // the propositional variable associated to the distance constraint..
      const var from;
      const var to;
      const I dist;
    };

    struct layer
    {
      std::map<std::pair<var, var>, I> old_dists;                // the updated distances..
      std::map<std::pair<var, var>, var> old_preds;              // the updated predecessors..
      std::map<std::pair<var, var>, idl_distance *> old_constrs; // the updated constraints..
    };

    size_t n_vars = 1;
    std::vector<std::vector<I>> _dists;                                      // the distance matrix..
    std::vector<std::vector<var>> _preds;                                    // the predecessor matrix..
    std::map<std::pair<var, var>, idl_distance *> dist_constr;               // the currently enforced constraints..
    std::unordered_map<var, std::vector<idl_distance *>> var_dists;          // the constraints controlled by a propositional variable (for propagation purposes)..
    std::map<std::pair<var, var>, std::vector<idl_distance *>> dist_constrs; // the constraints between two temporal points (for propagation purposes)..
    std::vector<layer> layers;                                               // we store the updates..
    std::unordered_map<var, std::set<idl_value_listener *>> listening;
  };
} // namespace smt