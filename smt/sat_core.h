#pragma once

#include "lit.h"
#include <vector>
#include <queue>
#include <string>
#include <unordered_map>
#include <set>

namespace smt
{
  class constr;
  class theory;
  class sat_value_listener;

  class sat_core
  {
    friend class constr;
    friend class theory;
    friend class sat_value_listener;

  public:
    sat_core();
    sat_core(const sat_core &orig) = delete;
    ~sat_core();

    var new_var() noexcept;                                 // creates a new propositional variable..
    bool new_clause(const std::vector<lit> &lits) noexcept; // creates a new clause given the 'lits' literals returning 'false' if some trivial inconsistency is detected..

    lit new_eq(const lit &left, const lit &right) noexcept;   // creates a new reified equality..
    lit new_conj(const std::vector<lit> &ls) noexcept;        // creates a new reified conjunction..
    lit new_disj(const std::vector<lit> &ls) noexcept;        // creates a new reified disjunction..
    lit new_at_most_one(const std::vector<lit> &ls) noexcept; // creates a new reified at-most-one..
    lit new_exct_one(const std::vector<lit> &ls) noexcept;    // creates a new reified exct-one..

    bool assume(const lit &p) noexcept;
    void pop() noexcept;
    bool simplify_db() noexcept;
    bool propagate() noexcept;
    bool check(const std::vector<lit> &lits) noexcept;

    lbool value(const var &x) const noexcept { return assigns.at(x); } // returns the value of variable 'x'..
    lbool value(const lit &p) const noexcept
    {
      switch (value(variable(p)))
      {
      case True:
        return sign(p) ? True : False;
      case False:
        return sign(p) ? False : True;
      default:
        return Undefined;
      }
    }                                                                   // returns the value of literal 'p'..
    size_t decision_level() const noexcept { return trail_lim.size(); } // returns the current decision level..
    bool root_level() const noexcept { return trail_lim.empty(); }      // checks whether the current decision level is root level..

  private:
    static std::string to_string(const lit &p) noexcept { return (sign(p) ? "b" : "!b") + std::to_string(variable(p)); }

    void analyze(constr &cnfl, std::vector<lit> &out_learnt, size_t &out_btlevel) noexcept;
    void record(const std::vector<lit> &lits) noexcept;

    bool enqueue(const lit &p, constr *const c = nullptr) noexcept;
    void pop_one() noexcept;

    void add_theory(theory &th) noexcept { theories.push_back(&th); }
    void bind(const var &v, theory &th) noexcept { bounds[v].insert(&th); }
    void listen(const var &v, sat_value_listener &l) noexcept { listening[v].insert(&l); }

  private:
    std::vector<constr *> constrs;              // the collection of problem constraints..
    std::vector<std::vector<constr *>> watches; // for each literal 'p', a list of constraints watching 'p'..
    std::vector<lbool> assigns;                 // the current assignments..

    std::queue<lit> prop_q;                     // propagation queue..
    std::vector<lit> trail;                     // the list of assignment in chronological order..
    std::vector<size_t> trail_lim;              // separator indices for different decision levels in 'trail'..
    std::vector<constr *> reason;               // for each variable, the constraint that implied its value..
    std::vector<size_t> level;                  // for each variable, the decision level it was assigned..
    std::unordered_map<std::string, lit> exprs; // the already existing expressions (string to literal)..

    std::vector<theory *> theories; // all the theories..
    std::unordered_map<size_t, std::set<theory *>> bounds;
    std::unordered_map<size_t, std::set<sat_value_listener *>> listening;
  };

} // namespace smt