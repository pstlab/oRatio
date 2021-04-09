#pragma once

#include "smt_export.h"
#include "lit.h"
#ifdef PARALLELIZE
#include "thread_pool.h"
#endif
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
    SMT_EXPORT sat_core();
    sat_core(const sat_core &orig) = delete;
    SMT_EXPORT ~sat_core();

#ifdef PARALLELIZE
    thread_pool &get_thread_pool()
    {
      return th_pool;
    }
#endif

    SMT_EXPORT var new_var() noexcept;                                 // creates a new propositional variable..
    SMT_EXPORT bool new_clause(const std::vector<lit> &lits) noexcept; // creates a new clause given the 'lits' literals returning 'false' if some trivial inconsistency is detected..

    SMT_EXPORT lit new_eq(const lit &left, const lit &right) noexcept;   // creates a new reified equality..
    SMT_EXPORT lit new_conj(const std::vector<lit> &ls) noexcept;        // creates a new reified conjunction..
    SMT_EXPORT lit new_disj(const std::vector<lit> &ls) noexcept;        // creates a new reified disjunction..
    SMT_EXPORT lit new_at_most_one(const std::vector<lit> &ls) noexcept; // creates a new reified at-most-one..
    SMT_EXPORT lit new_exct_one(const std::vector<lit> &ls) noexcept;    // creates a new reified exct-one..

    SMT_EXPORT bool assume(const lit &p) noexcept;
    SMT_EXPORT void pop() noexcept;
    SMT_EXPORT bool simplify_db() noexcept;
    SMT_EXPORT bool propagate() noexcept;
    SMT_EXPORT bool check(const std::vector<lit> &lits) noexcept;

    inline lbool value(const var &x) const noexcept { return assigns.at(x); } // returns the value of variable 'x'..
    inline lbool value(const lit &p) const noexcept
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
    }                                                                          // returns the value of literal 'p'..
    inline size_t decision_level() const noexcept { return trail_lim.size(); } // returns the current decision level..
    inline bool root_level() const noexcept { return trail_lim.empty(); }      // checks whether the current decision level is root level..

  private:
    void analyze(constr &cnfl, std::vector<lit> &out_learnt, size_t &out_btlevel) noexcept;
    void record(const std::vector<lit> &lits) noexcept;

    bool enqueue(const lit &p, constr *const c = nullptr) noexcept;
    void pop_one() noexcept;

    inline void add_theory(theory &th) noexcept { theories.push_back(&th); }
    inline void bind(const var &v, theory &th) noexcept { bounds[v].insert(&th); }
    inline void listen(const var &v, sat_value_listener &l) noexcept { listening[v].insert(&l); }

  private:
#ifdef PARALLELIZE
    thread_pool th_pool;
#endif

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