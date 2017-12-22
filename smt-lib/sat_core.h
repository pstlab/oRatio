#pragma once

#include "lit.h"
#include "lbool.h"
#include <vector>
#include <set>
#include <queue>
#include <string>
#include <unordered_map>

namespace smt
{
static const var FALSE_var = 0;
static const var TRUE_var = 1;

class clause;
class theory;
class sat_value_listener;

class sat_core
{
  friend class clause;
  friend class theory;
  friend class sat_value_listener;

public:
  sat_core();
  sat_core(const sat_core &orig) = delete;
  virtual ~sat_core();

  const var new_var();                           // creates a new propositional variable..
  bool new_clause(const std::vector<lit> &lits); // creates a new clause given the 'lits' literals..

  const var new_eq(const lit &left, const lit &right); // creates a new reified equality..
  const var new_conj(const std::vector<lit> &ls);      // creates a new reified conjunction..
  const var new_disj(const std::vector<lit> &ls);      // creates a new reified disjunction..
  const var new_exct_one(const std::vector<lit> &ls);  // creates a new reified exct-one..

  bool eq(const lit &left, const lit &right) { return new_clause({!left, right}) && new_clause({left, !right}); }
  bool exct_one(const std::vector<lit> &lits)
  {
    // the at-least-one clause..
    std::vector<lit> ls;
    for (size_t i = 0; i < lits.size(); i++)
    {
      for (size_t j = i + 1; j < lits.size(); j++)
        // the at-most-one clauses..
        if (!new_clause({!lits.at(i), !lits.at(j)}))
          return false;
      ls.push_back(lits.at(i));
    }
    return new_clause(ls);
  }

  bool assume(const lit &p);
  void pop();
  bool check();
  bool check(const std::vector<lit> &lits);

  lbool value(const var &x) const { return assigns[x]; } // returns the value of variable 'v'..
  lbool value(const lit &p) const
  {
    switch (value(p.v))
    {
    case True:
      return p.sign ? True : False;
    case False:
      return p.sign ? False : True;
    default:
      return Undefined;
    }
  }
  size_t decision_level() { return trail_lim.size(); } // returns the current decision level..
  bool root_level() { return trail_lim.empty(); }      // checks whether this decision level is root level..

private:
  static size_t index(const lit &p) { return p.sign ? p.v << 1 : (p.v << 1) ^ 1; }

  bool propagate(std::vector<lit> &cnfl);
  void analyze(const std::vector<lit> &cnfl, std::vector<lit> &out_learnt, size_t &out_btlevel);
  void record(const std::vector<lit> &lits);

  bool enqueue(const lit &p, clause *const c = nullptr);
  void pop_one();

  void add_theory(theory &th) { theories.push_back(&th); }
  void bind(const var &v, theory &th) { bounds[v].push_back(&th); }
  void listen(const var &v, sat_value_listener *const l) { listening[v].insert(l); }

private:
  std::vector<clause *> constrs;              // collection of problem constraints..
  std::vector<std::vector<clause *>> watches; // for each literal 'p', a list of constraints watching 'p'..
  std::queue<lit> prop_q;                     // propagation queue..
  std::vector<lbool> assigns;                 // the current assignments..
  std::vector<lit> trail;                     // the list of assignment in chronological order..
  std::vector<size_t> trail_lim;              // separator indices for different decision levels in 'trail'..
  std::vector<clause *> reason;               // for each variable, the constraint that implied its value..
  std::vector<size_t> level;                  // for each variable, the decision level it was assigned..
  std::unordered_map<std::string, var> exprs; // the already existing expressions (string to bool variable)..

  std::vector<theory *> theories; // all the theories..
  std::unordered_map<var, std::vector<theory *>> bounds;
  std::unordered_map<var, std::set<sat_value_listener *>> listening;
};
}