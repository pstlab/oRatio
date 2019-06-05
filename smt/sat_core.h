#pragma once

#include "lit.h"
#include "lbool.h"
#include "theory.h"
#include <vector>
#include <queue>
#include <string>
#include <unordered_map>
#include <set>

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
  ~sat_core();

  var new_var();                                 // creates a new propositional variable..
  bool new_clause(const std::vector<lit> &lits); // creates a new clause given the 'lits' literals returning 'false' if some trivial inconsistency is detected..

  var new_eq(const lit &left, const lit &right); // creates a new reified equality..
  var new_conj(const std::vector<lit> &ls);      // creates a new reified conjunction..
  var new_disj(const std::vector<lit> &ls);      // creates a new reified disjunction..
  var new_exct_one(const std::vector<lit> &ls);  // creates a new reified exct-one..

  bool eq(const lit &left, const lit &right, const var &p = TRUE_var); // creates a new reified equality controlled by the 'p' literal..
  bool conj(const std::vector<lit> &ls, const var &p = TRUE_var);      // creates a new reified conjunction controlled by the 'p' literal..
  bool disj(const std::vector<lit> &ls, const var &p = TRUE_var);      // creates a new reified disjunction controlled by the 'p' literal..
  bool exct_one(const std::vector<lit> &ls, const var &p = TRUE_var);  // creates a new reified exct-one controlled by the 'p' literal..

  bool assume(const lit &p);
  void pop();
  bool check();
  bool check(const std::vector<lit> &lits);

  lbool value(const var &x) const { return assigns.at(x); } // returns the value of variable 'x'..
  lbool value(const lit &p) const
  {
    switch (value(p.get_var()))
    {
    case True:
      return p.get_sign() ? True : False;
    case False:
      return p.get_sign() ? False : True;
    default:
      return Undefined;
    }
  }                                                          // returns the value of literal 'p'..
  size_t decision_level() const { return trail_lim.size(); } // returns the current decision level..
  bool root_level() const { return trail_lim.empty(); }      // checks whether this decision level is root level..

private:
  static size_t index(const lit &p) { return p.get_sign() ? p.get_var() << 1 : (p.get_var() << 1) ^ 1; }

  bool propagate(std::vector<lit> &cnfl);
  void analyze(std::vector<lit> &cnfl, std::vector<lit> &out_learnt, size_t &out_btlevel);
  void record(const std::vector<lit> &lits);

  bool enqueue(const lit &p, clause *const c = nullptr);
  void pop_one();

  void add_theory(theory &th) { theories.push_back(&th); }
  void bind(const var &v, theory &th) { bounds[v].insert(&th); }
  void listen(const var &v, sat_value_listener &l) { listening[v].insert(&l); }

private:
  std::vector<clause *> constrs;              // the collection of problem constraints..
  std::vector<std::vector<clause *>> watches; // for each literal 'p', a list of constraints watching 'p'..
  std::queue<lit> prop_q;                     // propagation queue..
  std::vector<lbool> assigns;                 // the current assignments..
  std::vector<lit> trail;                     // the list of assignment in chronological order..
  std::vector<size_t> trail_lim;              // separator indices for different decision levels in 'trail'..
  std::vector<clause *> reason;               // for each variable, the constraint that implied its value..
  std::vector<size_t> level;                  // for each variable, the decision level it was assigned..
  std::unordered_map<std::string, var> exprs; // the already existing expressions (string to bool variable)..

  std::vector<theory *> theories; // all the theories..
  std::unordered_map<var, std::set<theory *>> bounds;
  std::unordered_map<var, std::set<sat_value_listener *>> listening;
};

} // namespace smt