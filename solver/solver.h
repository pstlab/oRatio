#pragma once

#include "core.h"
#include "graph.h"

namespace ratio
{

class smart_type;
#ifdef BUILD_GUI
class solver_listener;
#endif

class solver : public core, private smt::theory
{
  friend class graph;
  friend class smart_type;
#ifdef BUILD_GUI
  friend class solver_listener;
#endif
public:
  solver();
  solver(const solver &orig) = delete;
  ~solver();

  /**
   * Initializes the solver.
   */
  void init();

  /**
   * Solves the given problem.
   */
  void solve() override;

  expr new_enum(const type &tp, const std::unordered_set<item *> &allowed_vals) override;
  atom_flaw &get_reason(const atom &atm) const { return *reason.at(&atm); } // returns the flaw which has given rise to the atom..

private:
  void new_fact(atom &atm) override;                                      // creates a new fact token..
  void new_goal(atom &atm) override;                                      // creates a new goal token..
  void new_disjunction(context &d_ctx, const disjunction &disj) override; // creates a new disjunction..

  void take_decision(const smt::lit &ch);
  void next();

  bool propagate(const smt::lit &p, std::vector<smt::lit> &cnfl) override;
  bool check(std::vector<smt::lit> &cnfl) override;
  void push() override;
  void pop() override;

  flaw *select_flaw();          // selects the most promising (i.e. the most expensive one) flaw from the 'flaws' set, returns a nullptr if there are no more active flaws..
  void solve_inconsistencies(); // checks whether the types have any inconsistency and, in case, solve them..

private:
  struct layer
  {

    layer(const smt::lit &dec) : decision(dec) {}

    const smt::lit decision;                                   // the decision which introduced the new layer..
    std::unordered_map<resolver *, smt::rational> old_r_costs; // the old estimated resolvers' costs..
    std::unordered_map<flaw *, smt::rational> old_f_costs;     // the old estimated flaws' costs..
    std::unordered_set<flaw *> new_flaws;                      // the just activated flaws..
    std::unordered_set<flaw *> solved_flaws;                   // the just solved flaws..
  };
  graph gr;                                             // the causal graph..
  std::unordered_map<const atom *, atom_flaw *> reason; // the reason for having introduced an atom..
  std::unordered_set<flaw *> flaws;                     // the currently active flaws..
  std::vector<layer> trail;                             // the list of applied resolvers, with the associated changes made, in chronological order..

#ifdef BUILD_GUI
private:
  std::vector<solver_listener *> listeners; // the solver listeners..

  void fire_new_flaw(const flaw &f) const;
  void fire_flaw_state_changed(const flaw &f) const;
  void fire_flaw_cost_changed(const flaw &f) const;
  void fire_current_flaw(const flaw &f) const;
  void fire_new_resolver(const resolver &r) const;
  void fire_resolver_state_changed(const resolver &r) const;
  void fire_resolver_cost_changed(const resolver &r) const;
  void fire_current_resolver(const resolver &r) const;
  void fire_causal_link_added(const flaw &f, const resolver &r) const;
#endif
};
} // namespace ratio
