#pragma once

#include "core.h"
#include "graph.h"

#define START "start"
#define END "end"

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

  graph &get_graph() { return gr; }

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

  size_t decision_level() const { return trail.size(); } // returns the current decision level..
  bool root_level() const { return trail.empty(); }      // checks whether the current decision level is root level..
  void take_decision(const smt::lit &ch);
  void next();

  bool propagate(const smt::lit &p, std::vector<smt::lit> &cnfl) override;
  bool check(std::vector<smt::lit> &cnfl) override;
  void push() override;
  void pop() override;

  void solve_inconsistencies();                                     // checks whether the types have any inconsistency and, in case, solve them..
  std::vector<std::vector<std::pair<smt::lit, double>>> get_incs(); // collects all the current inconsistencies..

private:
  struct layer
  {

    layer(const smt::lit &dec) : decision(dec) {}

    const smt::lit decision;                               // the decision which introduced the new layer..
    std::unordered_map<flaw *, smt::rational> old_f_costs; // the old estimated flaws' costs..
    std::unordered_set<flaw *> new_flaws;                  // the just activated flaws..
    std::unordered_set<flaw *> solved_flaws;               // the just solved flaws..
  };
  graph gr;                                             // the causal graph..
  std::vector<smart_type *> sts;                        // the smart-types..
  std::unordered_map<const atom *, atom_flaw *> reason; // the reason for having introduced an atom..
  std::unordered_set<flaw *> flaws;                     // the currently active flaws..
  smt::lit current_decision;                            // the current decision which has just been taken..
  std::vector<layer> trail;                             // the list of taken decisions, with the associated changes made, in chronological order..

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
