#pragma once

#include "core.h"

namespace ratio
{

class flaw;
class resolver;

class solver : public core, private smt::theory
{
  friend class flaw;
  friend class resolver;
  friend class atom_flaw;

public:
  solver();
  solver(const solver &orig) = delete;
  virtual ~solver();

  expr new_enum(const type &tp, const std::unordered_set<item *> &allowed_vals) override;
  void solve() override; // solves the given problem..

private:
  void new_fact(atom &atm) override;                                      // creates a new fact token..
  void new_goal(atom &atm) override;                                      // creates a new goal token..
  void new_disjunction(context &d_ctx, const disjunction &disj) override; // creates a new disjunction..

  bool propagate(const smt::lit &p, std::vector<smt::lit> &cnfl) override;
  bool check(std::vector<smt::lit> &cnfl) override;
  void push() override;
  void pop() override;

  void new_flaw(flaw &f);
  void new_resolver(resolver &r);
  void new_causal_link(flaw &f, resolver &r);

  void set_estimated_cost(resolver &r, const smt::rational &cst);     // sets the estimated cost of the given resolver, propagating it to other resolvers..
  static const smt::rational evaluate(const std::vector<flaw *> &fs); // evaluates, together, the given vector of flaws..
  flaw *select_flaw();                                                // selects the most promising (i.e. the most expensive one) flaw from the 'flaws' set, returns a nullptr if there are no more active flaws..

private:
  resolver *res = nullptr;                                    // the current resolver (will go straight into the trail)..
  std::unordered_set<flaw *> flaws;                           // the current active flaws..
  std::unordered_map<smt::var, std::vector<flaw *>> phis;     // the phi variables (propositional variable to flaws) of the flaws..
  std::unordered_map<smt::var, std::vector<resolver *>> rhos; // the rho variables (propositional variable to resolver) of the resolvers..
  std::deque<flaw *> flaw_q;                                  // the flaw queue (for the graph building procedure)..

  struct layer
  {

    layer(resolver *const r) : r(r) {}

    resolver *const r;                                       // the resolver which introduced the new layer..
    std::unordered_map<resolver *, smt::rational> old_costs; // the old estimated resolvers' costs..
    std::unordered_set<flaw *> new_flaws;                    // the just activated flaws..
    std::unordered_set<flaw *> solved_flaws;                 // the just solved flaws..
  };
  std::vector<layer> trail; // the list of applied resolvers, with the associated changes made, in chronological order..
};
} // namespace ratio
