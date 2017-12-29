#pragma once

#include "core.h"
#include <deque>

namespace ratio
{

class flaw;
class resolver;
class atom_flaw;
#ifndef NDEBUG
class solver_listener;
#endif

class solver : public core, public smt::theory
{
  friend class flaw;
  friend class resolver;
  friend class atom_flaw;
#ifndef NDEBUG
  friend class solver_listener;
#endif

public:
  solver();
  solver(const solver &orig) = delete;
  virtual ~solver();

  void init(); // initializes the solver..

  expr new_enum(const type &tp, const std::unordered_set<item *> &allowed_vals) override;

private:
  void new_fact(atom &atm) override;
  void new_goal(atom &atm) override;
  void new_disjunction(context &d_ctx, const disjunction &disj) override;

public:
  void solve() override;                                                    // solves the given problem..
  atom_flaw &get_reason(const atom &atm) const { return *reason.at(&atm); } // returns the flaw which has given rise to the atom..

private:
  void build();                     // builds the planning graph..
  bool is_deferrable(flaw &f);      // checks whether the given flaw is deferrable..
  void add_layer();                 // adds a layer to the current planning graph..
  bool has_inconsistencies();       // checks whether the types have some inconsistency..
  void expand_flaw(flaw &f);        // expands the given flaw into the planning graph..
  void apply_resolver(resolver &r); // applies the given resolver into the planning graph..

  void new_flaw(flaw &f);
  void new_resolver(resolver &r);
  void new_causal_link(flaw &f, resolver &r);

  void set_estimated_cost(resolver &r, const smt::rational &cst); // sets the estimated cost of the given resolver propagating it to other resolvers..
  flaw *select_flaw();                                            // selects the most expensive flaw from the 'flaws' set, returns a nullptr if there are no active flaws..

  bool propagate(const smt::lit &p, std::vector<smt::lit> &cnfl) override;
  bool check(std::vector<smt::lit> &cnfl) override;
  void push() override;
  void pop() override;

private:
  struct layer
  {

    layer(resolver *const r) : r(r) {}

    resolver *const r;
    std::unordered_map<resolver *, smt::rational> old_costs; // the old estimated resolvers' costs..
    std::unordered_set<flaw *> new_flaws;                    // the just activated flaws..
    std::unordered_set<flaw *> solved_flaws;                 // the just solved flaws..
  };

  resolver *res = nullptr;                                    // the current resolver (will be into the trail)..
  smt::var gamma;                                             // this variable represents the validity of the current graph..
  unsigned short accuracy = 1;                                // the current heuristic accuracy..
  std::deque<flaw *> flaw_q;                                  // the flaw queue (for graph building procedure)..
  std::unordered_set<flaw *> flaws;                           // the current active flaws..
  std::unordered_map<smt::var, std::vector<flaw *>> phis;     // the phi variables (boolean variable to flaws) of the flaws..
  std::unordered_map<smt::var, std::vector<resolver *>> rhos; // the rho variables (boolean variable to resolver) of the resolvers..
  std::unordered_map<const atom *, atom_flaw *> reason;       // the reason for having introduced an atom..
  std::vector<layer> trail;                                   // the list of resolvers in chronological order..
#ifndef NDEBUG
  std::vector<solver_listener *> listeners; // the causal-graph listeners..
#endif
};
}