#pragma once

#include "rational.h"
#include <deque>
#include <unordered_map>
#include <map>
#include <set>
#include <unordered_set>
#include <vector>

namespace smt
{
typedef size_t var;
} // namespace smt

namespace ratio
{

class solver;
class flaw;
class composite_flaw;
class composite_resolver;
class resolver;
class atom_flaw;

class graph
{
  friend class solver;
  friend class flaw;
  friend class composite_flaw;
  friend class composite_resolver;
  friend class atom_flaw;

public:
  graph(solver &slv);
  graph(const graph &that) = delete;
  ~graph();

  solver &get_solver() const { return slv; }
  unsigned short get_accuracy() const { return accuracy; }

private:
  void new_flaw(flaw &f, const bool &enqueue = true); // notifies the graph that a new flaw 'f' has been created..
  void new_resolver(resolver &r);                     // notifies the graph that a new resolver 'r' has been created..
  void new_causal_link(flaw &f, resolver &r);         // notifies the graph that a new causal link between a flaw 'f' and a resolver 'r' has been created..

  void set_estimated_cost(flaw &f, std::unordered_set<flaw *> &visited); // sets the estimated cost of the given flaw, propagating it to other flaws..

  void build();                                 // builds the planning graph..
  void add_layer();                             // adds a layer to the current planning graph..
  void set_accuracy(const unsigned short &acc); // set the heuristic accuracy to the given value..

  void expand_flaw(flaw &f);        // expands the given flaw into the planning graph..
  void apply_resolver(resolver &r); // applies the given resolver into the planning graph..

#ifdef DEFERRABLE_FLAWS
  bool is_deferrable(flaw &f); // checks whether the given flaw is deferrable..
#endif

#ifdef CHECK_CYCLES
  std::vector<std::vector<resolver const *>> circuits(flaw &f, resolver &r) const; // returns all the simple circuits starting from node 'f' and containing resolver 'r'..
#endif

#ifdef CHECK_GRAPH
  void check_graph();                                                                                                                // checks the applicability of the resolvers and performs some graph refinement..
  bool check_flaw(flaw &f, std::unordered_set<resolver *> &visited, std::vector<resolver *> &inv_r, std::vector<resolver *> &inc_r); // checks the solvability of the given flaw filling the set of invalid resolvers (i.e., those which can never be applyed) and the set of incompatible resolvers (i.e., those which are not valid in the current graph)..
#endif

  void check_gamma(); // checks and possibly resets the value of gamma..

private:
  solver &slv;                                                // the solver this graph belongs to..
  unsigned short accuracy = 1;                                // the current heuristic accuracy..
  smt::var gamma;                                             // this variable represents the validity of the current graph..
  resolver *res = nullptr;                                    // the current resolver (i.e. the cause for the new flaws)..
  std::deque<flaw *> flaw_q;                                  // the flaw queue (for the graph building procedure)..
  std::unordered_map<smt::var, std::vector<flaw *>> phis;     // the phi variables (propositional variable to flaws) of the flaws..
  std::unordered_map<smt::var, std::vector<resolver *>> rhos; // the rho variables (propositional variable to resolver) of the resolvers..
#ifdef GRAPH_PRUNING
  std::unordered_set<smt::var> already_closed; // already closed flaws (for avoiding duplicating graph pruning constraints)..
#endif
#ifdef GRAPH_REFINING
  std::unordered_set<resolver *> empty_precs_resolvers; // resolvers having an empty precondition set..
#endif
};
} // namespace ratio
