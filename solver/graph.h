#pragma once

#include "rational.h"
#include <deque>
#include <unordered_map>
#ifdef GRAPH_PRUNING
#include <unordered_set>
#endif
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
class resolver;
class atom_flaw;

class graph
{
  friend class solver;
  friend class flaw;
  friend class composite_flaw;
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

  void set_estimated_cost(resolver &r, const smt::rational &cst);     // sets the estimated cost of the given resolver, propagating it to other resolvers..
  static const smt::rational evaluate(const std::vector<flaw *> &fs); // evaluates, together, the given vector of flaws..

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

  void check_gamma(); // checks and possibly resets the value of gamma..

private:
  solver &slv;                                                // the solver this graph belongs to..
  unsigned short accuracy = 1;                                // the current heuristic accuracy..
  static const unsigned short min_accuracy = MIN_ACCURACY;    // the minimum heuristic accuracy..
  static const unsigned short max_accuracy = MAX_ACCURACY;    // the maximum heuristic accuracy..
  smt::var gamma;                                             // this variable represents the validity of the current graph..
  resolver *res = nullptr;                                    // the current resolver (i.e. the cause for the new flaws)..
  std::deque<flaw *> flaw_q;                                  // the flaw queue (for the graph building procedure)..
  std::unordered_map<smt::var, std::vector<flaw *>> phis;     // the phi variables (propositional variable to flaws) of the flaws..
  std::unordered_map<smt::var, std::vector<resolver *>> rhos; // the rho variables (propositional variable to resolver) of the resolvers..
#ifdef GRAPH_PRUNING
  std::unordered_set<smt::var> already_closed; // already closed flaws (for avoiding duplicating graph pruning constraints)..
#endif
};

class flaw
{
  friend class graph;
  friend class solver;

public:
  flaw(graph &gr, const std::vector<resolver *> &causes, const bool &exclusive = false);
  flaw(const flaw &that) = delete;
  ~flaw();

  graph &get_graph() const { return gr; }
  smt::var get_phi() const { return phi; }
  const std::vector<resolver *> &get_resolvers() const { return resolvers; }
  const std::vector<resolver *> &get_causes() const { return causes; }
  const std::vector<resolver *> &get_supports() const { return supports; }
  bool is_expanded() const { return expanded; }

  smt::rational get_estimated_cost() const { return est_cost; }
  resolver *get_best_resolver() const;

#ifdef BUILD_GUI
  virtual std::string get_label() const = 0;
#endif

private:
  /**
   * Initializes this flaw.
   * 
   * @pre the solver must be at root-level.
   */
  void init();
  /**
   * Expands this flaw, invoking the compute_resolvers procedure.
   * 
   * @pre the solver must be at root-level.
   */
  void expand();
  virtual void compute_resolvers() = 0;

protected:
  /**
   * Adds the resolver 'r' to this flaw.
   */
  void add_resolver(resolver &r);
  /**
   * Adds this flaw to the preconditions of the resolver 'r'.
   */
  void make_precondition_of(resolver &r);

private:
  graph &gr;                                                 // the graph this flaw belongs to..
  smt::var phi;                                              // the propositional variable indicating whether the flaw is active or not..
  smt::rational est_cost = smt::rational::POSITIVE_INFINITY; // the estimated cost of the flaw..
  std::vector<resolver *> resolvers;                         // the resolvers for this flaw..
  const std::vector<resolver *> causes;                      // the causes for having this flaw (used for activating the flaw through causal propagation)..
  std::vector<resolver *> supports;                          // the resolvers supported by this flaw (used for propagating cost estimates)..
  const bool exclusive;                                      // a boolean indicating whether the flaw is exclusive (i.e. exactly one of its resolver can be applied)..
  bool expanded = false;                                     // a boolean indicating whether the flaw has been expanded..
};

class resolver
{
  friend class graph;
  friend class solver;
  friend class flaw;

public:
  resolver(graph &gr, const smt::rational &cost, flaw &eff);
  resolver(graph &gr, const smt::var &r, const smt::rational &cost, flaw &eff);
  resolver(const resolver &that) = delete;
  ~resolver();

  graph &get_graph() const { return gr; }
  smt::var get_rho() const { return rho; }
  smt::rational get_intrinsic_cost() const { return intrinsic_cost; }
  smt::rational get_estimated_cost() const { return est_cost + intrinsic_cost; }
  flaw &get_effect() const { return effect; }
  const std::vector<flaw *> &get_preconditions() const { return preconditions; }

#ifdef BUILD_GUI
  virtual std::string get_label() const = 0;
#endif

private:
  virtual void apply() = 0;

private:
  graph &gr;                                                 // the graph this resolver belongs to..
  const smt::var rho;                                        // the propositional variable indicating whether the resolver is active or not..
  const smt::rational intrinsic_cost;                        // the intrinsic cost of the resolver..
  smt::rational est_cost = smt::rational::POSITIVE_INFINITY; // the estimated cost of the resolver, computed by the heuristic, except for the intrinsic cost..
  flaw &effect;                                              // the flaw solved by this resolver..
  std::vector<flaw *> preconditions;                         // the preconditions of this resolver..
};
} // namespace ratio
