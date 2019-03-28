#pragma once

#include "rational.h"
#include <deque>
#include <unordered_map>
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

class graph
{
  friend class solver;
  friend class flaw;
  friend class composite_flaw;

public:
  graph(solver &slv);
  graph(const graph &that) = delete;
  ~graph();

  solver &get_solver() const { return slv; }
  unsigned short get_accuracy() const { return accuracy; }

private:
  void new_flaw(flaw &f);
  void new_resolver(resolver &r);
  void new_causal_link(flaw &f, resolver &r);

  void set_estimated_cost(resolver &r, const smt::rational &cst);     // sets the estimated cost of the given resolver, propagating it to other resolvers..
  static const smt::rational evaluate(const std::vector<flaw *> &fs); // evaluates, together, the given vector of flaws..

  void build();             // builds the planning graph..
  void add_layer();         // adds a layer to the current planning graph..
  void increase_accuracy(); // increases the heuristic accuracy by one..

  void expand_flaw(flaw &f);        // expands the given flaw into the planning graph..
  void apply_resolver(resolver &r); // applies the given resolver into the planning graph..

private:
  solver &slv;                                                // the solver this graph belongs to..
  unsigned short accuracy = MIN_ACCURACY;                     // the current heuristic accuracy..
  static const unsigned short max_accuracy = MAX_ACCURACY;    // the maximum heuristic accuracy..
  smt::var gamma;                                             // this variable represents the validity of the current graph..
  resolver *res = nullptr;                                    // the current resolver (i.e. the cause for the new flaws)..
  std::deque<flaw *> flaw_q;                                  // the flaw queue (for the graph building procedure)..
  std::unordered_map<smt::var, std::vector<flaw *>> phis;     // the phi variables (propositional variable to flaws) of the flaws..
  std::unordered_map<smt::var, std::vector<resolver *>> rhos; // the rho variables (propositional variable to resolver) of the resolvers..
#if defined CHECK_UNIFICATIONS || defined CHECK_COMPOSITE_FLAWS
  bool checking = false;
#endif
};

class flaw
{
public:
  flaw(graph &gr, const std::vector<resolver *> &causes, const bool &exclusive = false);
  flaw(const flaw &that) = delete;
  ~flaw();

  graph &get_graph() const { return gr; }
  smt::var get_phi() const { return phi; }
  std::vector<resolver *> get_resolvers() const { return resolvers; }
  std::vector<resolver *> get_causes() const { return causes; }
  std::vector<resolver *> get_supports() const { return supports; }
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
  void add_resolver(resolver &r);

private:
  graph &gr;                                                 // the graph this flaw belongs to..
  smt::var phi;                                              // the propositional variable indicating whether the flaw is active or not..
  smt::rational est_cost = smt::rational::POSITIVE_INFINITY; // the estimated cost of the flaw..
  std::vector<resolver *> resolvers;                         // the resolvers for this flaw..
  std::vector<resolver *> causes;                            // the causes for having this flaw..
  std::vector<resolver *> supports;                          // the resolvers supported by this flaw..
  const bool exclusive;                                      // a boolean indicating whether the flaw is exclusive (i.e. exactly one of its resolver can be applied)..
  bool expanded = false;                                     // a boolean indicating whether the flaw has been expanded..
};

class resolver
{
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
  std::vector<flaw *> get_preconditions() const { return preconditions; }

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
