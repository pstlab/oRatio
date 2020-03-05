#pragma once

#include "rational.h"
#include <vector>

namespace ratio
{

class solver;
class graph;
class resolver;

class flaw
{
  friend class solver;
  friend class graph;
  friend class resolver;

public:
  flaw(graph &gr, const std::vector<resolver *> &causes, const bool &exclusive = false);
  flaw(const flaw &that) = delete;
  ~flaw();

  graph &get_graph() const { return gr; }
  smt::var get_phi() const { return phi; }
  const std::vector<resolver *> &get_resolvers() const { return resolvers; }
  const std::vector<resolver *> &get_causes() const { return causes; }
  const std::vector<resolver *> &get_supports() const { return supports; }

  smt::rational get_estimated_cost() const { return est_cost; }
  bool is_expanded() const { return expanded; }

  resolver *get_cheapest_resolver() const;
  virtual resolver *get_best_resolver() const { return get_cheapest_resolver(); }

  virtual std::string get_label() const = 0;

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

private:
  graph &gr;                                                 // the graph this flaw belongs to..
  smt::var phi;                                              // the propositional variable indicating whether the flaw is active or not..
  smt::rational est_cost = smt::rational::POSITIVE_INFINITY; // the current estimated cost of the flaw..
  bool expanded = false;                                     // a boolean indicating whether the flaw has been expanded..
  std::vector<resolver *> resolvers;                         // the resolvers for this flaw..
  const std::vector<resolver *> causes;                      // the causes for having this flaw (used for activating the flaw through causal propagation)..
  std::vector<resolver *> supports;                          // the resolvers supported by this flaw (used for propagating cost estimates)..
  const bool exclusive;                                      // a boolean indicating whether the flaw is exclusive (i.e. exactly one of its resolver can be applied)..
};
} // namespace ratio