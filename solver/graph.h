#pragma once

#include "rational.h"
#include <vector>
#include <deque>

namespace smt
{
typedef size_t var;
} // namespace smt

namespace ratio
{

class solver;
class flaw;
class resolver;

class graph
{
  friend class flaw;
  friend class resolver;

public:
  graph(solver &slv);
  graph(const graph &that) = delete;
  ~graph();

  solver &get_solver() { return slv; }

private:
  solver &slv;
  smt::var gamma;            // this variable represents the validity of the current graph..
  std::deque<flaw *> flaw_q; // the flaw queue (for the graph building procedure)..
};

class flaw
{
public:
  flaw(graph &gr, const std::vector<resolver *> &causes, const bool &exclusive = false);
  flaw(const flaw &that) = delete;
  ~flaw();

  graph &get_graph() { return gr; }

private:
  graph &gr;
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
public:
  resolver(graph &gr, const smt::rational &cost, flaw &eff);
  resolver(graph &gr, const smt::var &r, const smt::rational &cost, flaw &eff);
  resolver(const resolver &that) = delete;
  ~resolver();

  graph &get_graph() { return gr; }

private:
  graph &gr;
  const smt::var rho;                                        // the propositional variable indicating whether the resolver is active or not..
  const smt::rational intrinsic_cost;                        // the intrinsic cost of the resolver..
  smt::rational est_cost = smt::rational::POSITIVE_INFINITY; // the estimated cost of the resolver, computed by the heuristic, except for the intrinsic cost..
  flaw &effect;                                              // the flaw solved by this resolver..
  std::vector<flaw *> preconditions;                         // the preconditions of this resolver..
};
} // namespace ratio
