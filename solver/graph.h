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
public:
  graph(solver &slv);
  graph(const graph &that) = delete;
  ~graph();

private:
  solver &slv;
  smt::var gamma;            // this variable represents the validity of the current graph..
  std::deque<flaw *> flaw_q; // the flaw queue (for the graph building procedure)..
};

class flaw
{
public:
  flaw(graph &gr);
  flaw(const flaw &that) = delete;
  ~flaw();

private:
  graph &gr;
};

class resolver
{
public:
  resolver(graph &gr);
  resolver(const resolver &that) = delete;
  ~resolver();

private:
  graph &gr;
};
} // namespace ratio
