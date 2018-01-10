#pragma once

#include "rational.h"
#include <vector>
#include <string>

namespace smt
{
typedef size_t var;
}

namespace ratio
{

class solver;
class resolver;

class flaw
{
  friend class solver;
  friend class resolver;

public:
  flaw(solver &slv, const std::vector<resolver *> &causes, const bool &exclusive = false, const bool &structural = false);
  flaw(const flaw &orig) = delete;
  virtual ~flaw();

  bool is_expanded() const { return expanded; }
  const smt::var &get_phi() const { return phi; }
  std::vector<resolver *> get_resolvers() const { return resolvers; }
  std::vector<resolver *> get_causes() const { return causes; }
  std::vector<resolver *> get_supports() const { return supports; }
  smt::rational get_estimated_cost() const;
  resolver *get_best_resolver() const;

  virtual std::string get_label() const = 0;

private:
  void init();
  void expand();
  virtual void compute_resolvers() = 0;

protected:
  void add_resolver(resolver &r);

protected:
  solver &slv;

private:
  const bool exclusive;
  const bool structural;
  bool expanded = false;
  smt::var phi;                      // the propositional variable indicates whether the flaw is active or not..
  std::vector<resolver *> resolvers; // the resolvers for this flaw..
  std::vector<resolver *> causes;    // the causes for having this flaw..
  std::vector<resolver *> supports;  // the resolvers supported by this flaw..
};
}