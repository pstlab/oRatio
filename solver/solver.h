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
};
} // namespace ratio
