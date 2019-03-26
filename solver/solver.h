#pragma once

#include "core.h"

namespace ratio
{

class solver : public core, private smt::theory
{
public:
  solver();
  solver(const solver &orig) = delete;
  ~solver();

  /**
   * Initializes the solver.
   */
  void init();

  /**
   * Solves the given problem.
   */
  void solve() override;

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
