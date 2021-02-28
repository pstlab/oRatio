#pragma once

#include "core_listener.h"
#include "solver_listener.h"
#include "inf_rational.h"
#include "lit.h"
#include <vector>
#include <map>
#include <set>

namespace ratio
{

  class solver;
  class predicate;
  class atom;
  class executor_listener;

  class executor : public core_listener, public solver_listener
  {
    friend class executor_listener;

  public:
    executor(solver &slv, const smt::rational &units_per_tick = smt::rational::ONE);
    executor(const executor &orig) = delete;
    ~executor();

    smt::rational get_current_time() const { return current_time; };

    void tick();
    void dont_start_yet(const std::set<atom *> &atoms);
    void dont_end_yet(const std::set<atom *> &atoms);
    void failure(const std::set<atom *> &atoms);

  private:
    void solution_found() override;
    void inconsistent_problem() override;

    void resolver_created(const resolver &r) override;

    void reset_timelines();

  private:
    smt::rational current_time;         // the current time in plan units..
    const smt::rational units_per_tick; // the number of plan units for each tick..
    std::vector<smt::lit> pending_facts;
    std::unordered_map<item *, smt::inf_rational> frozen_nums;
    std::unordered_map<item *, smt::lit> frozen_vals;
    std::set<atom *> not_starting, not_ending;
    std::map<smt::inf_rational, std::set<atom *>> s_atms, e_atms; // for each pulse, the atoms starting/ending at that pulse..
    std::set<smt::inf_rational> pulses;                           // all the pulses of the plan..
    std::vector<executor_listener *> listeners;                   // the executor listeners..
  };

  class execution_exception : public std::exception
  {
    const char *what() const noexcept override { return "the plan cannot be executed.."; }
  };
} // namespace ratio