#pragma once

#include "solver_listener.h"
#include "inf_rational.h"
#include "lit.h"
#include <thread>
#include <vector>
#include <mutex>
#include <map>
#include <set>

namespace ratio
{

  class solver;
  class predicate;
  class atom;
  class executor_listener;

  class executor : public solver_listener
  {
    friend class executor_listener;

  public:
    executor(solver &slv, const size_t &tick_duration, const smt::rational &units_per_tick = smt::rational::ONE);
    executor(const executor &orig) = delete;
    ~executor();

    smt::rational get_current_time() const { return current_time; };

    std::thread start();
    void stop();

    void reset_timelines();

  private:
    void tick();
    void dont_start_yet(const std::set<atom *> &atoms);
    void dont_end_yet(const std::set<atom *> &atoms);
    void failure(const std::set<atom *> &atoms);

    void flaw_created(const flaw &f) override;
    void resolver_created(const resolver &r) override;

  private:
    predicate &int_pred;                // the interval predicate..
    predicate &imp_pred;                // the impulse predicate..
    const size_t tick_duration;         // the duration of each tick in milliseconds..
    smt::rational current_time;         // the current time in plan units..
    const smt::rational units_per_tick; // the number of plan units for each tick..
    std::mutex mtx;                     // a mutex for the critical sections..
    bool executing = false;
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