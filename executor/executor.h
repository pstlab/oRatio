#pragma once

#include "inf_rational.h"
#include <vector>
#include <mutex>
#include <map>
#include <set>

namespace ratio
{

  class core;
  class atom;
  class executor_listener;

  class executor
  {
    friend class executor_listener;

  public:
    executor(core &cr, const size_t &tick_duration, const smt::rational &units_for_milliseconds = smt::rational(1, 1000));
    executor(const executor &orig) = delete;
    ~executor();

    size_t get_current_time() const { return current_time; };

    void start();
    void stop();
    void set_timeout(const size_t &delay);

    void reset_timelines();

  private:
    void end_atoms(const std::set<atom *> &atoms);

  private:
    core &cr;
    const size_t tick_duration;                 // the duration of the ticks in milliseconds..
    const smt::rational units_for_milliseconds; // the number of plan units for a millisecond (e.g., 1/1000 means one unit for second)..
    std::vector<executor_listener *> listeners; // the executor listeners..
    std::mutex mtx;                             // a mutex for the critical sections..
    bool executing = false;
    size_t current_time;
    std::map<smt::inf_rational, std::set<atom *>> starting_atoms; // for each pulse (in milliseconds), the atoms starting at that pulse..
    std::map<smt::inf_rational, std::set<atom *>> ending_atoms;   // for each pulse (in milliseconds), the atoms ending at that pulse..
    std::set<smt::inf_rational> pulses;                           // all the pulses (in milliseconds) of the plan..
  };
} // namespace ratio