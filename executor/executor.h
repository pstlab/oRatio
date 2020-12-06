#pragma once

#include "inf_rational.h"
#include <map>
#include <set>

namespace ratio
{

  class core;
  class atom;

  class executor
  {
  public:
    executor(core &cr);
    executor(const executor &orig) = delete;
    ~executor();

    size_t get_current_time() const { return current_time; };

    void set_interval(const size_t &interval);
    void set_timeout(const size_t &delay);
    void stop();

    void reset_timelines();

  private:
    core &cr;
    bool executing = false;
    size_t current_time = 0;
    std::map<smt::inf_rational, std::set<atom *>> starting_atoms; // for each pulse, the atoms starting at that pulse..
    std::map<smt::inf_rational, std::set<atom *>> ending_atoms;   // for each pulse, the atoms ending at that pulse..
    std::set<smt::inf_rational> pulses;                           // all the pulses of the plan..
  };
} // namespace ratio