#pragma once
#include "core_listener.h"

namespace ratio
{

  class executor : public core_listener
  {
  public:
    executor(core &cr);
    executor(const executor &orig) = delete;
    ~executor();

    void set_interval(const size_t& interval);
    void set_timeout(const size_t& delay);
    void stop();

    void state_changed() override;

  private:
    bool executing = false;
    std::map<smt::inf_rational, std::set<atom *>> starting_atoms; // for each pulse, the atoms starting at that pulse..
    std::map<smt::inf_rational, std::set<atom *>> ending_atoms;   // for each pulse, the atoms ending at that pulse..
    std::set<smt::inf_rational> pulses;                           // all the pulses of the plan..
  };
} // namespace ratio