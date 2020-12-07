#pragma once

#include "executor.h"
#include <algorithm>

namespace ratio
{

  class executor_listener
  {
    friend class executor;

  public:
    executor_listener(executor &exec) : exec(exec) { exec.listeners.push_back(this); }
    executor_listener(const executor_listener &orig) = delete;
    virtual ~executor_listener() { exec.listeners.erase(std::find(exec.listeners.begin(), exec.listeners.end(), this)); }

  private:
    virtual void tick() {}
    virtual void starting_atoms(const std::set<atom *>& atoms) {}
    virtual void ending_atoms(const std::set<atom *>& atoms) {}

    void end_atoms(const std::set<atom *>& atoms) { exec.end_atoms(atoms); }

  protected:
    executor &exec;
  };
} // namespace ratio