#pragma once

#include "executor.h"

namespace ratio
{

  class executor_listener
  {
    friend class executor;

  public:
    executor_listener(executor &e) : exec(e) { exec.listeners.push_back(this); }
    executor_listener(const executor_listener &that) = delete;
    virtual ~executor_listener() { exec.listeners.erase(std::find(exec.listeners.begin(), exec.listeners.end(), this)); }

  protected:
    void dont_start_yet(const std::set<atom *> &atoms) {}
    void dont_end_yet(const std::set<atom *> &atoms) {}
    void failure(const std::set<atom *> &atoms) {}

  private:
    virtual void tick() { LOG("current time: " << exec.current_time); }
    virtual void starting(const std::set<atom *> &atoms) {}
    virtual void ending(const std::set<atom *> &atoms) {}

  private:
    executor &exec;
  };
} // namespace ratio
