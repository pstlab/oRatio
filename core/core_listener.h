#pragma once

#include "core.h"
#include <algorithm>

namespace ratio
{
  class core_listener
  {
    friend class core;

  public:
    core_listener(core &cr) : cr(cr) { cr.listeners.push_back(this); }
    core_listener(const core_listener &orig) = delete;
    virtual ~core_listener() { cr.listeners.erase(std::find(cr.listeners.begin(), cr.listeners.end(), this)); }

  private:
    virtual void log(const std::string &msg) {}
    virtual void read(const std::string &script) {}
    virtual void read(const std::vector<std::string> &files) {}

    virtual void state_changed() {}

    virtual void started_solving() {}
    virtual void solution_found() {}
    virtual void inconsistent_problem() {}

  protected:
    core &cr;
  };
} // namespace ratio