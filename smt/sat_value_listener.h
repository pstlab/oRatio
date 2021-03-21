
#pragma once

#include "sat_core.h"

namespace smt
{
  class sat_value_listener
  {
    friend class sat_core;

  public:
    sat_value_listener(sat_core &s) : sat(s) {}
    sat_value_listener(const sat_value_listener &that) = delete;
    virtual ~sat_value_listener() {}

  protected:
    void listen_sat(var v) noexcept { sat.listen(v, *this); }

  private:
    virtual void sat_value_change(const var &) {}

  private:
    sat_core &sat;
  };
} // namespace smt