#pragma once

#include "lra_theory.h"

namespace smt
{
  class lra_value_listener
  {
    friend class lra_theory;

  public:
    lra_value_listener(lra_theory &s) : th(s) {}
    lra_value_listener(const lra_value_listener &that) = delete;
    virtual ~lra_value_listener() {}

  protected:
    inline void listen_lra(var v) noexcept { th.listen(v, this); }

  private:
    virtual void lra_value_change(const var &) {}

  private:
    lra_theory &th;
  };
} // namespace smt