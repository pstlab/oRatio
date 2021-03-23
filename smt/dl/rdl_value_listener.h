#pragma once

#include "rdl_theory.h"

namespace smt
{
  class rdl_value_listener
  {
    friend class rdl_theory;

  public:
    rdl_value_listener(rdl_theory &s) : th(s) {}
    rdl_value_listener(const rdl_value_listener &that) = delete;
    virtual ~rdl_value_listener() {}

  protected:
    inline void listen_rdl(var v) noexcept { th.listen(v, this); }

  private:
    virtual void rdl_value_change(const var &) {}

  private:
    rdl_theory &th;
  };
} // namespace smt