#pragma once

#include "idl_theory.h"

namespace smt
{
  class idl_value_listener
  {
    friend class idl_theory;

  public:
    idl_value_listener(idl_theory &s) : th(s) {}
    idl_value_listener(const idl_value_listener &that) = delete;
    virtual ~idl_value_listener() {}

  protected:
    inline void listen_idl(var v) { th.listen(v, this); }

  private:
    virtual void idl_value_change(const var &) {}

  private:
    idl_theory &th;
  };
} // namespace smt