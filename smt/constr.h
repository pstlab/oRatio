#pragma once

#include "lit.h"
#include <vector>

namespace smt
{

  class sat_core;

  /**
   * This class is used for representing propositional constraints.
   */
  class constr
  {
    friend class sat_core;

  protected:
    constr(sat_core &s) : s(s) {}
    constr(const constr &orig) = delete;
    virtual ~constr() {}

  private:
    virtual const bool propagate(const lit &p) = 0;
    virtual const bool simplify() = 0;
    virtual void remove() = 0;
    virtual void get_reason(const lit &p, std::vector<lit> &out_reason) const = 0;

  protected:
    std::vector<constr *> &watches(const lit &p);
    bool enqueue(const lit &p);

    lbool value(const var &x) const;
    lbool value(const lit &p) const;

  private:
    sat_core &s;
  };
} // namespace smt