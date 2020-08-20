#pragma once

#include "defs.h"
#include <cstddef>
#include <limits>

namespace smt
{

  /**
   * This class is used for representing propositional literals.
   */
  class lit
  {
  public:
    lit(var v = -1, bool sign = true) : x((v << 1) + sign) {}
    virtual ~lit() {}

    friend var variable(const lit &p) { return p.x >> 1; }
    friend bool sign(const lit &p) { return p.x & 1; }
    friend size_t index(const lit &p) { return p.x; }
    friend bool is_undefined(const lit &p) { return p.x == std::numeric_limits<size_t>::max(); }

    lit operator!() const
    {
      lit p;
      p.x = x ^ 1;
      return p;
    }
    bool operator<(const lit &rhs) const { return x < rhs.x; }
    bool operator==(const lit &rhs) const { return x == rhs.x; }
    bool operator!=(const lit &rhs) const { return x != rhs.x; }

  private:
    size_t x;
  };
} // namespace smt