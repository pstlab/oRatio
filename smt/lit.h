#pragma once

#include "defs.h"
#include <limits>
#include <string>

namespace smt
{

  /**
   * This class is used for representing propositional literals.
   */
  class lit
  {
  public:
    explicit lit(var v = -1, bool sign = true) : x((v << 1) + sign) {}
    virtual ~lit() {}

    friend var variable(const lit &p) noexcept { return p.x >> 1; }
    friend bool sign(const lit &p) noexcept { return p.x & 1; }
    friend size_t index(const lit &p) noexcept { return p.x; }
    friend bool is_undefined(const lit &p) noexcept { return p.x == std::numeric_limits<size_t>::max(); }

    lit operator!() const
    {
      lit p;
      p.x = x ^ 1;
      return p;
    }
    bool operator<(const lit &rhs) const noexcept { return x < rhs.x; }
    bool operator==(const lit &rhs) const noexcept { return x == rhs.x; }
    bool operator!=(const lit &rhs) const noexcept { return x != rhs.x; }

    friend std::string to_string(const lit &p) noexcept { return sign(p) ? ("b" + std::to_string(variable(p))) : ("¬b" + std::to_string(variable(p))); }

  private:
    size_t x;
  };

  const lit FALSE_lit = lit(FALSE_var);
  const lit TRUE_lit = !FALSE_lit;
} // namespace smt