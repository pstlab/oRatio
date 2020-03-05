#pragma once

#include "defs.h"
#include <cstddef>

namespace smt
{

/**
 * This class is used for representing propositional literals.
 */
class lit
{
public:
  lit(var v = -1, bool sign = true) : v(v), sign(sign) {}
  virtual ~lit() {}

  const var &get_var() const { return v; }
  const bool &get_sign() const { return sign; }

  lit operator!() const { return lit(v, !sign); }
  bool operator<(const lit &rhs) const { return v < rhs.v || (v == rhs.v && sign < rhs.sign); }
  bool operator==(const lit &rhs) const { return v == rhs.v && sign == rhs.sign; }
  bool operator!=(const lit &rhs) const { return !operator==(rhs); }

private:
  var v;     // this is the variable of the propositional literal..
  bool sign; // this is the sign of the propositional literal..
};
} // namespace smt