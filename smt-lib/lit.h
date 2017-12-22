#pragma once

#include <cstddef>

namespace smt
{

typedef size_t var;

class lit
{
public:
  lit(var v = -1, bool sign = true) : v(v), sign(sign) {}
  virtual ~lit() {}

  lit operator!() const { return lit(v, !sign); }
  bool operator<(const lit &rhs) const { return v < rhs.v || (v == rhs.v && sign < rhs.sign); }
  bool operator==(const lit &rhs) const { return v == rhs.v && sign == rhs.sign; }
  bool operator!=(const lit &rhs) const { return !operator==(rhs); }

public:
  var v;
  bool sign;
};
}