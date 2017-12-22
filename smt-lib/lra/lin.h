#pragma once

#include "rational.h"
#include <map>

namespace smt
{

typedef size_t var;

class lin
{
public:
  lin();
  lin(const rational &known_term);
  lin(const var v, const rational &c);

public:
  lin operator+(const lin &right) const;
  lin operator+(const rational &right) const;
  friend lin operator+(const rational &lhs, const lin &rhs);

  lin operator-(const lin &right) const;
  lin operator-(const rational &right) const;
  friend lin operator-(const rational &lhs, const lin &rhs);

  lin operator*(const rational &right) const;
  friend lin operator*(const rational &lhs, const lin &rhs);

  lin operator/(const rational &right) const;

  lin operator+=(const lin &right);
  lin operator+=(const std::pair<var, rational> &term);
  lin operator+=(const rational &right);

  lin operator-=(const lin &right);
  lin operator-=(const std::pair<var, rational> &term);
  lin operator-=(const rational &right);

  lin operator*=(const rational &right);

  lin operator/=(const rational &right);

  lin operator-() const;

  std::string to_string() const;

public:
  std::map<const var, rational> vars;
  rational known_term;
};
}