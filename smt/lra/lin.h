#pragma once

#include "rational.h"
#include <map>

namespace smt
{

  class lin
  {
  public:
    lin();
    lin(const rational &known_term);
    lin(const var v, const rational &c);

  public:
    lin operator+(const lin &rhs) const;
    lin operator+(const rational &rhs) const;
    friend lin operator+(const rational &lhs, const lin &rhs);

    lin operator-(const lin &rhs) const;
    lin operator-(const rational &rhs) const;
    friend lin operator-(const rational &lhs, const lin &rhs);

    lin operator*(const rational &rhs) const;
    friend lin operator*(const rational &lhs, const lin &rhs);

    lin operator/(const rational &rhs) const;

    lin operator+=(const lin &rhs);
    lin operator+=(const rational &rhs);

    lin operator-=(const lin &rhs);
    lin operator-=(const rational &rhs);

    lin operator*=(const rational &rhs);

    lin operator/=(const rational &rhs);

    lin operator-() const;

    friend std::string to_string(const lin &rhs);

  public:
    std::map<const var, rational> vars;
    rational known_term;
  };
} // namespace smt