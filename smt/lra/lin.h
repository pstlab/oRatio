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
    lin operator+(const lin &rhs) const noexcept;
    lin operator+(const rational &rhs) const noexcept;
    friend lin operator+(const rational &lhs, const lin &rhs) noexcept;

    lin operator-(const lin &rhs) const noexcept;
    lin operator-(const rational &rhs) const noexcept;
    friend lin operator-(const rational &lhs, const lin &rhs) noexcept;

    lin operator*(const rational &rhs) const noexcept;
    friend lin operator*(const rational &lhs, const lin &rhs) noexcept;

    lin operator/(const rational &rhs) const noexcept;

    lin operator+=(const lin &rhs) noexcept;
    lin operator+=(const rational &rhs) noexcept;

    lin operator-=(const lin &rhs) noexcept;
    lin operator-=(const rational &rhs) noexcept;

    lin operator*=(const rational &rhs) noexcept;

    lin operator/=(const rational &rhs) noexcept;

    lin operator-() const noexcept;

    friend std::string to_string(const lin &rhs) noexcept;

  public:
    std::map<const var, rational> vars;
    rational known_term;
  };
} // namespace smt