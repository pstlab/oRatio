#pragma once

#include "rational.h"
#include <map>

namespace smt
{
  class lin
  {
  public:
    SMT_EXPORT explicit lin();
    SMT_EXPORT explicit lin(const rational &known_term);
    SMT_EXPORT explicit lin(const var v, const rational &c);

  public:
    SMT_EXPORT lin operator+(const lin &rhs) const noexcept;
    SMT_EXPORT lin operator+(const rational &rhs) const noexcept;
    SMT_EXPORT friend lin operator+(const rational &lhs, const lin &rhs) noexcept;

    SMT_EXPORT lin operator-(const lin &rhs) const noexcept;
    SMT_EXPORT lin operator-(const rational &rhs) const noexcept;
    SMT_EXPORT friend lin operator-(const rational &lhs, const lin &rhs) noexcept;

    SMT_EXPORT lin operator*(const rational &rhs) const noexcept;
    SMT_EXPORT friend lin operator*(const rational &lhs, const lin &rhs) noexcept;

    SMT_EXPORT lin operator/(const rational &rhs) const noexcept;

    SMT_EXPORT lin operator+=(const lin &rhs) noexcept;
    SMT_EXPORT lin operator+=(const rational &rhs) noexcept;

    SMT_EXPORT lin operator-=(const lin &rhs) noexcept;
    SMT_EXPORT lin operator-=(const rational &rhs) noexcept;

    SMT_EXPORT lin operator*=(const rational &rhs) noexcept;

    SMT_EXPORT lin operator/=(const rational &rhs) noexcept;

    SMT_EXPORT lin operator-() const noexcept;

    SMT_EXPORT friend std::string to_string(const lin &rhs) noexcept;

  public:
    std::map<const var, rational> vars;
    rational known_term;
  };
} // namespace smt