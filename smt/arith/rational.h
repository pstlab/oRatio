#pragma once

#include "smt_export.h"
#include "defs.h"
#include <string>

namespace smt
{
  class rational
  {
  public:
    SMT_EXPORT static const rational ZERO;
    SMT_EXPORT static const rational ONE;
    SMT_EXPORT static const rational POSITIVE_INFINITY;
    SMT_EXPORT static const rational NEGATIVE_INFINITY;

    SMT_EXPORT explicit rational();
    SMT_EXPORT explicit rational(I n);
    SMT_EXPORT explicit rational(I n, I d);

    SMT_EXPORT operator double() const noexcept { return static_cast<double>(num) / den; }
    SMT_EXPORT inline I numerator() const noexcept { return num; }
    SMT_EXPORT inline I denominator() const noexcept { return den; }

    SMT_EXPORT inline friend bool is_integer(const rational &rhs) noexcept { return rhs.den == 1; }
    SMT_EXPORT inline friend bool is_positive(const rational &rhs) noexcept { return rhs.num > 0; }
    SMT_EXPORT inline friend bool is_negative(const rational &rhs) noexcept { return rhs.num < 0; }
    SMT_EXPORT inline friend bool is_infinite(const rational &rhs) noexcept { return rhs.den == 0; }
    SMT_EXPORT inline friend bool is_positive_infinite(const rational &rhs) noexcept { return is_positive(rhs) && is_infinite(rhs); }
    SMT_EXPORT inline friend bool is_negative_infinite(const rational &rhs) noexcept { return is_negative(rhs) && is_infinite(rhs); }

    SMT_EXPORT bool operator!=(const rational &rhs) const noexcept;
    SMT_EXPORT bool operator<(const rational &rhs) const noexcept;
    SMT_EXPORT bool operator<=(const rational &rhs) const noexcept;
    SMT_EXPORT bool operator==(const rational &rhs) const noexcept;
    SMT_EXPORT bool operator>=(const rational &rhs) const noexcept;
    SMT_EXPORT bool operator>(const rational &rhs) const noexcept;

    SMT_EXPORT bool operator!=(const I &rhs) const noexcept;
    SMT_EXPORT bool operator<(const I &rhs) const noexcept;
    SMT_EXPORT bool operator<=(const I &rhs) const noexcept;
    SMT_EXPORT bool operator==(const I &rhs) const noexcept;
    SMT_EXPORT bool operator>=(const I &rhs) const noexcept;
    SMT_EXPORT bool operator>(const I &rhs) const noexcept;

    SMT_EXPORT rational operator+(const rational &rhs) const noexcept;
    SMT_EXPORT rational operator-(const rational &rhs) const noexcept;
    SMT_EXPORT rational operator*(const rational &rhs) const noexcept;
    SMT_EXPORT rational operator/(const rational &rhs) const noexcept;

    SMT_EXPORT rational operator+(const I &rhs) const noexcept;
    SMT_EXPORT rational operator-(const I &rhs) const noexcept;
    SMT_EXPORT rational operator*(const I &rhs) const noexcept;
    SMT_EXPORT rational operator/(const I &rhs) const noexcept;

    SMT_EXPORT rational &operator+=(const rational &rhs) noexcept;
    SMT_EXPORT rational &operator-=(const rational &rhs) noexcept;
    SMT_EXPORT rational &operator*=(const rational &rhs) noexcept;
    SMT_EXPORT rational &operator/=(const rational &rhs) noexcept;

    SMT_EXPORT rational &operator+=(const I &rhs) noexcept;
    SMT_EXPORT rational &operator-=(const I &rhs) noexcept;
    SMT_EXPORT rational &operator*=(const I &rhs) noexcept;
    SMT_EXPORT rational &operator/=(const I &rhs) noexcept;

    SMT_EXPORT friend rational operator+(const I &lhs, const rational &rhs) noexcept;
    SMT_EXPORT friend rational operator-(const I &lhs, const rational &rhs) noexcept;
    SMT_EXPORT friend rational operator*(const I &lhs, const rational &rhs) noexcept;
    SMT_EXPORT friend rational operator/(const I &lhs, const rational &rhs) noexcept;

    SMT_EXPORT rational operator-() const noexcept;

  private:
    void normalize() noexcept;

    friend SMT_EXPORT std::string to_string(const rational &rhs) noexcept;

  private:
    I num; // the numerator..
    I den; // the denominator..
  };
} // namespace smt