#pragma once

#include "defs.h"
#include <string>

namespace smt
{

  class rational
  {
  public:
    static const rational ZERO;
    static const rational ONE;
    static const rational POSITIVE_INFINITY;
    static const rational NEGATIVE_INFINITY;

    rational();
    rational(I n);
    rational(I n, I d);

    operator double() const noexcept { return static_cast<double>(num) / den; }
    I numerator() const noexcept { return num; }
    I denominator() const noexcept { return den; }

    friend bool is_integer(const rational &rhs) noexcept { return rhs.den == 1; }
    friend bool is_positive(const rational &rhs) noexcept { return rhs.num > 0; }
    friend bool is_negative(const rational &rhs) noexcept { return rhs.num < 0; }
    friend bool is_infinite(const rational &rhs) noexcept { return rhs.den == 0; }
    friend bool is_positive_infinite(const rational &rhs) noexcept { return is_positive(rhs) && is_infinite(rhs); }
    friend bool is_negative_infinite(const rational &rhs) noexcept { return is_negative(rhs) && is_infinite(rhs); }

    bool operator!=(const rational &rhs) const noexcept;
    bool operator<(const rational &rhs) const noexcept;
    bool operator<=(const rational &rhs) const noexcept;
    bool operator==(const rational &rhs) const noexcept;
    bool operator>=(const rational &rhs) const noexcept;
    bool operator>(const rational &rhs) const noexcept;

    bool operator!=(const I &rhs) const noexcept;
    bool operator<(const I &rhs) const noexcept;
    bool operator<=(const I &rhs) const noexcept;
    bool operator==(const I &rhs) const noexcept;
    bool operator>=(const I &rhs) const noexcept;
    bool operator>(const I &rhs) const noexcept;

    rational operator+(const rational &rhs) const noexcept;
    rational operator-(const rational &rhs) const noexcept;
    rational operator*(const rational &rhs) const noexcept;
    rational operator/(const rational &rhs) const noexcept;

    rational operator+(const I &rhs) const noexcept;
    rational operator-(const I &rhs) const noexcept;
    rational operator*(const I &rhs) const noexcept;
    rational operator/(const I &rhs) const noexcept;

    rational &operator+=(const rational &rhs) noexcept;
    rational &operator-=(const rational &rhs) noexcept;
    rational &operator*=(const rational &rhs) noexcept;
    rational &operator/=(const rational &rhs) noexcept;

    rational &operator+=(const I &rhs) noexcept;
    rational &operator-=(const I &rhs) noexcept;
    rational &operator*=(const I &rhs) noexcept;
    rational &operator/=(const I &rhs) noexcept;

    friend rational operator+(const I &lhs, const rational &rhs) noexcept;
    friend rational operator-(const I &lhs, const rational &rhs) noexcept;
    friend rational operator*(const I &lhs, const rational &rhs) noexcept;
    friend rational operator/(const I &lhs, const rational &rhs) noexcept;

    rational operator-() const noexcept;

  private:
    void normalize() noexcept;

    friend std::string to_string(const rational &rhs) noexcept;

  private:
    I num; // the numerator..
    I den; // the denominator..
  };
} // namespace smt