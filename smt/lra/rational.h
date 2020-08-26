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

    operator double() const { return static_cast<double>(num) / den; }
    I numerator() const { return num; }
    I denominator() const { return den; }

    friend bool is_integer(const rational &rhs) { return rhs.den == 1; }
    friend bool is_positive(const rational &rhs) { return rhs.num > 0; }
    friend bool is_negative(const rational &rhs) { return rhs.num < 0; }
    friend bool is_infinite(const rational &rhs) { return rhs.den == 0; }
    friend bool is_positive_infinite(const rational &rhs) { return is_positive(rhs) && is_infinite(rhs); }
    friend bool is_negative_infinite(const rational &rhs) { return is_negative(rhs) && is_infinite(rhs); }

    bool operator!=(const rational &rhs) const;
    bool operator<(const rational &rhs) const;
    bool operator<=(const rational &rhs) const;
    bool operator==(const rational &rhs) const;
    bool operator>=(const rational &rhs) const;
    bool operator>(const rational &rhs) const;

    bool operator!=(const I &rhs) const;
    bool operator<(const I &rhs) const;
    bool operator<=(const I &rhs) const;
    bool operator==(const I &rhs) const;
    bool operator>=(const I &rhs) const;
    bool operator>(const I &rhs) const;

    rational operator+(const rational &rhs) const;
    rational operator-(const rational &rhs) const;
    rational operator*(const rational &rhs) const;
    rational operator/(const rational &rhs) const;

    rational operator+(const I &rhs) const;
    rational operator-(const I &rhs) const;
    rational operator*(const I &rhs) const;
    rational operator/(const I &rhs) const;

    rational &operator+=(const rational &rhs);
    rational &operator-=(const rational &rhs);
    rational &operator*=(const rational &rhs);
    rational &operator/=(const rational &rhs);

    rational &operator+=(const I &rhs);
    rational &operator-=(const I &rhs);
    rational &operator*=(const I &rhs);
    rational &operator/=(const I &rhs);

    friend rational operator+(const I &lhs, const rational &rhs);
    friend rational operator-(const I &lhs, const rational &rhs);
    friend rational operator*(const I &lhs, const rational &rhs);
    friend rational operator/(const I &lhs, const rational &rhs);

    rational operator-() const;

  private:
    void normalize();

  public:
    std::string to_string() const;

  private:
    I num; // the numerator..
    I den; // the denominator..
  };
} // namespace smt