#pragma once

#include <string>

namespace smt
{

typedef long I;

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

  bool is_positive() const { return num > 0; }
  bool is_negative() const { return num < 0; }
  bool is_infinite() const { return den == 0; }
  bool is_positive_infinite() const { return is_positive() && is_infinite(); }
  bool is_negative_infinite() const { return is_negative() && is_infinite(); }

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

  static I gcd(I u, I v) // computes the greatest common divisor..
  {
    if (u < 0)
      u = -u;
    if (v < 0)
      v = -v;
    while (v != 0)
    {
      I r = u % v;
      u = v;
      v = r;
    }
    return u;
  }

  static I lcm(I u, I v) // computes the least common multiplier..
  {
    if (u < 0)
      u = -u;
    if (v < 0)
      v = -v;
    return u * (v / gcd(u, v));
  }

  std::string to_string() const;

private:
  void normalize()
  {
    if (den != 1)
    {
      I c_gcd = gcd(num, den);
      if (den < 0)
        c_gcd = -c_gcd;
      num /= c_gcd;
      den /= c_gcd;
    }
    if (den < 0)
    {
      den = -den;
      num = -num;
    }
  }

private:
  I num; // the numerator..
  I den; // the denominator..
};
}