#pragma once

#include "rational.h"

namespace smt
{

  class inf_rational
  {
  public:
    inf_rational() {}
    inf_rational(I nun) : rat(nun) {}
    inf_rational(const rational &rat) : rat(rat) {}
    inf_rational(I nun, I den) : rat(nun, den) {}
    inf_rational(const rational &rat, I inf) : rat(rat), inf(inf) {}
    inf_rational(const rational &rat, const rational &inf) : rat(rat), inf(inf) {}

    rational get_rational() const { return rat; }
    rational get_infinitesimal() const { return inf; }

    friend bool is_positive(const inf_rational &rhs) { return is_positive(rhs.rat) || (rhs.rat.numerator() == 0 && is_positive(rhs.rat)); }
    friend bool is_negative(const inf_rational &rhs) { return is_negative(rhs.rat) || (rhs.rat.numerator() == 0 && is_negative(rhs.rat)); }
    friend bool is_infinite(const inf_rational &rhs) { return is_infinite(rhs.rat); }
    friend bool is_positive_infinite(const inf_rational &rhs) { return is_positive(rhs) && is_infinite(rhs); }
    friend bool is_negative_infinite(const inf_rational &rhs) { return is_negative(rhs) && is_infinite(rhs); }

    bool operator!=(const inf_rational &rhs) const { return rat != rhs.rat && inf != rhs.inf; };
    bool operator<(const inf_rational &rhs) const { return rat < rhs.rat || (rat == rhs.rat && inf < rhs.inf); };
    bool operator<=(const inf_rational &rhs) const { return rat < rhs.rat || (rat == rhs.rat && inf <= rhs.inf); };
    bool operator==(const inf_rational &rhs) const { return rat == rhs.rat && inf == rhs.inf; };
    bool operator>=(const inf_rational &rhs) const { return rat > rhs.rat || (rat == rhs.rat && inf >= rhs.inf); };
    bool operator>(const inf_rational &rhs) const { return rat > rhs.rat || (rat == rhs.rat && inf > rhs.inf); };

    bool operator!=(const rational &rhs) const { return rat != rhs || inf.numerator() != 0; };
    bool operator<(const rational &rhs) const { return rat < rhs || (rat == rhs && inf.numerator() < 0); };
    bool operator<=(const rational &rhs) const { return rat < rhs || (rat == rhs && inf.numerator() <= 0); };
    bool operator==(const rational &rhs) const { return rat == rhs && inf.numerator() == 0; };
    bool operator>=(const rational &rhs) const { return rat > rhs || (rat == rhs && inf.numerator() >= 0); };
    bool operator>(const rational &rhs) const { return rat > rhs || (rat == rhs && inf.numerator() > 0); };

    bool operator!=(const I &rhs) const { return rat != rhs || inf.numerator() != 0; };
    bool operator<(const I &rhs) const { return rat < rhs || (rat == rhs && inf.numerator() < 0); };
    bool operator<=(const I &rhs) const { return rat < rhs || (rat == rhs && inf.numerator() <= 0); };
    bool operator==(const I &rhs) const { return rat == rhs && inf.numerator() == 0; };
    bool operator>=(const I &rhs) const { return rat > rhs || (rat == rhs && inf.numerator() >= 0); };
    bool operator>(const I &rhs) const { return rat > rhs || (rat == rhs && inf.numerator() > 0); };

    inf_rational operator+(const inf_rational &rhs) const { return inf_rational(rat + rhs.rat, inf + rhs.inf); };
    inf_rational operator-(const inf_rational &rhs) const { return inf_rational(rat - rhs.rat, inf - rhs.inf); };

    inf_rational operator+(const rational &rhs) const { return inf_rational(rat + rhs, inf); };
    inf_rational operator-(const rational &rhs) const { return inf_rational(rat - rhs, inf); };
    inf_rational operator*(const rational &rhs) const { return inf_rational(rat * rhs, inf * rhs); };
    inf_rational operator/(const rational &rhs) const { return inf_rational(rat / rhs, inf / rhs); };

    inf_rational operator+(const I &rhs) const { return inf_rational(rat + rhs, inf); };
    inf_rational operator-(const I &rhs) const { return inf_rational(rat - rhs, inf); };
    inf_rational operator*(const I &rhs) const { return inf_rational(rat * rhs, inf * rhs); };
    inf_rational operator/(const I &rhs) const { return inf_rational(rat / rhs, inf / rhs); };

    inf_rational &operator+=(const inf_rational &rhs)
    {
      rat += rhs.rat;
      inf += rhs.inf;
      return *this;
    }
    inf_rational &operator-=(const inf_rational &rhs)
    {
      rat -= rhs.rat;
      inf -= rhs.inf;
      return *this;
    }

    inf_rational &operator+=(const rational &rhs)
    {
      rat += rhs;
      return *this;
    }
    inf_rational &operator-=(const rational &rhs)
    {
      rat -= rhs;
      return *this;
    }
    inf_rational &operator*=(const rational &rhs)
    {
      rat *= rhs;
      inf *= rhs;
      return *this;
    }
    inf_rational &operator/=(const rational &rhs)
    {
      rat /= rhs;
      inf /= rhs;
      return *this;
    }

    inf_rational &operator+=(const I &rhs)
    {
      rat += rhs;
      return *this;
    }
    inf_rational &operator-=(const I &rhs)
    {
      rat -= rhs;
      return *this;
    }
    inf_rational &operator*=(const I &rhs)
    {
      rat *= rhs;
      inf *= rhs;
      return *this;
    }
    inf_rational &operator/=(const I &rhs)
    {
      rat /= rhs;
      inf /= rhs;
      return *this;
    }

    inf_rational operator-() const { return inf_rational(-rat, -inf); }

    friend inf_rational operator+(const rational &lhs, const inf_rational &rhs) { return inf_rational(lhs + rhs.rat, rhs.inf); }
    friend inf_rational operator-(const rational &lhs, const inf_rational &rhs) { return inf_rational(lhs - rhs.rat, rhs.inf); }
    friend inf_rational operator*(const rational &lhs, const inf_rational &rhs) { return inf_rational(lhs * rhs.rat, lhs * rhs.inf); }
    friend inf_rational operator/(const rational &lhs, const inf_rational &rhs) { return inf_rational(lhs / rhs.rat, lhs / rhs.inf); }

    friend inf_rational operator+(const I &lhs, const inf_rational &rhs) { return inf_rational(lhs + rhs.rat, rhs.inf); }
    friend inf_rational operator-(const I &lhs, const inf_rational &rhs) { return inf_rational(lhs - rhs.rat, rhs.inf); }
    friend inf_rational operator*(const I &lhs, const inf_rational &rhs) { return inf_rational(lhs * rhs.rat, lhs * rhs.inf); }
    friend inf_rational operator/(const I &lhs, const inf_rational &rhs) { return inf_rational(lhs / rhs.rat, lhs / rhs.inf); }

    std::string to_string() const
    {
      if (is_infinite(rat) || inf == rational::ZERO)
        return rat.to_string();
      std::string c_str;
      if (rat != rational::ZERO)
        c_str += rat.to_string();
      if (inf == rational::ONE)
        if (c_str.empty())
          c_str += "ε";
        else
          c_str += " + ε";
      else if (inf == -rational::ONE)
        if (c_str.empty())
          c_str += "-ε";
        else
          c_str += " - ε";
      else if (is_negative(inf))
        if (c_str.empty())
          c_str += inf.to_string() + "ε";
        else
          c_str += " " + inf.to_string() + "ε";
      else if (c_str.empty())
        c_str += inf.to_string() + "ε";
      else
        c_str += " +" + inf.to_string() + "ε";
      return c_str;
    };

  private:
    rational rat; // the rational part..
    rational inf; // the infinitesimal part..
  };
} // namespace smt