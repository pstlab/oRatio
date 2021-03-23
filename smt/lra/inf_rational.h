#pragma once

#include "rational.h"

namespace smt
{
  class inf_rational
  {
  public:
    explicit inf_rational() {}
    explicit inf_rational(I nun) : rat(nun) {}
    explicit inf_rational(const rational &rat) : rat(rat) {}
    explicit inf_rational(I nun, I den) : rat(nun, den) {}
    explicit inf_rational(const rational &rat, I inf) : rat(rat), inf(inf) {}
    explicit inf_rational(const rational &rat, const rational &inf) : rat(rat), inf(inf) {}

    inline rational get_rational() const noexcept { return rat; }
    inline rational get_infinitesimal() const noexcept { return inf; }

    inline friend bool is_positive(const inf_rational &rhs) noexcept { return is_positive(rhs.rat) || (rhs.rat.numerator() == 0 && is_positive(rhs.rat)); }
    inline friend bool is_negative(const inf_rational &rhs) noexcept { return is_negative(rhs.rat) || (rhs.rat.numerator() == 0 && is_negative(rhs.rat)); }
    inline friend bool is_infinite(const inf_rational &rhs) noexcept { return is_infinite(rhs.rat); }
    inline friend bool is_positive_infinite(const inf_rational &rhs) noexcept { return is_positive(rhs) && is_infinite(rhs); }
    inline friend bool is_negative_infinite(const inf_rational &rhs) noexcept { return is_negative(rhs) && is_infinite(rhs); }

    inline bool operator!=(const inf_rational &rhs) const noexcept { return rat != rhs.rat && inf != rhs.inf; };
    inline bool operator<(const inf_rational &rhs) const noexcept { return rat < rhs.rat || (rat == rhs.rat && inf < rhs.inf); };
    inline bool operator<=(const inf_rational &rhs) const noexcept { return rat < rhs.rat || (rat == rhs.rat && inf <= rhs.inf); };
    inline bool operator==(const inf_rational &rhs) const noexcept { return rat == rhs.rat && inf == rhs.inf; };
    inline bool operator>=(const inf_rational &rhs) const noexcept { return rat > rhs.rat || (rat == rhs.rat && inf >= rhs.inf); };
    inline bool operator>(const inf_rational &rhs) const noexcept { return rat > rhs.rat || (rat == rhs.rat && inf > rhs.inf); };

    inline bool operator!=(const rational &rhs) const noexcept { return rat != rhs || inf.numerator() != 0; };
    inline bool operator<(const rational &rhs) const noexcept { return rat < rhs || (rat == rhs && inf.numerator() < 0); };
    inline bool operator<=(const rational &rhs) const noexcept { return rat < rhs || (rat == rhs && inf.numerator() <= 0); };
    inline bool operator==(const rational &rhs) const noexcept { return rat == rhs && inf.numerator() == 0; };
    inline bool operator>=(const rational &rhs) const noexcept { return rat > rhs || (rat == rhs && inf.numerator() >= 0); };
    inline bool operator>(const rational &rhs) const noexcept { return rat > rhs || (rat == rhs && inf.numerator() > 0); };

    inline bool operator!=(const I &rhs) const noexcept { return rat != rhs || inf.numerator() != 0; };
    inline bool operator<(const I &rhs) const noexcept { return rat < rhs || (rat == rhs && inf.numerator() < 0); };
    inline bool operator<=(const I &rhs) const noexcept { return rat < rhs || (rat == rhs && inf.numerator() <= 0); };
    inline bool operator==(const I &rhs) const noexcept { return rat == rhs && inf.numerator() == 0; };
    inline bool operator>=(const I &rhs) const noexcept { return rat > rhs || (rat == rhs && inf.numerator() >= 0); };
    inline bool operator>(const I &rhs) const noexcept { return rat > rhs || (rat == rhs && inf.numerator() > 0); };

    inline inf_rational operator+(const inf_rational &rhs) const noexcept { return inf_rational(rat + rhs.rat, inf + rhs.inf); };
    inline inf_rational operator-(const inf_rational &rhs) const noexcept { return inf_rational(rat - rhs.rat, inf - rhs.inf); };

    inline inf_rational operator+(const rational &rhs) const noexcept { return inf_rational(rat + rhs, inf); };
    inline inf_rational operator-(const rational &rhs) const noexcept { return inf_rational(rat - rhs, inf); };
    inline inf_rational operator*(const rational &rhs) const noexcept { return inf_rational(rat * rhs, inf * rhs); };
    inline inf_rational operator/(const rational &rhs) const noexcept { return inf_rational(rat / rhs, inf / rhs); };

    inline inf_rational operator+(const I &rhs) const noexcept { return inf_rational(rat + rhs, inf); };
    inline inf_rational operator-(const I &rhs) const noexcept { return inf_rational(rat - rhs, inf); };
    inline inf_rational operator*(const I &rhs) const noexcept { return inf_rational(rat * rhs, inf * rhs); };
    inline inf_rational operator/(const I &rhs) const noexcept { return inf_rational(rat / rhs, inf / rhs); };

    inline inf_rational &operator+=(const inf_rational &rhs) noexcept
    {
      rat += rhs.rat;
      inf += rhs.inf;
      return *this;
    }
    inline inf_rational &operator-=(const inf_rational &rhs) noexcept
    {
      rat -= rhs.rat;
      inf -= rhs.inf;
      return *this;
    }

    inline inf_rational &operator+=(const rational &rhs) noexcept
    {
      rat += rhs;
      return *this;
    }
    inline inf_rational &operator-=(const rational &rhs) noexcept
    {
      rat -= rhs;
      return *this;
    }
    inline inf_rational &operator*=(const rational &rhs) noexcept
    {
      rat *= rhs;
      inf *= rhs;
      return *this;
    }
    inline inf_rational &operator/=(const rational &rhs) noexcept
    {
      rat /= rhs;
      inf /= rhs;
      return *this;
    }

    inline inf_rational &operator+=(const I &rhs) noexcept
    {
      rat += rhs;
      return *this;
    }
    inline inf_rational &operator-=(const I &rhs) noexcept
    {
      rat -= rhs;
      return *this;
    }
    inline inf_rational &operator*=(const I &rhs) noexcept
    {
      rat *= rhs;
      inf *= rhs;
      return *this;
    }
    inline inf_rational &operator/=(const I &rhs) noexcept
    {
      rat /= rhs;
      inf /= rhs;
      return *this;
    }

    inline inf_rational operator-() const noexcept { return inf_rational(-rat, -inf); }

    inline friend inf_rational operator+(const rational &lhs, const inf_rational &rhs) noexcept { return inf_rational(lhs + rhs.rat, rhs.inf); }
    inline friend inf_rational operator-(const rational &lhs, const inf_rational &rhs) noexcept { return inf_rational(lhs - rhs.rat, rhs.inf); }
    inline friend inf_rational operator*(const rational &lhs, const inf_rational &rhs) noexcept { return inf_rational(lhs * rhs.rat, lhs * rhs.inf); }
    inline friend inf_rational operator/(const rational &lhs, const inf_rational &rhs) noexcept { return inf_rational(lhs / rhs.rat, lhs / rhs.inf); }

    inline friend inf_rational operator+(const I &lhs, const inf_rational &rhs) noexcept { return inf_rational(lhs + rhs.rat, rhs.inf); }
    inline friend inf_rational operator-(const I &lhs, const inf_rational &rhs) noexcept { return inf_rational(lhs - rhs.rat, rhs.inf); }
    inline friend inf_rational operator*(const I &lhs, const inf_rational &rhs) noexcept { return inf_rational(lhs * rhs.rat, lhs * rhs.inf); }
    inline friend inf_rational operator/(const I &lhs, const inf_rational &rhs) noexcept { return inf_rational(lhs / rhs.rat, lhs / rhs.inf); }

    friend std::string to_string(const inf_rational &rhs) noexcept
    {
      if (is_infinite(rhs.rat) || rhs.inf == rational::ZERO)
        return to_string(rhs.rat);
      std::string c_str;
      if (rhs.rat != rational::ZERO)
        c_str += to_string(rhs.rat);
      if (rhs.inf == rational::ONE)
        if (c_str.empty())
          c_str += "ε";
        else
          c_str += " + ε";
      else if (rhs.inf == -rational::ONE)
        if (c_str.empty())
          c_str += "-ε";
        else
          c_str += " - ε";
      else if (is_negative(rhs.inf))
        if (c_str.empty())
          c_str += to_string(rhs.inf) + "ε";
        else
          c_str += " " + to_string(rhs.inf) + "ε";
      else if (c_str.empty())
        c_str += to_string(rhs.inf) + "ε";
      else
        c_str += " +" + to_string(rhs.inf) + "ε";
      return c_str;
    };

  private:
    rational rat; // the rational part..
    rational inf; // the infinitesimal part..
  };
} // namespace smt