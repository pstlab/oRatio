#include "rational.h"
#include <numeric>
#include <cassert>

namespace smt
{
    SMT_EXPORT const rational rational::ZERO(0);
    SMT_EXPORT const rational rational::ONE(1);
    SMT_EXPORT const rational rational::NEGATIVE_INFINITY(-1, 0);
    SMT_EXPORT const rational rational::POSITIVE_INFINITY(1, 0);

    SMT_EXPORT rational::rational() : num(0), den(1) {}
    SMT_EXPORT rational::rational(I n) : num(n), den(1) {}
    SMT_EXPORT rational::rational(I n, I d) : num(n), den(d) { normalize(); }

    SMT_EXPORT bool rational::operator!=(const rational &rhs) const noexcept { return num != rhs.num || den != rhs.den; }
    SMT_EXPORT bool rational::operator<(const rational &rhs) const noexcept { return (den == rhs.den) ? num < rhs.num : num * rhs.den < den * rhs.num; }
    SMT_EXPORT bool rational::operator<=(const rational &rhs) const noexcept { return num * rhs.den <= den * rhs.num; }
    SMT_EXPORT bool rational::operator==(const rational &rhs) const noexcept { return num == rhs.num && den == rhs.den; }
    SMT_EXPORT bool rational::operator>=(const rational &rhs) const noexcept { return num * rhs.den >= den * rhs.num; }
    SMT_EXPORT bool rational::operator>(const rational &rhs) const noexcept { return (den == rhs.den) ? num > rhs.num : num * rhs.den > den * rhs.num; }

    SMT_EXPORT bool rational::operator!=(const I &rhs) const noexcept { return num != rhs || den != 1; }
    SMT_EXPORT bool rational::operator<(const I &rhs) const noexcept { return num < den * rhs; }
    SMT_EXPORT bool rational::operator<=(const I &rhs) const noexcept { return num <= den * rhs; }
    SMT_EXPORT bool rational::operator==(const I &rhs) const noexcept { return num == rhs && den == 1; }
    SMT_EXPORT bool rational::operator>=(const I &rhs) const noexcept { return num >= den * rhs; }
    SMT_EXPORT bool rational::operator>(const I &rhs) const noexcept { return num > den * rhs; }

    SMT_EXPORT rational rational::operator+(const rational &rhs) const noexcept
    {
        assert(den != 0 || rhs.den != 0 || num == rhs.num); // inf + -inf or -inf + inf..

        // special cases..
        if (num == 0 || is_infinite(rhs))
            return rhs;
        if (rhs.num == 0 || is_infinite(*this))
            return *this;
        if (den == 1 && rhs.den == 1)
            return rational(num + rhs.num);

        I f = std::gcd(num, rhs.num);
        I g = std::gcd(den, rhs.den);

        rational res((num / f) * (rhs.den / g) + (rhs.num / f) * (den / g), std::lcm(den, rhs.den));
        res.num *= f;
        return res;
    }

    SMT_EXPORT rational rational::operator-(const rational &rhs) const noexcept { return operator+(-rhs); }

    SMT_EXPORT rational rational::operator*(const rational &rhs) const noexcept
    {
        assert(num != 0 || rhs.den != 0); // 0*inf..
        assert(den != 0 || rhs.num != 0); // inf*0..

        // special cases..
        if (rhs == ONE)
            return *this;
        if (operator==(ONE))
            return rhs;
        if (den == 1 && rhs.den == 1)
            return rational(num * rhs.num);
        if (is_infinite(*this) || is_infinite(rhs))
            return ((num >= 0 && rhs.num >= 0) || (num <= 0 && rhs.num <= 0)) ? POSITIVE_INFINITY : NEGATIVE_INFINITY;

        rational c(num, rhs.den);
        rational d(rhs.num, den);
        return rational(c.num * d.num, c.den * d.den);
    }

    SMT_EXPORT rational rational::operator/(const rational &rhs) const noexcept
    {
        rational rec;
        if (rhs.num >= 0)
        {
            rec.num = rhs.den;
            rec.den = rhs.num;
        }
        else
        {
            rec.num = -rhs.den;
            rec.den = -rhs.num;
        }
        return operator*(rec);
    }

    SMT_EXPORT rational rational::operator+(const I &rhs) const noexcept
    {
        // special cases..
        if (num == 0)
            return rational(rhs);
        if (rhs == 0 || is_infinite(*this))
            return *this;
        if (den == 1)
            return rational(num + rhs);

        rational res;
        res.num = num + rhs * den;
        res.den = den;
        return res;
    }

    SMT_EXPORT rational rational::operator-(const I &rhs) const noexcept { return operator+(-rhs); }

    SMT_EXPORT rational rational::operator*(const I &rhs) const noexcept
    {
        assert(den != 0 || rhs != 0); // inf*0..

        // special cases..
        if (rhs == 1)
            return *this;
        if (operator==(ONE))
            return rational(rhs);
        if (den == 1)
            return rational(num * rhs);
        if (is_infinite(*this))
            return ((num >= 0 && rhs >= 0) || (num <= 0 && rhs <= 0)) ? POSITIVE_INFINITY : NEGATIVE_INFINITY;

        return rational(num * rhs, den);
    }

    SMT_EXPORT rational rational::operator/(const I &rhs) const noexcept
    {
        rational rec;
        rec.num = 1;
        rec.den = rhs;
        if (rhs >= 0)
        {
            rec.num = 1;
            rec.den = rhs;
        }
        else
        {
            rec.num = -1;
            rec.den = -rhs;
        }
        return operator*(rec);
    }

    SMT_EXPORT rational &rational::operator+=(const rational &rhs) noexcept
    {
        assert(den != 0 || rhs.den != 0 || num == rhs.num); // inf + -inf or -inf + inf..

        // special cases..
        if (num == 0 || is_infinite(rhs))
        {
            num = rhs.num;
            den = rhs.den;
            return *this;
        }
        if (rhs.num == 0 || is_infinite(*this))
            return *this;
        if (den == 1 && rhs.den == 1)
        {
            num += rhs.num;
            return *this;
        }

        I f = std::gcd(num, rhs.num);
        I g = std::gcd(den, rhs.den);

        num = (num / f) * (rhs.den / g) + (rhs.num / f) * (den / g);
        den = std::lcm(den, rhs.den);
        normalize();
        num *= f;
        return *this;
    }

    SMT_EXPORT rational &rational::operator-=(const rational &rhs) noexcept { return operator+=(-rhs); }

    SMT_EXPORT rational &rational::operator*=(const rational &rhs) noexcept
    {
        assert(num != 0 || rhs.den != 0); // 0*inf..
        assert(den != 0 || rhs.num != 0); // inf*0..

        // special cases..
        if (rhs == ONE)
            return *this;
        if (operator==(ONE))
        {
            num = rhs.num;
            den = rhs.den;
            return *this;
        }
        if (den == 1 && rhs.den == 1)
        {
            num *= rhs.num;
            return *this;
        }
        if (is_infinite(*this) || is_infinite(rhs))
        {
            num = ((num >= 0 && rhs.num >= 0) || (num <= 0 && rhs.num <= 0)) ? 1 : -1;
            den = 0;
            return *this;
        }

        rational c(num, rhs.den);
        rational d(rhs.num, den);

        num = c.num * d.num;
        den = c.den * d.den;
        normalize();
        return *this;
    }

    SMT_EXPORT rational &rational::operator/=(const rational &rhs) noexcept
    {
        rational rec;
        rec.num = rhs.den;
        rec.den = rhs.num;
        if (rhs.num >= 0)
        {
            rec.num = rhs.den;
            rec.den = rhs.num;
        }
        else
        {
            rec.num = -rhs.den;
            rec.den = -rhs.num;
        }
        return operator*=(rec);
    }

    SMT_EXPORT rational &rational::operator+=(const I &rhs) noexcept
    {
        // special cases..
        if (num == 0)
        {
            num = rhs;
            return *this;
        }
        if (rhs == 0 || is_infinite(*this))
            return *this;
        if (den == 1)
        {
            num += rhs;
            return *this;
        }

        num += rhs * den;
        return *this;
    }

    SMT_EXPORT rational &rational::operator-=(const I &rhs) noexcept { return operator+=(-rhs); }

    SMT_EXPORT rational &rational::operator*=(const I &rhs) noexcept
    {
        assert(den != 0 || rhs != 0); // inf*0..

        // special cases..
        if (rhs == 1)
            return *this;
        if (operator==(ONE))
        {
            num = rhs;
            return *this;
        }
        if (is_infinite(*this))
        {
            num = ((num >= 0 && rhs >= 0) || (num <= 0 && rhs <= 0)) ? 1 : -1;
            return *this;
        }

        num *= rhs;
        if (den != 1)
            normalize();

        return *this;
    }

    SMT_EXPORT rational &rational::operator/=(const I &rhs) noexcept
    {
        rational rec;
        rec.num = 1;
        rec.den = rhs;
        if (rhs >= 0)
        {
            rec.num = 1;
            rec.den = rhs;
        }
        else
        {
            rec.num = -1;
            rec.den = -rhs;
        }
        return operator*=(rec);
    }

    SMT_EXPORT rational operator+(const I &lhs, const rational &rhs) noexcept { return rational(lhs) + rhs; }
    SMT_EXPORT rational operator-(const I &lhs, const rational &rhs) noexcept { return rational(lhs) - rhs; }
    SMT_EXPORT rational operator*(const I &lhs, const rational &rhs) noexcept { return rational(lhs) * rhs; }
    SMT_EXPORT rational operator/(const I &lhs, const rational &rhs) noexcept { return rational(lhs) / rhs; }

    SMT_EXPORT rational rational::operator-() const noexcept
    {
        rational res(*this);
        res.num = -res.num;
        return res;
    }

    void rational::normalize() noexcept
    {
        if (den != 1)
        {
            I c_gcd = std::gcd(num, den);
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

    SMT_EXPORT std::string to_string(const rational &rhs) noexcept
    {
        switch (rhs.den)
        {
        case 0:
            return rhs.num > 0 ? "+inf" : "-inf";
        case 1:
            return std::to_string(rhs.num);
        default:
            return std::to_string(rhs.num) + "/" + std::to_string(rhs.den);
        }
    }
} // namespace smt