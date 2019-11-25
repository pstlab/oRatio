#include "rational.h"
#include <cassert>

namespace smt
{
const rational rational::ZERO(0);
const rational rational::ONE(1);
const rational rational::NEGATIVE_INFINITY(-1, 0);
const rational rational::POSITIVE_INFINITY(1, 0);

rational::rational() : num(0), den(1) {}
rational::rational(I n) : num(n), den(1) {}
rational::rational(I n, I d) : num(n), den(d) { normalize(); }

bool rational::operator!=(const rational &rhs) const { return num != rhs.num || den != rhs.den; }
bool rational::operator<(const rational &rhs) const { return (den == rhs.den) ? num < rhs.num : num * rhs.den < den * rhs.num; }
bool rational::operator<=(const rational &rhs) const { return num * rhs.den <= den * rhs.num; }
bool rational::operator==(const rational &rhs) const { return num == rhs.num && den == rhs.den; }
bool rational::operator>=(const rational &rhs) const { return num * rhs.den >= den * rhs.num; }
bool rational::operator>(const rational &rhs) const { return (den == rhs.den) ? num > rhs.num : num * rhs.den > den * rhs.num; }

bool rational::operator!=(const I &rhs) const { return num != rhs || den != 1; }
bool rational::operator<(const I &rhs) const { return num < den * rhs; }
bool rational::operator<=(const I &rhs) const { return num <= den * rhs; }
bool rational::operator==(const I &rhs) const { return num == rhs && den == 1; }
bool rational::operator>=(const I &rhs) const { return num >= den * rhs; }
bool rational::operator>(const I &rhs) const { return num > den * rhs; }

rational rational::operator+(const rational &rhs) const
{
    assert(den != 0 || rhs.den != 0 || num == rhs.num); // inf + -inf or -inf + inf..

    // special cases..
    if (num == 0 || rhs.is_infinite())
        return rhs;
    if (rhs.num == 0 || is_infinite())
        return *this;
    if (den == 1 && rhs.den == 1)
        return num + rhs.num;

    I f = gcd(num, rhs.num);
    I g = gcd(den, rhs.den);

    rational res((num / f) * (rhs.den / g) + (rhs.num / f) * (den / g), lcm(den, rhs.den));
    res.num *= f;
    return res;
}

rational rational::operator-(const rational &rhs) const { return operator+(-rhs); }

rational rational::operator*(const rational &rhs) const
{
    assert(num != 0 || rhs.den != 0); // 0*inf..
    assert(den != 0 || rhs.num != 0); // inf*0..

    // special cases..
    if (rhs == ONE)
        return *this;
    if (operator==(ONE))
        return rhs;
    if (den == 1 && rhs.den == 1)
        return num * rhs.num;
    if (is_infinite() || rhs.is_infinite())
        return ((num >= 0 && rhs.num >= 0) || (num <= 0 && rhs.num <= 0)) ? POSITIVE_INFINITY : NEGATIVE_INFINITY;

    rational c(num, rhs.den);
    rational d(rhs.num, den);
    return rational(c.num * d.num, c.den * d.den);
}

rational rational::operator/(const rational &rhs) const
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

rational rational::operator+(const I &rhs) const
{
    // special cases..
    if (num == 0)
        return rhs;
    if (rhs == 0 || is_infinite())
        return *this;
    if (den == 1)
        return num + rhs;

    rational res;
    res.num = num + rhs * den;
    res.den = den;
    return res;
}

rational rational::operator-(const I &rhs) const { return operator+(-rhs); }

rational rational::operator*(const I &rhs) const
{
    assert(den != 0 || rhs != 0); // inf*0..

    // special cases..
    if (rhs == 1)
        return *this;
    if (operator==(ONE))
        return rhs;
    if (den == 1)
        return num * rhs;
    if (is_infinite())
        return ((num >= 0 && rhs >= 0) || (num <= 0 && rhs <= 0)) ? POSITIVE_INFINITY : NEGATIVE_INFINITY;

    return rational(num * rhs, den);
}

rational rational::operator/(const I &rhs) const
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

rational &rational::operator+=(const rational &rhs)
{
    assert(den != 0 || rhs.den != 0 || num == rhs.num); // inf + -inf or -inf + inf..

    // special cases..
    if (num == 0 || rhs.is_infinite())
    {
        num = rhs.num;
        den = rhs.den;
        return *this;
    }
    if (rhs.num == 0 || is_infinite())
        return *this;
    if (den == 1 && rhs.den == 1)
    {
        num += rhs.num;
        return *this;
    }

    I f = gcd(num, rhs.num);
    I g = gcd(den, rhs.den);

    num = (num / f) * (rhs.den / g) + (rhs.num / f) * (den / g);
    den = lcm(den, rhs.den);
    normalize();
    num *= f;
    return *this;
}

rational &rational::operator-=(const rational &rhs) { return operator+=(-rhs); }

rational &rational::operator*=(const rational &rhs)
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
    if (is_infinite() || rhs.is_infinite())
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

rational &rational::operator/=(const rational &rhs)
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

rational &rational::operator+=(const I &rhs)
{
    // special cases..
    if (num == 0)
    {
        num = rhs;
        return *this;
    }
    if (rhs == 0 || is_infinite())
        return *this;
    if (den == 1)
    {
        num += rhs;
        return *this;
    }

    num += rhs * den;
    return *this;
}

rational &rational::operator-=(const I &rhs) { return operator+=(-rhs); }

rational &rational::operator*=(const I &rhs)
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
    if (is_infinite())
    {
        num = ((num >= 0 && rhs >= 0) || (num <= 0 && rhs <= 0)) ? 1 : -1;
        return *this;
    }

    num *= rhs;
    if (den != 1)
        normalize();

    return *this;
}

rational &rational::operator/=(const I &rhs)
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

rational operator+(const I &lhs, const rational &rhs) { return rational(lhs) + rhs; }
rational operator-(const I &lhs, const rational &rhs) { return rational(lhs) - rhs; }
rational operator*(const I &lhs, const rational &rhs) { return rational(lhs) * rhs; }
rational operator/(const I &lhs, const rational &rhs) { return rational(lhs) / rhs; }

rational rational::operator-() const
{
    rational res(*this);
    res.num = -res.num;
    return res;
}

std::string rational::to_string() const
{
    switch (den)
    {
    case 0:
        return num > 0 ? "+inf" : "-inf";
    case 1:
        return std::to_string(num);
    default:
        return std::to_string(num) + "/" + std::to_string(den);
    }
}
} // namespace smt