#include "rational.h"

namespace smt
{
const rational rational::ZERO(0);
const rational rational::ONE(1);
const rational rational::NEGATIVE_INFINITY(-1, 0);
const rational rational::POSITIVE_INFINITY(1, 0);

rational::rational() : num(0), den(1) {}
rational::rational(I n) : num(n), den(1) {}
rational::rational(I n, I d) : num(n), den(d) { normalize(); }
}