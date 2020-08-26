#include "sat_core.h"
#include "lra_theory.h"
#include <cassert>

using namespace smt;

void test_rationals_0()
{
    rational r0;
    assert(r0.numerator() == 0);
    assert(r0.denominator() == 1);
    assert(r0 == rational::ZERO);

    r0 += 1;
    assert(r0.numerator() == 1);
    assert(r0.denominator() == 1);
    assert(r0 == rational::ONE);

    r0 += rational(1, 2);
    assert(r0.numerator() == 3);
    assert(r0.denominator() == 2);

    r0 += rational::NEGATIVE_INFINITY;
    assert(r0.numerator() == -1);
    assert(r0.denominator() == 0);
    assert(r0 == rational::NEGATIVE_INFINITY);

    rational r1(4, 2);
    assert(r1.numerator() == 2);
    assert(r1.denominator() == 1);

    rational r2 = r1 / r0;
    assert(r2.numerator() == 0);
    assert(r2.denominator() == 1);
    assert(r2 == rational::ZERO);

    r2 += 2;
    assert(r2.numerator() == 2);
    assert(r2.denominator() == 1);

    r2 -= -2;
    assert(r2.numerator() == 4);
    assert(r2.denominator() == 1);

    r2 *= 2;
    assert(r2.numerator() == 8);
    assert(r2.denominator() == 1);
}

void test_rationals_1()
{
    assert(rational::NEGATIVE_INFINITY < rational::POSITIVE_INFINITY);
    assert(rational::NEGATIVE_INFINITY <= rational::POSITIVE_INFINITY);
    assert(rational::POSITIVE_INFINITY >= rational::NEGATIVE_INFINITY);
    assert(rational::POSITIVE_INFINITY > rational::NEGATIVE_INFINITY);

    inf_rational eps(rational::ZERO, rational::ONE);
    assert(!(eps <= 0));
    assert(eps <= inf_rational(rational::ZERO, rational::ONE));
    assert(eps > 0);
}

void test_lin()
{
    lin l0;
    l0 += lin(0, 1);
    l0 += lin(1, 2);

    lin l1;
    l1 += lin(0, 1);
    l1 += lin(1, 2);

    lin l2 = l0 + l1;
    assert(l2.vars.at(0) == rational(2));
    assert(l2.vars.at(1) == rational(4));
}

int main(int, char **)
{
    test_rationals_0();
    test_rationals_1();
    test_lin();
}