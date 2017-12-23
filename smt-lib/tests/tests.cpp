#include "sat_core.h"
#include "lra_theory.h"
#include <cassert>

using namespace smt;

void test_basic_core()
{
    sat_core core;
    var b0 = core.new_var();
    var b1 = core.new_var();
    var b2 = core.new_var();

    bool nc = core.new_clause({lit(b0, false), !lit(b1), b2});
    assert(nc);
    bool ch = core.check();
    assert(ch);
    assert(core.value(b0) == Undefined);
    assert(core.value(b1) == Undefined);
    assert(core.value(b2) == Undefined);

    bool assm = core.assume(b0) && core.check();
    assert(assm);
    assert(core.value(b0) == True);
    assert(core.value(b1) == Undefined);
    assert(core.value(b2) == Undefined);

    assm = core.assume(b1) && core.check();
    assert(assm);
    assert(core.value(b0) == True);
    assert(core.value(b1) == True);
    assert(core.value(b2) == True);
}

void test_no_good()
{
    sat_core core;

    var b0 = core.new_var();
    var b1 = core.new_var();
    var b2 = core.new_var();
    var b3 = core.new_var();
    var b4 = core.new_var();
    var b5 = core.new_var();
    var b6 = core.new_var();
    var b7 = core.new_var();
    var b8 = core.new_var();

    bool nc = core.new_clause({b0, b1});
    assert(nc);
    nc = core.new_clause({b0, b1, b6});
    assert(nc);
    nc = core.new_clause({lit(b1, false), lit(b2, false), b3});
    assert(nc);
    nc = core.new_clause({lit(b3, false), b4, b7});
    assert(nc);
    nc = core.new_clause({lit(b3, false), b5, b8});
    assert(nc);
    nc = core.new_clause({lit(b4, false), lit(b5, false)});
    assert(nc);

    bool assm = core.assume(lit(b6, false)) && core.check();
    assert(assm);
    assm = core.assume(lit(b7, false)) && core.check();
    assert(assm);
    assm = core.assume(lit(b8, false)) && core.check();
    assert(assm);
    assm = core.assume(lit(b0, false)) && core.check();
    assert(assm);
}

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

void test_lra_theory()
{
    sat_core core;
    lra_theory lra(core);

    var x = lra.new_var();
    var y = lra.new_var();
    var s1 = lra.new_var(lin(x, -1) + lin(y, 1));
    var s2 = lra.new_var(lin(x, 1) + lin(y, 1));

    // x <= -4
    bool nc = core.new_clause({lra.new_leq(lin(x, 1), lin(rational(-4)))}) && core.check();
    assert(nc);
    // x >= -8
    nc = core.new_clause({lra.new_geq(lin(x, 1), lin(rational(-8)))}) && core.check();
    assert(nc);
    // s1 <= 1
    nc = core.new_clause({lra.new_leq(lin(s1, 1), lin(rational(1)))}) && core.check();
    assert(nc);

    // s2 >= -3
    bool assm = core.assume(lra.new_geq(lin(s2, 1), lin(rational(-3))));
    assert(!assm);
}

void test_inequalities()
{
    sat_core core;
    lra_theory lra(core);

    var x = lra.new_var();
    var y = lra.new_var();

    // x >= y;
    bool nc = core.new_clause({lra.new_geq(lin(x, 1), lin(y, 1))}) && core.check();
    assert(nc);

    inf_rational x_val = lra.value(x);
    assert(x_val == rational::ZERO);

    inf_rational y_val = lra.value(y);
    assert(y_val == rational::ZERO);

    // y >= 1
    nc = core.new_clause({lra.new_geq(lin(y, 1), lin(1))}) && core.check();
    assert(nc);

    x_val = lra.value(x);
    assert(x_val == rational::ONE);

    y_val = lra.value(y);
    assert(y_val == rational::ONE);
}

void test_strict_inequalities()
{
    sat_core core;
    lra_theory lra(core);

    var x = lra.new_var();
    var y = lra.new_var();

    // x > y;
    bool nc = core.new_clause({lra.new_gt(lin(x, 1), lin(y, 1))}) && core.check();
    assert(nc);

    inf_rational x_val = lra.value(x);
    assert(x_val == inf_rational(rational::ZERO, rational::ONE));

    inf_rational y_val = lra.value(y);
    assert(y_val == rational::ZERO);

    // y >= 1
    nc = core.new_clause({lra.new_geq(lin(y, 1), lin(1))}) && core.check();
    assert(nc);

    x_val = lra.value(x);
    assert(x_val == inf_rational(rational::ONE, rational::ONE));

    y_val = lra.value(y);
    assert(y_val == rational::ONE);
}

int main(int argc, char *argv[])
{
    test_basic_core();
    test_no_good();

    test_rationals_0();
    test_rationals_1();
    test_lin();

    test_lra_theory();
    test_inequalities();
    test_strict_inequalities();
}