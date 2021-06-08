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
    l0 += lin(0, rational::ONE);
    l0 += lin(1, rational(2));

    lin l1;
    l1 += lin(0, rational::ONE);
    l1 += lin(1, rational(2));

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
    var s1 = lra.new_var(lin(x, -rational::ONE) + lin(y, rational::ONE));
    var s2 = lra.new_var(lin(x, rational::ONE) + lin(y, rational::ONE));

    // x <= -4
    bool nc = core.new_clause({lra.new_leq(lin(x, rational::ONE), lin(rational(-4)))});
    assert(nc);
    // x >= -8
    nc = core.new_clause({lra.new_geq(lin(x, rational::ONE), lin(-rational(8)))});
    assert(nc);
    // s1 <= 1
    nc = core.new_clause({lra.new_leq(lin(s1, rational::ONE), lin(rational::ONE))});
    assert(nc);

    bool prop = core.propagate();
    assert(prop);

    // s2 >= -3
    bool assm = core.assume(lra.new_geq(lin(s2, rational::ONE), lin(-rational(3))));
    assert(!assm);
}

void test_inequalities_0()
{
    sat_core core;
    lra_theory lra(core);

    var x = lra.new_var();
    var y = lra.new_var();

    // x >= y
    bool nc = core.new_clause({lra.new_geq(lin(x, rational::ONE), lin(y, rational::ONE))});
    assert(nc);

    inf_rational x_val = lra.value(x);
    assert(x_val == rational::ZERO);

    inf_rational y_val = lra.value(y);
    assert(y_val == rational::ZERO);

    // y >= 1
    nc = core.new_clause({lra.new_geq(lin(y, rational::ONE), lin(rational::ONE))});
    assert(nc);

    bool prop = core.propagate();
    assert(prop);

    x_val = lra.value(x);
    assert(x_val == rational::ONE);

    y_val = lra.value(y);
    assert(y_val == rational::ONE);
}

void test_inequalities_1()
{
    sat_core core;
    lra_theory lra(core);

    var x = lra.new_var();
    var y = lra.new_var();

    // x >= y
    bool nc = core.new_clause({lra.new_geq(lin(x, rational::ONE), lin(y, rational::ONE))});
    assert(nc);

    bool prop = core.propagate();
    assert(prop);

    inf_rational x_val = lra.value(x);
    assert(x_val == rational::ZERO);

    inf_rational y_val = lra.value(y);
    assert(y_val == rational::ZERO);

    // y >= 1
    nc = core.new_clause({lra.new_geq(lin(y, rational::ONE), lin(rational::ONE))});
    assert(nc);

    prop = core.propagate();
    assert(prop);

    x_val = lra.value(x);
    assert(x_val == rational::ONE);

    y_val = lra.value(y);
    assert(y_val == rational::ONE);
}

void test_strict_inequalities_0()
{
    sat_core core;
    lra_theory lra(core);

    var x = lra.new_var();
    var y = lra.new_var();

    // x > y
    bool nc = core.new_clause({lra.new_gt(lin(x, rational::ONE), lin(y, rational::ONE))});
    assert(nc);

    bool prop = core.propagate();
    assert(prop);

    inf_rational x_val = lra.value(x);
    assert(x_val == inf_rational(rational::ZERO, rational::ONE));

    inf_rational y_val = lra.value(y);
    assert(y_val == rational::ZERO);

    // y >= 1
    nc = core.new_clause({lra.new_geq(lin(y, rational::ONE), lin(rational::ONE))});
    assert(nc);

    prop = core.propagate();
    assert(prop);

    x_val = lra.value(x);
    assert(x_val == inf_rational(rational::ONE, rational::ONE));

    y_val = lra.value(y);
    assert(y_val == rational::ONE);
}

void test_strict_inequalities_1()
{
    sat_core core;
    lra_theory lra(core);

    var x = lra.new_var();
    var y = lra.new_var();

    // ![x >= y] --> x < y
    bool nc = core.new_clause({!lra.new_geq(lin(x, rational::ONE), lin(y, rational::ONE))});
    assert(nc);

    bool prop = core.propagate();
    assert(prop);

    inf_rational x_val = lra.value(x);
    assert(x_val == inf_rational(rational::ZERO, -rational::ONE));

    inf_rational y_val = lra.value(y);
    assert(y_val == rational::ZERO);

    // x >= 1
    nc = core.new_clause({lra.new_geq(lin(x, rational::ONE), lin(rational::ONE))});
    assert(nc);

    prop = core.propagate();
    assert(prop);

    x_val = lra.value(x);
    assert(x_val == rational::ONE);

    y_val = lra.value(y);
    assert(y_val == inf_rational(rational::ONE, rational::ONE));
}

void test_nonroot_constraints()
{
    sat_core core;
    lra_theory lra(core);

    var x = lra.new_var();
    var y = lra.new_var();

    lit x_leq_y = lra.new_leq(lin(x, rational::ONE), lin(y, rational::ONE));
    lit y_leq_x = lra.new_leq(lin(y, rational::ONE), lin(x, rational::ONE));

    bool nc = core.new_clause({lra.new_leq(lin(y, rational::ONE), lin(rational::ONE))});
    assert(nc);

    bool prop = core.propagate();
    assert(prop);

    nc = core.new_clause({x_leq_y, y_leq_x});
    assert(nc);

    bool assm = core.assume({x_leq_y});
    assert(assm);

    assert(core.value(x_leq_y) == True);

    prop = lra.set_lb(x, inf_rational(rational::ONE), TRUE_lit);
    assert(prop);

    prop = lra.set_lb(x, inf_rational(rational(2)), TRUE_lit);
    assert(!prop);
}

int main(int, char **)
{
    test_rationals_0();
    test_rationals_1();
    test_lin();

    test_lra_theory();
    test_inequalities_0();
    test_inequalities_1();

    test_strict_inequalities_0();
    test_strict_inequalities_1();

    test_nonroot_constraints();
}