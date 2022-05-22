#include "sat_core.h"
#include "ov_theory.h"
#include <cassert>

using namespace smt;

class test_val : public var_value
{
};

void test_ov_0()
{
    sat_core core;
    ov_theory ov(core);

    test_val a;
    test_val b;
    test_val c;

    var v0 = ov.new_var({&a, &b, &c});
    var v1 = ov.new_var({&a, &b});

    // v0 == v1
    bool nc = core.new_clause({ov.new_eq(v0, v1)});
    assert(nc);

    bool prop = core.propagate();
    assert(prop);

    assert(core.value(ov.allows(v0, a)) == Undefined);
    assert(core.value(ov.allows(v0, b)) == Undefined);
    assert(core.value(ov.allows(v0, c)) == False);

    bool assm = core.assume(ov.allows(v0, a));
    assert(assm);
    assert(core.value(ov.allows(v0, a)) == True);
    assert(core.value(ov.allows(v0, b)) == False);
    assert(core.value(ov.allows(v0, c)) == False);
    assert(core.value(ov.allows(v1, a)) == True);
    assert(core.value(ov.allows(v1, b)) == False);
}

void test_ov_1()
{
    sat_core core;
    ov_theory ov(core);

    test_val a;
    test_val b;
    test_val c;

    var v0 = ov.new_var({&a, &b, &c});
    var v1 = ov.new_var({&a, &b});

    // v0 == v1
    bool nc = core.new_clause({ov.new_eq(v0, v1)});
    assert(nc);

    bool prop = core.propagate();
    assert(prop);

    assert(core.value(ov.allows(v0, a)) == Undefined);
    assert(core.value(ov.allows(v0, b)) == Undefined);
    assert(core.value(ov.allows(v0, c)) == False);

    bool assm = core.assume(!ov.allows(v0, a));
    assert(assm);
    assert(core.value(ov.allows(v0, a)) == False);
    assert(core.value(ov.allows(v0, b)) == True);
    assert(core.value(ov.allows(v0, c)) == False);
    assert(core.value(ov.allows(v1, a)) == False);
    assert(core.value(ov.allows(v1, b)) == True);
}

void test_ov_2()
{
    sat_core core;
    ov_theory ov(core);

    test_val a;
    test_val b;
    test_val c;

    var v0 = ov.new_var({&a, &b, &c});
    var v1 = ov.new_var({&a, &b});

    // v0 == v1
    lit eq0 = ov.new_eq(v0, v1);

    bool prop = core.propagate();
    assert(prop);

    // enforcing the [v0 != v1] constraint has no effect on the allowed values of the variables..
    bool assm = core.assume(!eq0);
    assert(assm);
    assert(core.value(eq0) == False);
    assert(core.value(ov.allows(v0, a)) == Undefined);
    assert(core.value(ov.allows(v0, b)) == Undefined);
    assert(core.value(ov.allows(v0, c)) == Undefined);
    assert(core.value(ov.allows(v1, a)) == Undefined);
    assert(core.value(ov.allows(v1, b)) == Undefined);

    // a new [v0 == v1] constraint is controlled by the same literal..
    lit eq1 = ov.new_eq(v0, v1);
    assert(eq0 == eq1);

    // removing the 'a' value from the 'v1' variable, however, results in the removal of the 'b' value from the 'v0' variable..
    assm = core.assume(!ov.allows(v1, a));
    assert(assm);
    assert(core.value(ov.allows(v0, a)) == Undefined);
    assert(core.value(ov.allows(v0, b)) == False);
    assert(core.value(ov.allows(v0, c)) == Undefined);
    assert(core.value(ov.allows(v1, a)) == False);
    assert(core.value(ov.allows(v1, b)) == True);

    // a new [v0 == v1] constraint is still controlled by the same literal..
    lit eq2 = ov.new_eq(v0, v1);
    assert(eq0 == eq2);
}

int main(int, char **)
{
    test_ov_0();
    test_ov_1();
    test_ov_2();
}