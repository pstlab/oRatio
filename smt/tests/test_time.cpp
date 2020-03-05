#include "idl_theory.h"
#include "rdl_theory.h"
#include <cassert>

using namespace smt;

void test_integer_distance_logic()
{
    sat_core core;
    idl_theory idl(core, 5);
    var origin = idl.new_var();
    var horizon = idl.new_var();
    bool nd = idl.distance(horizon, origin, 0);
    assert(nd);

    var tp0 = idl.new_var();
    nd = idl.distance(tp0, origin, 0);
    assert(nd);
    nd = idl.distance(horizon, tp0, 0);
    assert(nd);

    var tp1 = idl.new_var();
    nd = idl.distance(tp1, origin, 0);
    assert(nd);
    nd = idl.distance(horizon, tp1, 0);
    assert(nd);

    var tp2 = idl.new_var();
    nd = idl.distance(tp2, origin, 0);
    assert(nd);
    nd = idl.distance(horizon, tp2, 0);
    assert(nd);

    nd = idl.distance(tp0, tp1, 10);
    assert(nd);
    nd = idl.distance(tp1, tp0, 0);
    assert(nd);

    nd = idl.distance(tp1, tp2, 10);
    assert(nd);
    nd = idl.distance(tp2, tp1, 0);
    assert(nd);

    nd = idl.distance(origin, tp0, 10);
    assert(nd);
    nd = idl.distance(tp0, origin, 0);
    assert(nd);

    var tp2_after_40 = idl.new_distance(tp2, origin, -40);
    assert(tp2_after_40 == FALSE_var);

    var tp2_after_20 = idl.new_distance(tp2, origin, -20);
    var tp2_before_20 = idl.new_distance(origin, tp2, 20);

    var tp2_after_30 = idl.new_distance(tp2, origin, -30);
    var tp2_before_30 = idl.new_distance(origin, tp2, 30);

    nd = core.disj({core.new_conj({tp2_after_20, tp2_before_20}), core.new_conj({tp2_after_30, tp2_before_30})});
    assert(nd);

    nd = idl.distance(origin, tp2, 30);
    assert(nd);
    nd = idl.distance(tp2, origin, -25);
    assert(nd);

    bool ch = core.check();
    assert(ch);
}

void test_real_distance_logic()
{
    sat_core core;
    rdl_theory rdl(core, 5);
    var origin = rdl.new_var();
    var horizon = rdl.new_var();
    bool nd = rdl.distance(horizon, origin, 0);
    assert(nd);

    var tp0 = rdl.new_var();
    nd = rdl.distance(tp0, origin, 0);
    assert(nd);
    nd = rdl.distance(horizon, tp0, 0);
    assert(nd);

    var tp1 = rdl.new_var();
    nd = rdl.distance(tp1, origin, 0);
    assert(nd);
    nd = rdl.distance(horizon, tp1, 0);
    assert(nd);

    var tp2 = rdl.new_var();
    nd = rdl.distance(tp2, origin, 0);
    assert(nd);
    nd = rdl.distance(horizon, tp2, 0);
    assert(nd);

    nd = rdl.distance(tp0, tp1, 10);
    assert(nd);
    nd = rdl.distance(tp1, tp0, 0);
    assert(nd);

    nd = rdl.distance(tp1, tp2, 10);
    assert(nd);
    nd = rdl.distance(tp2, tp1, 0);
    assert(nd);

    nd = rdl.distance(origin, tp0, 10);
    assert(nd);
    nd = rdl.distance(tp0, origin, 0);
    assert(nd);

    var tp2_after_40 = rdl.new_distance(tp2, origin, -40);
    assert(tp2_after_40 == FALSE_var);

    var tp2_after_20 = rdl.new_distance(tp2, origin, -20);
    var tp2_before_20 = rdl.new_distance(origin, tp2, 20);

    var tp2_after_30 = rdl.new_distance(tp2, origin, -30);
    var tp2_before_30 = rdl.new_distance(origin, tp2, 30);

    nd = core.disj({core.new_conj({tp2_after_20, tp2_before_20}), core.new_conj({tp2_after_30, tp2_before_30})});
    assert(nd);

    nd = rdl.distance(origin, tp2, 30);
    assert(nd);
    nd = rdl.distance(tp2, origin, -25);
    assert(nd);

    bool ch = core.check();
    assert(ch);
}

int main(int, char **)
{
    test_integer_distance_logic();
    test_real_distance_logic();
}