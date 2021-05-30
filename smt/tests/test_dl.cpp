#include "idl_theory.h"
#include "rdl_theory.h"
#include <cassert>

using namespace smt;

void test_config()
{
    sat_core core;
    idl_theory idl(core, 5);
    var origin = idl.new_var();
    var horizon = idl.new_var();
    bool nc = core.new_clause({idl.new_distance(horizon, origin, 0)});
    assert(nc);

    bool prop = core.propagate();
    assert(prop);

    var tp0 = idl.new_var();
    nc = core.new_clause({idl.new_distance(tp0, origin, 0)});
    assert(nc);
    nc = core.new_clause({idl.new_distance(horizon, tp0, 0)});
    assert(nc);
    prop = core.propagate();
    assert(prop);

    var tp1 = idl.new_var();
    nc = core.new_clause({idl.new_distance(tp1, origin, 0)});
    assert(nc);
    nc = core.new_clause({idl.new_distance(horizon, tp1, 0)});
    assert(nc);
    prop = core.propagate();
    assert(prop);

    var tp2 = idl.new_var();
    nc = core.new_clause({idl.new_distance(tp2, origin, 0)});
    assert(nc);
    nc = core.new_clause({idl.new_distance(horizon, tp2, 0)});
    assert(nc);
    prop = core.propagate();
    assert(prop);

    nc = core.new_clause({idl.new_distance(tp0, tp1, 10)});
    assert(nc);
    nc = core.new_clause({idl.new_distance(tp1, tp0, 0)});
    assert(nc);

    nc = core.new_clause({idl.new_distance(tp1, tp2, 10)});
    assert(nc);
    nc = core.new_clause({idl.new_distance(tp2, tp1, 0)});
    assert(nc);

    nc = core.new_clause({idl.new_distance(origin, tp0, 10)});
    assert(nc);
    nc = core.new_clause({idl.new_distance(tp0, origin, 0)});
    assert(nc);
    prop = core.propagate();
    assert(prop);

    lit tp2_after_40 = idl.new_distance(tp2, origin, -40);
    assert(core.value(tp2_after_40) == FALSE_var);

    lit tp2_after_20 = idl.new_distance(tp2, origin, -20);
    lit tp2_before_20 = idl.new_distance(origin, tp2, 20);

    lit tp2_after_30 = idl.new_distance(tp2, origin, -30);
    lit tp2_before_30 = idl.new_distance(origin, tp2, 30);

    nc = core.new_clause({core.new_disj({core.new_conj({tp2_after_20, tp2_before_20}), core.new_conj({tp2_after_30, tp2_before_30})})});
    assert(nc);

    nc = core.new_clause({idl.new_distance(origin, tp2, 30)});
    assert(nc);
    nc = core.new_clause({idl.new_distance(tp2, origin, -25)});
    assert(nc);

    prop = core.propagate();
    assert(prop);

    std::pair<I, I> dist_origin_tp2 = idl.distance(origin, tp2);
    assert(dist_origin_tp2.first == 30 && dist_origin_tp2.second == 30);
}

void test_real_distance_logic()
{
    sat_core core;
    rdl_theory rdl(core, 5);
    var origin = rdl.new_var();
    var horizon = rdl.new_var();
    bool nc = core.new_clause({rdl.new_distance(horizon, origin, inf_rational(rational::ZERO))});
    assert(nc);

    bool prop = core.propagate();
    assert(prop);

    var tp0 = rdl.new_var();
    nc = core.new_clause({rdl.new_distance(tp0, origin, inf_rational(rational::ZERO))});
    assert(nc);
    nc = core.new_clause({rdl.new_distance(horizon, tp0, inf_rational(rational::ZERO))});
    assert(nc);

    var tp1 = rdl.new_var();
    nc = core.new_clause({rdl.new_distance(tp1, origin, inf_rational(rational::ZERO))});
    assert(nc);
    nc = core.new_clause({rdl.new_distance(horizon, tp1, inf_rational(rational::ZERO))});
    assert(nc);
    prop = core.propagate();
    assert(prop);

    var tp2 = rdl.new_var();
    nc = core.new_clause({rdl.new_distance(tp2, origin, inf_rational(rational::ZERO))});
    assert(nc);
    nc = core.new_clause({rdl.new_distance(horizon, tp2, inf_rational(rational::ZERO))});
    assert(nc);
    prop = core.propagate();
    assert(prop);

    nc = core.new_clause({rdl.new_distance(tp0, tp1, inf_rational(10))});
    assert(nc);
    nc = core.new_clause({rdl.new_distance(tp1, tp0, inf_rational(rational::ZERO))});
    assert(nc);

    nc = core.new_clause({rdl.new_distance(tp1, tp2, inf_rational(10))});
    assert(nc);
    nc = core.new_clause({rdl.new_distance(tp2, tp1, inf_rational(rational::ZERO))});
    assert(nc);

    nc = core.new_clause({rdl.new_distance(origin, tp0, inf_rational(10))});
    assert(nc);
    nc = core.new_clause({rdl.new_distance(tp0, origin, inf_rational(rational::ZERO))});
    assert(nc);
    prop = core.propagate();
    assert(prop);

    lit tp2_after_40 = rdl.new_distance(tp2, origin, -inf_rational(40));
    assert(tp2_after_40 == FALSE_lit);

    lit tp2_after_20 = rdl.new_distance(tp2, origin, -inf_rational(20));
    lit tp2_before_20 = rdl.new_distance(origin, tp2, inf_rational(20));

    lit tp2_after_30 = rdl.new_distance(tp2, origin, -inf_rational(30));
    lit tp2_before_30 = rdl.new_distance(origin, tp2, inf_rational(30));

    nc = core.new_clause({core.new_disj({core.new_conj({tp2_after_20, tp2_before_20}), core.new_conj({tp2_after_30, tp2_before_30})})});
    assert(nc);

    nc = core.new_clause({rdl.new_distance(origin, tp2, inf_rational(30))});
    assert(nc);
    nc = core.new_clause({rdl.new_distance(tp2, origin, -inf_rational(25))});
    assert(nc);

    prop = core.propagate();
    assert(prop);

    std::pair<inf_rational, inf_rational> dist_origin_tp2 = rdl.distance(origin, tp2);
    assert(dist_origin_tp2.first == 30 && dist_origin_tp2.second == 30);
}

void test_constraints_0()
{
    sat_core core;
    idl_theory idl(core, 5);
    var origin = idl.new_var();
    // origin >= 0..
    bool nc = core.new_clause({idl.new_distance(origin, 0, 0)});
    assert(nc);
    bool prop = core.propagate();
    assert(prop);
    std::pair<I, I> bound_origin = idl.bounds(origin);
    assert(bound_origin.first == 0 && bound_origin.second == idl_theory::inf());

    var horizon = idl.new_var();
    // horizon >= origin..
    nc = core.new_clause({idl.new_distance(horizon, origin, 0)});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    std::pair<I, I> bound_horizon = idl.bounds(horizon);
    assert(bound_horizon.first == 0 && bound_horizon.second == idl_theory::inf());

    // horizon < 20..
    lit horizon_lt_20 = idl.new_lt(lin(horizon, rational::ONE), lin(rational(20)));
    assert(bound_horizon == idl.bounds(horizon));

    nc = core.new_clause({horizon_lt_20});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    bound_horizon = idl.bounds(horizon);
    assert(bound_horizon.first == 0 && bound_horizon.second == 19);

    // horizon <= 15..
    lit horizon_leq_15 = idl.new_leq(lin(horizon, rational::ONE), lin(rational(15)));
    assert(bound_horizon == idl.bounds(horizon));

    nc = core.new_clause({horizon_leq_15});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    bound_horizon = idl.bounds(horizon);
    assert(bound_horizon.first == 0 && bound_horizon.second == 15);
    bound_origin = idl.bounds(origin);
    assert(bound_origin.first == 0 && bound_origin.second == 15);

    // origin > 5..
    lit origin_gt_5 = idl.new_gt(lin(origin, rational::ONE), lin(rational(5)));
    assert(bound_origin == idl.bounds(origin));

    nc = core.new_clause({origin_gt_5});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    bound_origin = idl.bounds(origin);
    assert(bound_origin.first == 6 && bound_origin.second == 15);

    // origin >= 10..
    lit origin_geq_10 = idl.new_geq(lin(origin, rational::ONE), lin(rational(10)));
    assert(bound_origin == idl.bounds(origin));

    nc = core.new_clause({origin_geq_10});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    bound_origin = idl.bounds(origin);
    assert(bound_origin.first == 10 && bound_origin.second == 15);
}

void test_constraints_1()
{
    sat_core core;
    idl_theory idl(core, 5);
    var origin = idl.new_var();
    // origin >= 0..
    bool nc = core.new_clause({idl.new_distance(origin, 0, 0)});
    assert(nc);
    bool prop = core.propagate();
    assert(prop);
    std::pair<I, I> bound_origin = idl.bounds(origin);
    assert(bound_origin.first == 0 && bound_origin.second == idl_theory::inf());

    var horizon = idl.new_var();
    // horizon >= origin..
    nc = core.new_clause({idl.new_distance(horizon, origin, 0)});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    std::pair<I, I> bound_horizon = idl.bounds(horizon);
    assert(bound_horizon.first == 0 && bound_horizon.second == idl_theory::inf());

    // -horizon >= -20..
    lit horizon_geq_20 = idl.new_geq(lin(horizon, -rational::ONE), lin(rational(-20)));
    assert(bound_horizon == idl.bounds(horizon));

    nc = core.new_clause({horizon_geq_20});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    bound_horizon = idl.bounds(horizon);
    assert(bound_horizon.first == 0 && bound_horizon.second == 20);

    // -horizon > -15..
    lit horizon_gt_15 = idl.new_gt(lin(horizon, -rational::ONE), lin(rational(-15)));
    assert(bound_horizon == idl.bounds(horizon));

    nc = core.new_clause({horizon_gt_15});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    bound_horizon = idl.bounds(horizon);
    assert(bound_horizon.first == 0 && bound_horizon.second == 14);
    bound_origin = idl.bounds(origin);
    assert(bound_origin.first == 0 && bound_origin.second == 14);

    // -origin <= -5..
    lit origin_leq_5 = idl.new_leq(lin(origin, -rational::ONE), lin(rational(-5)));
    assert(bound_origin == idl.bounds(origin));

    nc = core.new_clause({origin_leq_5});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    bound_origin = idl.bounds(origin);
    assert(bound_origin.first == 5 && bound_origin.second == 14);

    // -origin < -10..
    lit origin_lt_10 = idl.new_lt(lin(origin, -rational::ONE), lin(rational(-10)));
    assert(bound_origin == idl.bounds(origin));

    nc = core.new_clause({origin_lt_10});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    bound_origin = idl.bounds(origin);
    assert(bound_origin.first == 11 && bound_origin.second == 14);
}

void test_constraints_2()
{
    sat_core core;
    rdl_theory rdl(core, 5);
    var origin = rdl.new_var();
    // origin >= 0..
    bool nc = core.new_clause({rdl.new_distance(origin, 0, inf_rational(rational::ZERO))});
    assert(nc);
    bool prop = core.propagate();
    assert(prop);
    std::pair<inf_rational, inf_rational> bound_origin = rdl.bounds(origin);
    assert(bound_origin.first == 0 && bound_origin.second == rational::POSITIVE_INFINITY);

    var horizon = rdl.new_var();
    // horizon >= origin..
    nc = core.new_clause({rdl.new_distance(horizon, origin, inf_rational(rational::ZERO))});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    std::pair<inf_rational, inf_rational> bound_horizon = rdl.bounds(horizon);
    assert(bound_horizon.first == 0 && bound_horizon.second == rational::POSITIVE_INFINITY);

    // horizon < 20..
    lit horizon_lt_20 = rdl.new_lt(lin(horizon, rational::ONE), lin(rational(20)));
    assert(bound_horizon == rdl.bounds(horizon));

    nc = core.new_clause({horizon_lt_20});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    bound_horizon = rdl.bounds(horizon);
    assert(bound_horizon.first == 0 && bound_horizon.second == inf_rational(rational(20), -1));

    // horizon <= 15..
    lit horizon_leq_15 = rdl.new_leq(lin(horizon, rational::ONE), lin(rational(15)));
    assert(bound_horizon == rdl.bounds(horizon));

    nc = core.new_clause({horizon_leq_15});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    bound_horizon = rdl.bounds(horizon);
    assert(bound_horizon.first == 0 && bound_horizon.second == 15);
    bound_origin = rdl.bounds(origin);
    assert(bound_origin.first == 0 && bound_origin.second == 15);

    // origin > 5..
    lit origin_gt_5 = rdl.new_gt(lin(origin, rational::ONE), lin(rational(5)));
    assert(bound_origin == rdl.bounds(origin));

    nc = core.new_clause({origin_gt_5});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    bound_origin = rdl.bounds(origin);
    assert(bound_origin.first == inf_rational(rational(5), 1) && bound_origin.second == 15);

    // origin >= 10..
    lit origin_geq_10 = rdl.new_geq(lin(origin, rational::ONE), lin(rational(10)));
    assert(bound_origin == rdl.bounds(origin));

    nc = core.new_clause({origin_geq_10});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    bound_origin = rdl.bounds(origin);
    assert(bound_origin.first == 10 && bound_origin.second == 15);
}

void test_constraints_3()
{
    sat_core core;
    rdl_theory rdl(core, 5);
    var origin = rdl.new_var();
    // origin >= 0..
    bool nc = core.new_clause({rdl.new_distance(origin, 0, inf_rational(rational::ZERO))});
    assert(nc);
    bool prop = core.propagate();
    assert(prop);
    std::pair<inf_rational, inf_rational> bound_origin = rdl.bounds(origin);
    assert(bound_origin.first == 0 && bound_origin.second == rational::POSITIVE_INFINITY);

    var horizon = rdl.new_var();
    // horizon >= origin..
    nc = core.new_clause({rdl.new_distance(horizon, origin, inf_rational(rational::ZERO))});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    std::pair<inf_rational, inf_rational> bound_horizon = rdl.bounds(horizon);
    assert(bound_horizon.first == 0 && bound_horizon.second == rational::POSITIVE_INFINITY);

    // -horizon >= -20..
    lit horizon_geq_20 = rdl.new_geq(lin(horizon, -rational::ONE), lin(rational(-20)));
    assert(bound_horizon == rdl.bounds(horizon));

    nc = core.new_clause({horizon_geq_20});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    bound_horizon = rdl.bounds(horizon);
    assert(bound_horizon.first == 0 && bound_horizon.second == 20);

    // -horizon > -15..
    lit horizon_gt_15 = rdl.new_gt(lin(horizon, -rational::ONE), lin(rational(-15)));
    assert(bound_horizon == rdl.bounds(horizon));

    nc = core.new_clause({horizon_gt_15});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    bound_horizon = rdl.bounds(horizon);
    assert(bound_horizon.first == 0 && bound_horizon.second == inf_rational(rational(15), -1));
    bound_origin = rdl.bounds(origin);
    assert(bound_origin.first == 0 && bound_origin.second == inf_rational(rational(15), -1));

    // -origin <= -5..
    lit origin_leq_5 = rdl.new_leq(lin(origin, -rational::ONE), lin(rational(-5)));
    assert(bound_origin == rdl.bounds(origin));

    nc = core.new_clause({origin_leq_5});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    bound_origin = rdl.bounds(origin);
    assert(bound_origin.first == 5 && bound_origin.second == inf_rational(rational(15), -1));

    // -origin < -10..
    lit origin_lt_10 = rdl.new_lt(lin(origin, -rational::ONE), lin(rational(-10)));
    assert(bound_origin == rdl.bounds(origin));

    nc = core.new_clause({origin_lt_10});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    bound_origin = rdl.bounds(origin);
    assert(bound_origin.first == inf_rational(rational(10), 1) && bound_origin.second == inf_rational(rational(15), -1));
}

void test_constraints_4()
{
    sat_core core;
    rdl_theory rdl(core, 5);
    var origin = rdl.new_var();
    // origin >= 0..
    bool nc = core.new_clause({rdl.new_geq(lin(origin, rational::ONE), lin(rational::ZERO))});
    assert(nc);
    bool prop = core.propagate();
    assert(prop);
    std::pair<inf_rational, inf_rational> bound_origin = rdl.bounds(origin);
    assert(bound_origin.first == 0 && bound_origin.second == rational::POSITIVE_INFINITY);

    var horizon = rdl.new_var();
    // horizon >= origin..
    nc = core.new_clause({rdl.new_geq(lin(horizon, rational::ONE), lin(origin, rational::ONE))});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    std::pair<inf_rational, inf_rational> bound_horizon = rdl.bounds(horizon);
    assert(bound_horizon.first == 0 && bound_horizon.second == rational::POSITIVE_INFINITY);

    lit hor_eq_10 = rdl.new_eq(lin(horizon, rational::ONE), lin(rational(10)));
    assert(bound_horizon == rdl.bounds(horizon));
    assert(bound_origin == rdl.bounds(origin));

    nc = core.new_clause({hor_eq_10});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    bound_origin = rdl.bounds(origin);
    assert(bound_origin.first == 0 && bound_origin.second == 10);
    bound_horizon = rdl.bounds(horizon);
    assert(bound_horizon.first == 10 && bound_horizon.second == 10);
}

void test_constraints_5()
{
    sat_core core;
    rdl_theory rdl(core, 5);
    var origin = rdl.new_var();
    // origin >= 0..
    bool nc = core.new_clause({rdl.new_geq(lin(origin, rational::ONE), lin(rational::ZERO))});
    assert(nc);
    bool prop = core.propagate();
    assert(prop);
    std::pair<inf_rational, inf_rational> bound_origin = rdl.bounds(origin);
    assert(bound_origin.first == 0 && bound_origin.second == rational::POSITIVE_INFINITY);

    var horizon = rdl.new_var();
    // horizon >= origin..
    nc = core.new_clause({rdl.new_geq(lin(horizon, rational::ONE), lin(origin, rational::ONE))});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    std::pair<inf_rational, inf_rational> bound_horizon = rdl.bounds(horizon);
    assert(bound_horizon.first == 0 && bound_horizon.second == rational::POSITIVE_INFINITY);

    var v1 = rdl.new_var(), v2 = rdl.new_var(), v3 = rdl.new_var(), v4 = rdl.new_var();
    nc = core.new_clause({rdl.new_distance(v1, origin, inf_rational(rational::ZERO))});
    assert(nc);
    nc = core.new_clause({rdl.new_distance(horizon, v1, inf_rational(rational::ZERO))});
    assert(nc);
    nc = core.new_clause({rdl.new_distance(v2, origin, inf_rational(rational::ZERO))});
    assert(nc);
    nc = core.new_clause({rdl.new_distance(horizon, v2, inf_rational(rational::ZERO))});
    assert(nc);
    nc = core.new_clause({rdl.new_distance(v3, origin, inf_rational(rational::ZERO))});
    assert(nc);
    nc = core.new_clause({rdl.new_distance(horizon, v3, inf_rational(rational::ZERO))});
    assert(nc);
    nc = core.new_clause({rdl.new_distance(v4, origin, inf_rational(rational::ZERO))});
    assert(nc);
    nc = core.new_clause({rdl.new_distance(horizon, v4, inf_rational(rational::ZERO))});
    assert(nc);

    nc = core.new_clause({rdl.new_distance(v1, v2, inf_rational(20), inf_rational(20))});
    assert(nc);
    nc = core.new_clause({rdl.new_distance(v3, v4, inf_rational(20), inf_rational(20))});
    assert(nc);

    lit v2_v3 = rdl.new_leq(lin(v2, rational::ONE), lin(v3, rational::ONE));
    assert(core.value(v2_v3) == Undefined);
    lit v4_v1 = rdl.new_leq(lin(v4, rational::ONE), lin(v1, rational::ONE));
    assert(core.value(v4_v1) == Undefined);

    nc = core.new_clause({rdl.new_distance(v1, v3, inf_rational(rational::ZERO), inf_rational(rational::ZERO))});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    assert(core.value(v2_v3) == False);
    assert(core.value(v4_v1) == False);
}

void test_semantic_branching()
{
    sat_core core;
    rdl_theory rdl(core, 5);
    var origin = rdl.new_var();
    // origin >= 0..
    bool nc = core.new_clause({rdl.new_geq(lin(origin, rational::ONE), lin(rational::ZERO))});
    assert(nc);
    bool prop = core.propagate();
    assert(prop);
    std::pair<inf_rational, inf_rational> bound_origin = rdl.bounds(origin);
    assert(bound_origin.first == 0 && bound_origin.second == rational::POSITIVE_INFINITY);

    var horizon = rdl.new_var();
    // horizon >= origin..
    nc = core.new_clause({rdl.new_geq(lin(horizon, rational::ONE), lin(origin, rational::ONE))});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    std::pair<inf_rational, inf_rational> bound_horizon = rdl.bounds(horizon);
    assert(bound_horizon.first == 0 && bound_horizon.second == rational::POSITIVE_INFINITY);

    lit hor_leq_10 = rdl.new_leq(lin(horizon, rational::ONE), lin(rational(10)));
    assert(bound_horizon == rdl.bounds(horizon));
    assert(bound_origin == rdl.bounds(origin));

    nc = core.new_clause({!hor_leq_10});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    bound_origin = rdl.bounds(origin);
    assert(bound_origin.first == 0 && bound_origin.second == rational::POSITIVE_INFINITY);
    bound_horizon = rdl.bounds(horizon);
    assert(bound_horizon.first == inf_rational(rational(10), 1) && bound_horizon.second == rational::POSITIVE_INFINITY);

    lit hor_geq_20 = rdl.new_geq(lin(horizon, rational::ONE), lin(rational(20)));
    assert(bound_horizon == rdl.bounds(horizon));
    assert(bound_origin == rdl.bounds(origin));

    nc = core.new_clause({!hor_geq_20});
    assert(nc);
    prop = core.propagate();
    assert(prop);
    bound_origin = rdl.bounds(origin);
    assert(bound_origin.first == 0 && bound_origin.second == inf_rational(rational(20), -1));
    bound_horizon = rdl.bounds(horizon);
    assert(bound_horizon.first == inf_rational(rational(10), 1) && bound_horizon.second == inf_rational(rational(20), -1));
}

int main(int, char **)
{
    test_config();
    test_real_distance_logic();

    test_constraints_0();
    test_constraints_1();
    test_constraints_2();
    test_constraints_3();

    test_constraints_4();
    test_constraints_5();

    test_semantic_branching();
}