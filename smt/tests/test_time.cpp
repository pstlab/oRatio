#include "temporal_network.h"
#include "idl_theory.h"
#include "rdl_theory.h"
#include <cassert>

using namespace smt;

void test_temporal_network_0()
{
    temporal_network<double> tn(5);

    size_t tp0 = tn.new_tp();
    size_t tp1 = tn.new_tp();
    size_t tp2 = tn.new_tp();

    auto bnd_0 = tn.bound(tp0);
    auto bnd_1 = tn.bound(tp1);
    auto d_0_1 = tn.dist(tp0, tp1);

    tn.add_constraint(tp0, tp1, 0, 10);
    d_0_1 = tn.dist(tp0, tp1);
    tn.add_constraint(tp1, tp2, 0, 10);
    auto d_1_2 = tn.dist(tp1, tp2);
    auto d_0_2 = tn.dist(tp0, tp2);

    tn.add_constraint(temporal_network<double>::origin(), tp0, 0, 10);
    bnd_0 = tn.bound(tp0);
    bnd_1 = tn.bound(tp1);
    d_0_1 = tn.dist(tp0, tp1);
    d_1_2 = tn.dist(tp1, tp2);
    d_0_2 = tn.dist(tp0, tp2);

    size_t tp3 = tn.new_tp();
    size_t tp4 = tn.new_tp();
    size_t tp5 = tn.new_tp();
}

void test_temporal_network_1()
{
    temporal_network<rational> tn(5);

    size_t tp0 = tn.new_tp();
    tn.add_constraint(temporal_network<rational>::origin(), tp0, 0, 10);
    auto bnd_0 = tn.bound(tp0);
}

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
    test_temporal_network_0();
    test_temporal_network_1();

    test_integer_distance_logic();
    test_real_distance_logic();
}