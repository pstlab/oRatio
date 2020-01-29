#include "temporal_network.h"
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

int main(int, char **)
{
    test_temporal_network_0();
    test_temporal_network_1();
}