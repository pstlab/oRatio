#include "resolver.h"
#include "solver.h"
#include <cassert>

namespace ratio
{

resolver::resolver(solver &slv, const smt::var &r, const smt::rational &cost, flaw &eff) : slv(slv), rho(r), intrinsic_cost(cost), effect(eff) {}
resolver::resolver(solver &slv, const smt::rational &cost, flaw &eff) : resolver(slv, slv.sat_cr.new_var(), cost, eff) {}
resolver::~resolver() {}

void resolver::init()
{
    if (slv.sat_cr.value(rho) == smt::Undefined) // we do not have a top-level (a landmark) resolver..
    {
        // we listen for the resolver to become active..
        slv.rhos[rho].push_back(this);
        slv.bind(rho);
    }
}
}