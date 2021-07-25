#include "resolver.h"
#include "solver.h"
#include "flaw.h"

using namespace smt;

namespace ratio
{
    resolver::resolver(solver &slv, const rational &cost, flaw &eff) : resolver(slv, lit(slv.get_sat_core().new_var()), cost, eff) {}
    resolver::resolver(solver &slv, const lit &r, const rational &cost, flaw &eff) : slv(slv), rho(r), intrinsic_cost(cost), effect(eff) {}
} // namespace ratio