#include "resolver.h"
#include "solver.h"
#include "flaw.h"

using namespace smt;

namespace ratio
{
    resolver::resolver(const rational &cost, flaw &eff) : resolver(lit(eff.slv.get_sat_core().new_var()), cost, eff) {}
    resolver::resolver(const lit &r, const rational &cost, flaw &eff) : slv(eff.slv), rho(r), intrinsic_cost(cost), effect(eff) {}
} // namespace ratio