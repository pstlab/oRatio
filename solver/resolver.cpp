#include "resolver.h"
#include "solver.h"
#include "flaw.h"

using namespace smt;

namespace ratio
{

resolver::resolver(graph &gr, const rational &cost, flaw &eff) : resolver(gr, gr.get_solver().get_sat_core().new_var(), cost, eff) {}
resolver::resolver(graph &gr, const var &r, const rational &cost, flaw &eff) : gr(gr), rho(r), intrinsic_cost(cost), effect(eff) {}
resolver::~resolver() {}
} // namespace ratio
