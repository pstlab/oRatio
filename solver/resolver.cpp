#include "resolver.h"
#include "solver.h"
#include "flaw.h"

using namespace smt;

namespace ratio
{
    resolver::resolver(solver &slv, const rational &cost, flaw &eff) : resolver(slv, lit(slv.get_sat_core().new_var()), cost, eff) {}
    resolver::resolver(solver &slv, const lit &r, const rational &cost, flaw &eff) : slv(slv), rho(r), intrinsic_cost(cost), effect(eff) {}
    resolver::~resolver() {}

    smt::rational resolver::get_estimated_cost() const noexcept
    {
        if (get_solver().get_sat_core().value(rho) == False)
            return rational::POSITIVE_INFINITY;
        else if (preconditions.empty())
            return intrinsic_cost;

        rational est_cost;
#ifdef H_MAX
        est_cost = rational::NEGATIVE_INFINITY;
        for (const auto &f : preconditions)
            if (!f->is_expanded())
                return rational::POSITIVE_INFINITY;
            else // we compute the maximum of the flaws' estimated costs..
            {
                rational c = f->get_estimated_cost();
                if (c > est_cost)
                    est_cost = c;
            }
#endif
#ifdef H_ADD
        for (const auto &f : preconditions)
            if (!f->is_expanded())
                return rational::POSITIVE_INFINITY;
            else // we compute the sum of the flaws' estimated costs..
                est_cost += f->get_estimated_cost();
#endif
        return est_cost + intrinsic_cost;
    }
} // namespace ratio