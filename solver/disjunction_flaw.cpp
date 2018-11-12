#include "disjunction_flaw.h"
#include "disjunction.h"
#include "conjunction.h"
#include "solver.h"

namespace ratio
{

inline const std::vector<resolver *> get_cause(resolver *const cause)
{
    if (cause)
        return {cause};
    else
        return {};
}

disjunction_flaw::disjunction_flaw(solver &slv, resolver *const cause, const context &ctx, const disjunction &disj) : flaw(slv, get_cause(cause), false, true), ctx(ctx), disj(disj) {}
disjunction_flaw::~disjunction_flaw() {}

void disjunction_flaw::compute_resolvers()
{
    for (const auto &cnj : disj.get_conjunctions())
    {
        context cnj_ctx(new env(slv, ctx));
        add_resolver(*new choose_conjunction(slv, *this, cnj_ctx, *cnj));
    }
}

disjunction_flaw::choose_conjunction::choose_conjunction(solver &slv, disjunction_flaw &disj_flaw, const context &ctx, const conjunction &conj) : resolver(slv, conj.get_cost(), disj_flaw), ctx(ctx), conj(conj) {}
disjunction_flaw::choose_conjunction::~choose_conjunction() {}

void disjunction_flaw::choose_conjunction::apply() { conj.apply(ctx); }
}