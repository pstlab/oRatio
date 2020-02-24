#include "disjunction_flaw.h"
#include "disjunction.h"
#include "conjunction.h"
#include "solver.h"

namespace ratio
{

static inline const std::vector<resolver *> cause_to_vector(resolver *const cause)
{
    if (cause)
        return {cause};
    else
        return {};
}

disjunction_flaw::disjunction_flaw(graph &gr, resolver *const cause, const context &ctx, const disjunction &disj) : flaw(gr, cause_to_vector(cause), false), ctx(ctx), disj(disj) {}
disjunction_flaw::~disjunction_flaw() {}

std::string disjunction_flaw::get_label() const { return "{\"type\":\"disjunction\", \"phi\":" + std::to_string(get_phi()) + "}"; }

void disjunction_flaw::compute_resolvers()
{
    for (const auto &cnj : disj.get_conjunctions())
    {
        context cnj_ctx(new env(get_graph().get_solver(), ctx));
        add_resolver(*new choose_conjunction(get_graph(), *this, cnj_ctx, *cnj));
    }
}

disjunction_flaw::choose_conjunction::choose_conjunction(graph &gr, disjunction_flaw &disj_flaw, const context &ctx, const conjunction &conj) : resolver(gr, conj.get_cost(), disj_flaw), ctx(ctx), conj(conj) {}
disjunction_flaw::choose_conjunction::~choose_conjunction() {}

std::string disjunction_flaw::choose_conjunction::get_label() const { return "{\"rho\":" + std::to_string(get_rho()) + "}"; }

void disjunction_flaw::choose_conjunction::apply() { conj.apply(ctx); }
} // namespace ratio