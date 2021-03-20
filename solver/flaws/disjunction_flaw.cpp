#include "disjunction_flaw.h"
#include "conjunction.h"
#include "solver.h"

namespace ratio
{
    disjunction_flaw::disjunction_flaw(solver &slv, const std::vector<resolver *> &causes, const context &ctx, const std::vector<const conjunction *> &conjs) : flaw(slv, causes, false), ctx(ctx), conjs(conjs) {}
    disjunction_flaw::~disjunction_flaw() {}

    std::string disjunction_flaw::get_label() const noexcept { return "{\"type\":\"disjunction\", \"phi\":\"" + to_string(get_phi()) + "\", \"position\":" + std::to_string(get_position()) + "}"; }

    void disjunction_flaw::compute_resolvers()
    {
        for (const auto &cnj : conjs)
        {
            context cnj_ctx(new env(get_solver(), context(ctx)));
            add_resolver(*new choose_conjunction(get_solver(), *this, cnj_ctx, *cnj));
        }
    }

    disjunction_flaw::choose_conjunction::choose_conjunction(solver &slv, disjunction_flaw &disj_flaw, const context &ctx, const conjunction &conj) : resolver(slv, conj.get_cost(), disj_flaw), ctx(ctx), conj(conj) {}
    disjunction_flaw::choose_conjunction::~choose_conjunction() {}

    std::string disjunction_flaw::choose_conjunction::get_label() const noexcept { return "{\"rho\":\"" + to_string(get_rho()) + "\"}"; }

    void disjunction_flaw::choose_conjunction::apply() { conj.apply(ctx); }
} // namespace ratio