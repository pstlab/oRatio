#include "disj_flaw.h"
#include "solver.h"

namespace ratio
{
    disj_flaw::disj_flaw(solver &slv, const std::vector<resolver *> &causes, const std::vector<smt::lit> &lits) : flaw(slv, causes, false), lits(lits) {}
    disj_flaw::~disj_flaw() {}

    std::string disj_flaw::get_label() const noexcept { return "{\"type\":\"disj\", \"phi\":\"" + to_string(get_phi()) + "\", \"position\":" + std::to_string(get_position()) + "}"; }

    void disj_flaw::compute_resolvers()
    {
        for (const auto &p : lits)
            add_resolver(*new choose_lit(get_solver(), smt::rational(1, static_cast<smt::I>(lits.size())), *this, p));
    }

    disj_flaw::choose_lit::choose_lit(solver &slv, smt::rational cst, disj_flaw &disj_flaw, const smt::lit &p) : resolver(slv, p, cst, disj_flaw) {}
    disj_flaw::choose_lit::~choose_lit() {}

    std::string disj_flaw::choose_lit::get_label() const noexcept { return "{\"rho\":\"" + to_string(get_rho()) + "\"}"; }

    void disj_flaw::choose_lit::apply() {}
} // namespace ratio