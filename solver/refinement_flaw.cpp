#include "refinement_flaw.h"
#include "solver.h"
#include <cassert>

using namespace smt;

namespace ratio
{

static inline const std::vector<resolver *> cause_to_vector(resolver *const cause)
{
    if (cause)
        return {cause};
    else
        return {};
}

refinement_flaw::refinement_flaw(graph &gr, resolver *const cause, flaw &to_enqueue, const std::vector<resolver *> &non_mtx_rs) : flaw(gr, cause_to_vector(cause)), to_enqueue(to_enqueue), non_mtx_rs(non_mtx_rs) { assert(non_mtx_rs.size() > 1); } // non_mtx_rs should contain at least two resolvers, otherwise it should have been merged..
refinement_flaw::~refinement_flaw() {}

#ifdef BUILD_GUI
std::string refinement_flaw::get_label() const
{
    return "φ" + std::to_string(get_phi()) + " {" + to_enqueue.get_label() + "}";
}
#endif

void refinement_flaw::compute_resolvers()
{ // we need to create these resolvers (instead of using the original ones) for propagating costs..
    for (const auto &r : non_mtx_rs)
        add_resolver(*new refinement_resolver(get_graph(), *this, *r));
}

refinement_flaw::refinement_resolver::refinement_resolver(graph &gr, const smt::var &r, refinement_flaw &s_flaw, resolver &non_mtx_r) : resolver(gr, r, 0, s_flaw), non_mtx_r(non_mtx_r) {}
refinement_flaw::refinement_resolver::refinement_resolver(graph &gr, refinement_flaw &s_flaw, resolver &non_mtx_r) : resolver(gr, non_mtx_r.get_rho(), 0, s_flaw), non_mtx_r(non_mtx_r) {}
refinement_flaw::refinement_resolver::~refinement_resolver() {}

#ifdef BUILD_GUI
std::string refinement_flaw::refinement_resolver::get_label() const
{
    return "ρ" + std::to_string(get_rho()) + " {" + non_mtx_r.get_label() + "}";
}
#endif

void refinement_flaw::refinement_resolver::apply()
{
    for (const auto &f : non_mtx_r.get_preconditions())
        get_graph().new_causal_link(*f, *this);
}
} // namespace ratio