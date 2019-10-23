#include "mutex_flaw.h"
#include "solver.h"

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

mutex_flaw::mutex_flaw(graph &gr, resolver *const cause, flaw &to_enqueue, const std::vector<resolver *> &non_mtx_rs) : flaw(gr, cause_to_vector(cause)), to_enqueue(to_enqueue), non_mtx_rs(non_mtx_rs) {}
mutex_flaw::~mutex_flaw() {}

#ifdef BUILD_GUI
std::string mutex_flaw::get_label() const
{
    return "φ" + std::to_string(get_phi()) + " {" + to_enqueue.get_label() + "}";
}
#endif

void mutex_flaw::compute_resolvers()
{
    if (non_mtx_rs.size() == 1)
        add_resolver(*new mutex_resolver(get_graph(), get_phi(), *this, **non_mtx_rs.begin()));
    else
        for (const auto &r : non_mtx_rs)
            add_resolver(*new mutex_resolver(get_graph(), *this, *r));
}

mutex_flaw::mutex_resolver::mutex_resolver(graph &gr, const smt::var &r, mutex_flaw &s_flaw, resolver &non_mtx_r) : resolver(gr, r, 0, s_flaw), non_mtx_r(non_mtx_r) {}
mutex_flaw::mutex_resolver::mutex_resolver(graph &gr, mutex_flaw &s_flaw, resolver &non_mtx_r) : resolver(gr, 0, s_flaw), non_mtx_r(non_mtx_r) {}
mutex_flaw::mutex_resolver::~mutex_resolver() {}

#ifdef BUILD_GUI
std::string mutex_flaw::mutex_resolver::get_label() const
{
    return "ρ" + std::to_string(get_rho()) + " {" + non_mtx_r.get_label() + "}";
}
#endif

void mutex_flaw::mutex_resolver::apply()
{
    // the application of the resolver must trigger the application of the underlying non mutex resolver..
    if (!get_graph().slv.get_sat_core().new_clause({lit(get_rho(), false), non_mtx_r.get_rho()}))
        throw unsolvable_exception();
    for (const auto &f : non_mtx_r.get_preconditions())
        get_graph().new_causal_link(*f, *this);
}
} // namespace ratio