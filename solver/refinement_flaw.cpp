#include "refinement_flaw.h"
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

refinement_flaw::refinement_flaw(graph &gr, resolver *const cause, const std::vector<flaw *> &fs) : flaw(gr, cause_to_vector(cause)), flaws(fs)
{
    if (cause)
        make_precondition_of(*cause);
}
refinement_flaw::~refinement_flaw() {}

#ifdef BUILD_GUI
std::string refinement_flaw::get_label() const
{
    std::string f_str = "φ" + std::to_string(get_phi()) + " {";
    for (std::vector<flaw *>::const_iterator f_it = flaws.cbegin(); f_it != flaws.cend(); ++f_it)
    {
        if (f_it != flaws.cbegin())
            f_str += ", ";
        f_str += (*f_it)->get_label();
    }
    f_str += "}";
    return f_str;
}
#endif

void refinement_flaw::compute_resolvers()
{
    for (const auto &f : flaws)
        add_resolver(*new refinement_resolver(get_graph(), *this, f->get_causes()));
}

static inline rational intrinsic_costs(const std::vector<resolver *> &rs)
{
    rational cost;
    for (const auto &r : rs)
        cost += r->get_intrinsic_cost();
    return cost;
}

refinement_flaw::refinement_resolver::refinement_resolver(graph &gr, refinement_flaw &s_flaw, const std::vector<resolver *> &rs) : resolver(gr, intrinsic_costs(rs), s_flaw), resolvers(rs) {}
refinement_flaw::refinement_resolver::~refinement_resolver() {}

#ifdef BUILD_GUI
std::string refinement_flaw::refinement_resolver::get_label() const
{
    std::string r_str = "ρ" + std::to_string(get_rho()) + " {";
    for (std::vector<resolver *>::const_iterator f_it = resolvers.cbegin(); f_it != resolvers.cend(); ++f_it)
    {
        if (f_it != resolvers.cbegin())
            r_str += ", ";
        r_str += (*f_it)->get_label();
    }
    r_str += "}";
    return r_str;
}
#endif

void refinement_flaw::refinement_resolver::apply()
{
    for (const auto &r : resolvers)
    {
        // the application of the resolver must trigger the application of the underlying resolvers..
        if (!get_graph().slv.get_sat_core().new_clause({lit(get_rho(), false), r->get_rho()}))
            throw unsolvable_exception();
        for (const auto &f : r->get_preconditions())
            get_graph().new_causal_link(*f, *this);
    }
}
} // namespace ratio