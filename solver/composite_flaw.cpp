#include "composite_flaw.h"
#include "cartesian_product.h"
#include "combinations.h"
#include "solver.h"

using namespace smt;

namespace ratio
{

inline const std::vector<resolver *> get_cause(resolver *const cause)
{
    if (cause)
        return {cause};
    else
        return {};
}

composite_flaw::composite_flaw(graph &gr, resolver *const cause, const std::vector<flaw *> &fs) : flaw(gr, get_cause(cause)), flaws(fs)
{
    if (cause)
        make_precondition_of(*cause);
}
composite_flaw::~composite_flaw() {}

#ifdef BUILD_GUI
std::string composite_flaw::get_label() const
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

void composite_flaw::compute_resolvers()
{
    std::vector<std::vector<resolver *>> rss;
    for (const auto &f : flaws)
        rss.push_back(f->get_resolvers());

    for (const auto &rs : cartesian_product(rss))
        if (std::none_of(rs.begin(), rs.end(), [this](resolver *r) { return get_graph().get_solver().get_sat_core().value(r->get_rho()) == False; }))
        {
#ifdef CHECK_MUTEXES
            std::set<smt::var> mtx;
            for (const auto &r : rs)
                mtx.insert(r->get_rho());
            if (!get_graph().mutexes.count(mtx))
#endif
                add_resolver(*new composite_resolver(get_graph(), *this, rs));
        }
}

rational intrinsic_costs(const std::vector<resolver *> &rs)
{
    rational cost;
    for (const auto &r : rs)
        cost += r->get_intrinsic_cost();
    return cost;
}

composite_flaw::composite_resolver::composite_resolver(graph &gr, composite_flaw &s_flaw, const std::vector<resolver *> &rs) : resolver(gr, intrinsic_costs(rs), s_flaw), resolvers(rs) {}
composite_flaw::composite_resolver::~composite_resolver() {}

#ifdef BUILD_GUI
std::string composite_flaw::composite_resolver::get_label() const
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

void composite_flaw::composite_resolver::apply()
{
    // the application of the resolver must trigger the application of the underlying resolvers..
    for (const auto &r : resolvers)
        if (!get_graph().slv.get_sat_core().new_clause({lit(get_rho(), false), r->get_rho()}))
            throw unsolvable_exception();

    // all the resolver's preconditions..
    std::unordered_set<flaw *> c_precs;
    for (const auto &r : resolvers)
        c_precs.insert(r->get_preconditions().begin(), r->get_preconditions().end());

    for (auto it = c_precs.begin(); it != c_precs.end();)
        if (std::any_of((*it)->get_resolvers().begin(), (*it)->get_resolvers().end(), [this](resolver *r) { return get_graph().get_solver().get_sat_core().value(r->get_rho()) == True; }))
            it = c_precs.erase(it);
        else
            ++it;

    if (c_precs.size() > get_graph().get_accuracy()) // we create sets having the size of the accuracy..
    {
        const auto fss = combinations(std::vector<flaw *>(c_precs.begin(), c_precs.end()), get_graph().get_accuracy());
        for (const auto &fs : fss) // we create a new composite flaw for each of the possible combinations..
            get_graph().new_flaw(*new composite_flaw(get_graph(), this, fs));
    }
    else if (!c_precs.empty()) // we create a new composite flaw including all the preconditions of this resolver..
        get_graph().new_flaw(*new composite_flaw(get_graph(), this, std::vector<flaw *>(c_precs.begin(), c_precs.end())));
}
} // namespace ratio