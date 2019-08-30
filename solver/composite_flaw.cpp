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

inline const var conj(graph &gr, resolver *const cause, const std::vector<flaw *> &fs)
{
    if (cause)
    {
        for (const auto &f : fs)
            if (!gr.get_solver().get_sat_core().new_clause({lit(cause->get_rho(), false), f->get_phi()}))
                throw unsolvable_exception();
        return cause->get_rho();
    }
    else
        return TRUE_var;
}

composite_flaw::composite_flaw(graph &gr, resolver *const cause, const std::vector<flaw *> &fs) : flaw(gr, conj(gr, cause, fs), get_cause(cause), true), flaws(fs)
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
    for (const auto &f : flaws)
    {
        std::vector<flaw *> back;
        std::copy_if(flaws.begin(), flaws.end(), std::back_inserter(back), [f](const flaw *t) { return t != f; });
        for (const auto &r : f->get_resolvers())
            if (get_graph().get_solver().get_sat_core().value(r->get_rho()) != False)
                add_resolver(*new composite_resolver(get_graph(), *this, *r, back));
    }
}

composite_flaw::composite_resolver::composite_resolver(graph &gr, composite_flaw &s_flaw, const resolver &res, const std::vector<flaw *> &precs) : resolver(gr, res.get_intrinsic_cost(), s_flaw), res(res), precs(precs) {}
composite_flaw::composite_resolver::~composite_resolver() {}

#ifdef BUILD_GUI
std::string composite_flaw::composite_resolver::get_label() const
{
    return "ρ" + std::to_string(get_rho()) + " {" + res.get_label() + "}";
}
#endif

void composite_flaw::composite_resolver::apply()
{
    if (!get_graph().slv.get_sat_core().new_clause({lit(get_rho(), false), res.get_rho()}))
        throw unsolvable_exception();

    // all the resolver's preconditions..
    std::unordered_set<flaw *> c_precs(res.get_preconditions().begin(), res.get_preconditions().end());
    c_precs.insert(precs.begin(), precs.end());
    for (auto it = c_precs.begin(); it != c_precs.end();)
        if (std::any_of((*it)->get_resolvers().begin(), (*it)->get_resolvers().end(), [this](resolver *r) { return get_graph().get_solver().get_sat_core().value(r->get_rho()) == True; }))
            it = c_precs.erase(it);
        else
            ++it;

    if (c_precs.size() > get_graph().get_accuracy()) // we create sets having the size of the accuracy..
    {
        const auto fss = combinations(std::vector<flaw *>(c_precs.begin(), c_precs.end()), get_graph().get_accuracy());
        for (const auto &fs : fss) // we create a new composite flaw for each of the possible combinations..
            if (auto fs_it = get_graph().composite_flaws.find(std::set<flaw *>(fs.begin(), fs.end())); fs_it != get_graph().composite_flaws.end())
                get_graph().new_causal_link(*fs_it->second, *this);
            else
            {
                composite_flaw *cf = new composite_flaw(get_graph(), this, fs);
                get_graph().composite_flaws.emplace(std::set<flaw *>(fs.begin(), fs.end()), cf);
                get_graph().new_flaw(*cf);
            }
    }
    else if (!c_precs.empty()) // we create a new composite flaw including all the preconditions of this resolver..
        if (auto fs_it = get_graph().composite_flaws.find(std::set<flaw *>(c_precs.begin(), c_precs.end())); fs_it != get_graph().composite_flaws.end())
            get_graph().new_causal_link(*fs_it->second, *this);
        else
        {
            composite_flaw *cf = new composite_flaw(get_graph(), this, std::vector<flaw *>(c_precs.begin(), c_precs.end()));
            get_graph().composite_flaws.emplace(std::set<flaw *>(c_precs.begin(), c_precs.end()), cf);
            get_graph().new_flaw(*cf);
        }
}
} // namespace ratio