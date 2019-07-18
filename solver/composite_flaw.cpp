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
    for (const auto &f : flaws)
    {
        std::vector<flaw *> back;
        std::copy_if(flaws.begin(), flaws.end(), std::back_inserter(back), [f](const flaw *t) { return t != f; });
        for (const auto &r : f->get_resolvers())
            add_resolver(*new composite_resolver(get_graph(), *this, *r, back));
    }
}

composite_flaw::composite_resolver::composite_resolver(graph &gr, composite_flaw &s_flaw, const resolver &res, const std::vector<flaw *> &precs) : resolver(gr, res.get_rho(), res.get_intrinsic_cost(), s_flaw), res(res), precs(precs) {}
composite_flaw::composite_resolver::~composite_resolver() {}

#ifdef BUILD_GUI
std::string composite_flaw::composite_resolver::get_label() const
{
    return res.get_label();
}
#endif

void composite_flaw::composite_resolver::apply()
{
    // all the resolver's preconditions..
    std::vector<flaw *> c_precs = res.get_preconditions();
    c_precs.insert(c_precs.end(), precs.begin(), precs.end());

    if (c_precs.size() > get_graph().get_accuracy()) // we create sets having the size of the accuracy..
    {
        const auto fss = combinations(std::vector<flaw *>(c_precs.begin(), c_precs.end()), get_graph().get_accuracy());
        for (const auto &fs : fss) // we create a new composite flaw for each of the possible combinations..
            get_graph().new_flaw(*new composite_flaw(get_graph(), this, fs));
    }
    else if (!c_precs.empty()) // we create a new composite flaw including all the preconditions of this resolver..
        get_graph().new_flaw(*new composite_flaw(get_graph(), this, c_precs));
}
} // namespace ratio