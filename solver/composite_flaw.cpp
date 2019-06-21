#include "composite_flaw.h"
#include "cartesian_product.h"
#include "combinations.h"
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

composite_flaw::composite_flaw(graph &gr, resolver *const cause, const std::vector<flaw *> &fs) : flaw(gr, get_cause(cause)), flaws(fs)
{
    if (cause)
        add_precondition(*cause);
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
    std::vector<std::vector<resolver *>> rs;
    for (const auto &f : flaws)
        rs.push_back(f->get_resolvers());

#ifdef CHECK_COMPOSITE_FLAWS
    std::vector<smt::lit> check_lits;
    std::queue<const flaw *> q;
    q.push(this);
    std::unordered_set<const flaw *> seen;
    while (!q.empty())
    {
        if (seen.insert(q.front()).second) // we avoid some repetition of literals..
        {
            if (get_graph().get_solver().get_sat_core().value(q.front()->get_phi() == smt::False))
                return;
            for (const auto &cause : q.front()->get_causes())
                if (get_graph().get_solver().get_sat_core().value(cause->get_rho()) != smt::True)
                {
                    check_lits.push_back(cause->get_rho()); // we add the resolver's variable to the unification literals..
                    q.push(&cause->get_effect());           // we push its effect..
                }
        }
        q.pop();
    }
#endif

    for (const auto &rp : cartesian_product(rs))
    {
        // the resolver's cost is given by the sum of the enclosing resolvers' costs..
        smt::rational cst;
        std::vector<smt::lit> cnj;
        for (const auto &r : rp)
        {
            cst += r->get_intrinsic_cost();
            cnj.push_back(r->get_rho());
        }
        smt::var cnj_var = get_graph().get_solver().get_sat_core().new_conj(cnj);

#ifdef CHECK_COMPOSITE_FLAWS
        check_lits.push_back(cnj_var);
        if (get_graph().get_solver().get_sat_core().check(check_lits))
            add_resolver(*new composite_resolver(get_graph(), *this, cnj_var, cst, rp));
        else
        {
            std::vector<smt::lit> no_good;
            no_good.reserve(check_lits.size());
            for (const auto &l : check_lits)
                no_good.push_back(!l);
            if (!get_graph().get_solver().get_sat_core().new_clause(no_good))
                throw std::runtime_error("the problem is inconsistent..");
        }
        check_lits.pop_back();
#else
        if (get_graph().get_solver().get_sat_core().value(cnj_var) != smt::False)
            add_resolver(*new composite_resolver(get_graph(), *this, cnj_var, cst, rp));
#endif
    }
}

composite_flaw::composite_resolver::composite_resolver(graph &gr, composite_flaw &s_flaw, const smt::var &app_r, const smt::rational &cst, const std::vector<resolver *> &rs) : resolver(gr, app_r, cst, s_flaw), resolvers(rs) {}
composite_flaw::composite_resolver::~composite_resolver() {}

#ifdef BUILD_GUI
std::string composite_flaw::composite_resolver::get_label() const
{
    std::string r_str = "ρ" + std::to_string(get_rho()) + " {";
    for (std::vector<resolver *>::const_iterator r_it = resolvers.cbegin(); r_it != resolvers.cend(); ++r_it)
    {
        if (r_it != resolvers.cbegin())
            r_str += ", ";
        r_str += (*r_it)->get_label();
    }
    r_str += "}";
    return r_str;
}
#endif

void composite_flaw::composite_resolver::apply()
{
    // all the resolver's preconditions..
    std::vector<flaw *> precs;
    for (const auto &r : resolvers)
        for (const auto &pre : r->get_preconditions())
            if (get_graph().get_solver().get_sat_core().value(pre->get_phi()) != smt::True) // we don't need to consider those flaws which are already active..
                precs.push_back(pre);

    if (precs.size() > get_graph().get_accuracy()) // we create sets having the size of the accuracy..
    {
        const auto fss = combinations(std::vector<flaw *>(precs.begin(), precs.end()), get_graph().get_accuracy());
        for (const auto &fs : fss) // we create a new composite flaw for each of the possible combinations..
            get_graph().new_flaw(*new composite_flaw(get_graph(), this, fs));
    }
    else if (!precs.empty()) // we create a new composite flaw including all the preconditions of this resolver..
        get_graph().new_flaw(*new composite_flaw(get_graph(), this, precs));
}
} // namespace ratio