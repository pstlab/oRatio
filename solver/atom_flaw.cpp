#include "atom_flaw.h"
#include "solver.h"
#include "atom.h"
#include "smart_type.h"
#include "predicate.h"
#include <cassert>

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

atom_flaw::atom_flaw(graph &gr, resolver *const cause, atom &atm, const bool is_fact) : flaw(gr, get_cause(cause), true), atm(atm), is_fact(is_fact)
{
    if (cause)
        make_precondition_of(*cause);
}
atom_flaw::~atom_flaw() {}

#ifdef BUILD_GUI
std::string atom_flaw::get_label() const
{
    return "φ" + std::to_string(get_phi()) + (is_fact ? " fact σ" : " goal σ") + std::to_string(atm.get_sigma()) + " " + atm.get_type().get_name();
}
#endif

void atom_flaw::compute_resolvers()
{
    assert(get_graph().get_solver().get_sat_core().value(get_phi()) != False);
    assert(get_graph().get_solver().get_sat_core().value(atm.get_sigma()) != False);
    bool unify = false;
    if (get_graph().get_solver().get_sat_core().value(atm.get_sigma()) == Undefined) // we check if the atom can unify..
    {
        // we collect the ancestors of this flaw, so as to avoid cyclic causality..
        std::unordered_set<const flaw *> ancestors;
        std::queue<const flaw *> q;
        q.push(this);
        while (!q.empty())
        {
            if (ancestors.insert(q.front()).second)
                for (const auto &supp : q.front()->get_causes())
                    if (get_graph().get_solver().get_sat_core().value(supp->get_rho()) != False) // if false, the edge is broken..
                        q.push(&supp->get_effect());                                             // we push its effect..
            q.pop();
        }

        // we check for possible unifications (i.e. all the instances of the atom's type)..
        for (const auto &i : atm.get_type().get_instances())
        {
            if (&*i == &atm) // the current atom cannot unify with itself..
                continue;

            // this is the target (i.e. the atom we are checking for unification)..
            atom &t_atm = static_cast<atom &>(*i);

            // this is the target flaw (i.e. the one we are checking for unification) and cannot be in the current flaw's causes' effects..
            atom_flaw &t_flaw = get_graph().get_solver().get_reason(t_atm);

            if (!t_flaw.is_expanded() ||                                                     // the target flaw must hav been already expanded..
                ancestors.count(&t_flaw) ||                                                  // unifying with the target atom would introduce cyclic causality..
                get_graph().get_solver().get_sat_core().value(t_atm.get_sigma()) == False || // the target atom is unified with some other atom..
                !atm.equates(t_atm))                                                         // the atom does not equate with the target target..
                continue;

            // the equality propositional variable..
            var eq_v = atm.eq(t_atm);

            if (get_graph().get_solver().get_sat_core().value(eq_v) == False) // the two atoms cannot unify, hence, we skip this instance..
                continue;

            unify_atom *u_res = new unify_atom(get_graph(), *this, atm, t_atm, {lit(atm.get_sigma(), false), t_atm.get_sigma(), eq_v});
            assert(get_graph().get_solver().get_sat_core().value(u_res->get_rho()) != False);
            unify = true;
            add_resolver(*u_res);
            get_graph().new_causal_link(t_flaw, *u_res);
        }
    }

    if (is_fact)
        if (unify)
            add_resolver(*new activate_fact(get_graph(), *this, atm));
        else
            add_resolver(*new activate_fact(get_graph(), get_phi(), *this, atm));
    else if (unify)
        add_resolver(*new activate_goal(get_graph(), *this, atm));
    else
        add_resolver(*new activate_goal(get_graph(), get_phi(), *this, atm));
}

atom_flaw::activate_fact::activate_fact(graph &gr, atom_flaw &f, atom &a) : resolver(gr, 0, f), atm(a) {}
atom_flaw::activate_fact::activate_fact(graph &gr, const smt::var &r, atom_flaw &f, atom &a) : resolver(gr, r, 0, f), atm(a) {}
atom_flaw::activate_fact::~activate_fact() {}

#ifdef BUILD_GUI
std::string atom_flaw::activate_fact::get_label() const
{
    return "ρ" + std::to_string(get_rho()) + " add σ" + std::to_string(atm.get_sigma());
}
#endif

void atom_flaw::activate_fact::apply()
{
    get_graph().get_solver().get_sat_core().new_clause({lit(get_rho(), false), atm.get_sigma()});
}

atom_flaw::activate_goal::activate_goal(graph &gr, atom_flaw &f, atom &a) : resolver(gr, 1, f), atm(a) {}
atom_flaw::activate_goal::activate_goal(graph &gr, const smt::var &r, atom_flaw &f, atom &a) : resolver(gr, r, 1, f), atm(a) {}
atom_flaw::activate_goal::~activate_goal() {}

#ifdef BUILD_GUI
std::string atom_flaw::activate_goal::get_label() const
{
    return "ρ" + std::to_string(get_rho()) + " expand σ" + std::to_string(atm.get_sigma());
}
#endif

void atom_flaw::activate_goal::apply()
{
    get_graph().get_solver().get_sat_core().new_clause({lit(get_rho(), false), atm.get_sigma()});
    static_cast<const predicate &>(atm.get_type()).apply_rule(atm);
}

atom_flaw::unify_atom::unify_atom(graph &gr, atom_flaw &f, atom &atm, atom &trgt, const std::vector<lit> &unif_lits) : resolver(gr, 1, f), atm(atm), trgt(trgt), unif_lits(unif_lits) {}
atom_flaw::unify_atom::~unify_atom() {}

#ifdef BUILD_GUI
std::string atom_flaw::unify_atom::get_label() const
{
    return "ρ" + std::to_string(get_rho()) + " unify";
}
#endif

void atom_flaw::unify_atom::apply()
{
    for (const auto &v : unif_lits)
        get_graph().get_solver().get_sat_core().new_clause({lit(get_rho(), false), v});
}
} // namespace ratio