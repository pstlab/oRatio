#include "atom_flaw.h"
#include "solver.h"
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

atom_flaw::atom_flaw(solver &slv, resolver *const cause, atom &atm, const bool is_fact) : flaw(slv, get_cause(cause), true, true), atm(atm), is_fact(is_fact) {}
atom_flaw::~atom_flaw() {}

void atom_flaw::compute_resolvers()
{
    assert(slv.sat_cr.value(get_phi()) != False);
    assert(slv.sat_cr.value(atm.sigma) != False);
    if (slv.sat_cr.value(atm.sigma) == Undefined) // we check if the atom can unify..
    {
        // we collect the ancestors of this flaw, so as to avoid cyclic causality..
        std::unordered_set<const flaw *> ancestors;
        std::queue<const flaw *> q;
        q.push(this);
        while (!q.empty())
        {
            if (ancestors.find(q.front()) == ancestors.end())
            {
                ancestors.insert(q.front());
                for (const auto &supp : q.front()->get_supports())
                    if (slv.sat_cr.value(supp->get_rho()) != False) // if false, the edge is broken..
                        q.push(&supp->get_effect());                // we push its effect..
            }
            q.pop();
        }

        for (const auto &i : atm.tp.get_instances())
        {
            if (&*i == &atm) // the current atom cannot unify with itself..
                continue;

            // this is the atom we are checking for unification..
            atom &c_atm = static_cast<atom &>(*i);

            // this is the target flaw (i.e. the one we are checking for unification) and cannot be in the current flaw's causes' effects..
            atom_flaw &target = slv.get_reason(c_atm);

            if (!target.is_expanded() ||                      // the target flaw must hav been already expanded..
                ancestors.find(&target) != ancestors.end() || // unifying with the target atom would introduce cyclic causality..
                slv.sat_cr.value(c_atm.sigma) == False ||     // the target atom is unified with some other atom..
                !atm.equates(c_atm))                          // the atom does not equate with the target target..
                continue;

            // the equality propositional variable..
            var eq_v = atm.eq(c_atm);

            if (slv.sat_cr.value(eq_v) == False) // the two atoms cannot unify, hence, we skip this instance..
                continue;

#ifdef CHECK_UNIFICATIONS
            // since atom 'c_atm' is a good candidate for unification, we build the unification literals..
            std::vector<lit> unif_lits;
            q.push(this);
            q.push(&target);
            unif_lits.push_back(lit(atm.sigma, false)); // we force the state of this atom to be 'unified' within the unification literals..
            unif_lits.push_back(c_atm.sigma);           // we force the state of the target atom to be 'active' within the unification literals..
            std::unordered_set<const flaw *> seen;
            while (!q.empty())
            {
                if (seen.find(q.front()) == seen.end())
                {
                    seen.insert(q.front()); // we avoid some repetition of literals..
                    for (const auto &cause : q.front()->get_causes())
                        if (slv.sat_cr.value(cause->get_rho()) != True)
                        {
                            unif_lits.push_back(cause->get_rho()); // we add the resolver's variable to the unification literals..
                            q.push(&cause->get_effect());          // we push its effect..
                        }
                }
                q.pop();
            }

            if (slv.sat_cr.value(eq_v) != True)
                unif_lits.push_back(eq_v);
            if (unif_lits.empty() || slv.sat_cr.check(unif_lits))
            {
                unify_atom *u_res = new unify_atom(slv, *this, atm, c_atm, unif_lits);
#else
            unify_atom *u_res = new unify_atom(slv, *this, atm, c_atm, {lit(atm.sigma, false), c_atm.sigma, eq_v});
#endif
                assert(slv.sat_cr.value(u_res->get_rho()) != False);
                add_resolver(*u_res);
                slv.new_causal_link(target, *u_res);
                slv.set_estimated_cost(*u_res, target.get_estimated_cost());
#ifdef CHECK_UNIFICATIONS
            }
#endif
        }
    }

    if (is_fact)
        add_resolver(*new activate_fact(slv, *this, atm));
    else
        add_resolver(*new activate_goal(slv, *this, atm));
}

atom_flaw::activate_fact::activate_fact(solver &slv, atom_flaw &f, atom &a) : resolver(slv, 0, f), atm(a) {}
atom_flaw::activate_fact::~activate_fact() {}

void atom_flaw::activate_fact::apply() { slv.sat_cr.new_clause({lit(rho, false), atm.sigma}); }

atom_flaw::activate_goal::activate_goal(solver &slv, atom_flaw &f, atom &a) : resolver(slv, 1, f), atm(a) {}
atom_flaw::activate_goal::~activate_goal() {}

void atom_flaw::activate_goal::apply()
{
    slv.sat_cr.new_clause({lit(rho, false), atm.sigma});
    static_cast<const predicate &>(atm.tp).apply_rule(atm);
}

atom_flaw::unify_atom::unify_atom(solver &slv, atom_flaw &f, atom &atm, atom &trgt, const std::vector<lit> &unif_lits) : resolver(slv, 1, f), atm(atm), trgt(trgt), unif_lits(unif_lits) {}
atom_flaw::unify_atom::~unify_atom() {}

void atom_flaw::unify_atom::apply()
{
    for (const auto &v : unif_lits)
        slv.sat_cr.new_clause({lit(rho, false), v});
}
}