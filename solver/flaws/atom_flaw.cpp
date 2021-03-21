#include "atom_flaw.h"
#include "solver.h"
#include "atom.h"
#include "predicate.h"
#include <cassert>

using namespace smt;

namespace ratio
{
    atom_flaw::atom_flaw(solver &slv, const std::vector<resolver *> &causes, atom &atm, const bool is_fact) : flaw(slv, causes, true), atm(atm), is_fact(is_fact) {}
    atom_flaw::~atom_flaw() {}

    std::string atom_flaw::get_label() const noexcept
    {
        if (is_fact)
            return "{\"type\":\"fact\", \"phi\":\"" + to_string(get_phi()) + "\", \"position\":" + std::to_string(get_position()) + ", \"sigma\":" + std::to_string(atm.get_sigma()) + ", \"predicate\":\"" + atm.get_type().get_name() + "\", \"atom\":\"" + std::to_string(reinterpret_cast<uintptr_t>(&atm)) + "\"}";
        else
            return "{\"type\":\"goal\", \"phi\":\"" + to_string(get_phi()) + "\", \"position\":" + std::to_string(get_position()) + ", \"sigma\":" + std::to_string(atm.get_sigma()) + ", \"predicate\":\"" + atm.get_type().get_name() + "\", \"atom\":\"" + std::to_string(reinterpret_cast<uintptr_t>(&atm)) + "\"}";
    }

    void atom_flaw::compute_resolvers()
    {
        assert(get_solver().get_sat_core().value(get_phi()) != False);
        assert(get_solver().get_sat_core().value(atm.get_sigma()) != False);
        if (get_solver().get_sat_core().value(atm.get_sigma()) == Undefined) // we check if the atom can unify..
            for (const auto &i : atm.get_type().get_instances())
            { // we check for possible unifications (i.e. all the instances of the atom's type)..
                if (&*i == &atm)
                    continue; // the current atom cannot unify with itself..

                // this is the target (i.e. the atom we are trying to unify with)..
                atom &t_atm = static_cast<atom &>(*i);

                // this is the target flaw (i.e. the one we are trying to unify with) and cannot be in the current flaw's causes' effects..
                atom_flaw &t_flaw = get_solver().get_reason(t_atm);

                if (!t_flaw.is_expanded() ||                                                                   // the target flaw must hav been already expanded..
                    get_solver().get_idl_theory().distance(get_position(), t_flaw.get_position()).first > 0 || // unifying with the target atom would introduce cyclic causality..
                    get_solver().get_sat_core().value(t_atm.get_sigma()) == False ||                           // the target atom is unified with some other atom..
                    !atm.equates(t_atm))                                                                       // the atom does not equate with the target target..
                    continue;

                // the equality propositional literal..
                lit eq_lit = atm.new_eq(t_atm);

                if (get_solver().get_sat_core().value(eq_lit) == False)
                    continue; // the two atoms cannot unify, hence, we skip this instance..

                unify_atom *u_res = new unify_atom(get_solver(), *this, atm, t_atm, {lit(atm.get_sigma(), false), lit(t_atm.get_sigma()), eq_lit});
                assert(get_solver().get_sat_core().value(u_res->get_rho()) != False);
                add_resolver(*u_res);
                get_solver().new_causal_link(t_flaw, *u_res);
            }

        if (is_fact)
            if (get_resolvers().empty())
                add_resolver(*new activate_fact(get_solver(), get_phi(), *this, atm));
            else
                add_resolver(*new activate_fact(get_solver(), *this, atm));
        else if (get_resolvers().empty())
            add_resolver(*new activate_goal(get_solver(), get_phi(), *this, atm));
        else
            add_resolver(*new activate_goal(get_solver(), *this, atm));
    }

    atom_flaw::activate_fact::activate_fact(solver &slv, atom_flaw &f, atom &a) : resolver(slv, rational::ZERO, f), atm(a) {}
    atom_flaw::activate_fact::activate_fact(solver &slv, const lit &r, atom_flaw &f, atom &a) : resolver(slv, r, rational::ZERO, f), atm(a) {}
    atom_flaw::activate_fact::~activate_fact() {}

    std::string atom_flaw::activate_fact::get_label() const noexcept { return "{\"type\":\"activate\", \"rho\":\"" + to_string(get_rho()) + "\", \"atom\":\"" + std::to_string(reinterpret_cast<uintptr_t>(&atm)) + "\"}"; }

    void atom_flaw::activate_fact::apply()
    { // activating this resolver activates the fact..
        if (!get_solver().get_sat_core().new_clause({!get_rho(), lit(atm.get_sigma())}))
            throw unsolvable_exception();
    }

    atom_flaw::activate_goal::activate_goal(solver &slv, atom_flaw &f, atom &a) : resolver(slv, rational::ONE, f), atm(a) {}
    atom_flaw::activate_goal::activate_goal(solver &slv, const lit &r, atom_flaw &f, atom &a) : resolver(slv, r, rational::ONE, f), atm(a) {}
    atom_flaw::activate_goal::~activate_goal() {}

    std::string atom_flaw::activate_goal::get_label() const noexcept { return "{\"type\":\"activate\", \"rho\":\"" + to_string(get_rho()) + "\", \"atom\":\"" + std::to_string(reinterpret_cast<uintptr_t>(&atm)) + "\"}"; }

    void atom_flaw::activate_goal::apply()
    { // activating this resolver activates the goal..
        if (!get_solver().get_sat_core().new_clause({!get_rho(), lit(atm.get_sigma())}))
            throw unsolvable_exception();
        // we also apply the rule..
        static_cast<const predicate &>(atm.get_type()).apply_rule(atm);
    }

    atom_flaw::unify_atom::unify_atom(solver &slv, atom_flaw &f, atom &atm, atom &trgt, const std::vector<lit> &unif_lits) : resolver(slv, rational::ONE, f), atm(atm), trgt(trgt), unif_lits(unif_lits) {}
    atom_flaw::unify_atom::~unify_atom() {}

    std::string atom_flaw::unify_atom::get_label() const noexcept { return "{\"type\":\"unify\", \"rho\":\"" + to_string(get_rho()) + "\", \"atom\":\"" + std::to_string(reinterpret_cast<uintptr_t>(&atm)) + "\", \"target\":\"" + std::to_string(reinterpret_cast<uintptr_t>(&trgt)) + "\"}"; }

    void atom_flaw::unify_atom::apply()
    {
        // this is the target flaw..
        atom_flaw &t_flaw = get_solver().get_reason(trgt);
        assert(t_flaw.is_expanded());
        for (const auto &r : t_flaw.get_resolvers())
            if (activate_fact *act_f = dynamic_cast<activate_fact *>(r))
            { // we disable this unification if the target fact is not activable..
                if (!get_solver().get_sat_core().new_clause({act_f->get_rho(), !get_rho()}))
                    throw unsolvable_exception();
            }
            else if (activate_goal *act_g = dynamic_cast<activate_goal *>(r))
            { // we disable this unification if the target goal is not activable..
                if (!get_solver().get_sat_core().new_clause({act_g->get_rho(), !get_rho()}))
                    throw unsolvable_exception();
            }

        // activating this resolver entails the unification..
        for (const auto &v : unif_lits)
            if (!get_solver().get_sat_core().new_clause({!get_rho(), v}))
                throw unsolvable_exception();
    }
} // namespace ratio