#include "state_variable.h"
#include "solver.h"
#include "atom.h"
#include "predicate.h"
#include "field.h"
#include "atom_flaw.h"
#include "combinations.h"

using namespace smt;

namespace ratio
{
    state_variable::state_variable(solver &slv) : smart_type(slv, slv, STATE_VARIABLE_NAME), int_pred(slv.get_predicate("Interval")) { new_constructors({new sv_constructor(*this)}); }
    state_variable::~state_variable()
    {
        // we clear the atom listeners..
        for (const auto &[atm, lstnrs] : atoms)
            delete lstnrs;
    }

    std::vector<std::vector<std::pair<lit, double>>> state_variable::get_current_incs()
    {
        std::vector<std::vector<std::pair<lit, double>>> incs;
        // we partition atoms for each state-variable they might insist on..
        std::unordered_map<const item *, std::vector<atom *>> sv_instances;
        for (const auto &[atm, atm_lstnr] : atoms)
            if (get_core().get_sat_core().value(atm->get_sigma()) == True) // we filter out those which are not strictly active..
            {
                expr c_scope = atm->get(TAU);
                if (var_item *enum_scope = dynamic_cast<var_item *>(&*c_scope))
                {
                    for (const auto &val : get_core().get_ov_theory().value(enum_scope->ev))
                        if (to_check.count(static_cast<const item *>(val)))
                            sv_instances[static_cast<const item *>(val)].push_back(atm);
                }
                else if (to_check.count(static_cast<item *>(&*c_scope)))
                    sv_instances[static_cast<item *>(&*c_scope)].push_back(atm);
            }

        // we detect inconsistencies for each of the instances..
        for (const auto &[sv, atms] : sv_instances)
        {
            // for each pulse, the atoms starting at that pulse..
            std::map<inf_rational, std::set<atom *>> starting_atoms;
            // for each pulse, the atoms ending at that pulse..
            std::map<inf_rational, std::set<atom *>> ending_atoms;
            // all the pulses of the timeline..
            std::set<inf_rational> pulses;

            for (const auto &atm : atms)
            {
                arith_expr s_expr = atm->get(START);
                arith_expr e_expr = atm->get(END);
                inf_rational start = get_core().arith_value(s_expr);
                inf_rational end = get_core().arith_value(e_expr);
                starting_atoms[start].insert(atm);
                ending_atoms[end].insert(atm);
                pulses.insert(start);
                pulses.insert(end);
            }

            std::set<atom *> overlapping_atoms;
            for (const auto &p : pulses)
            {
                if (const auto at_start_p = starting_atoms.find(p); at_start_p != starting_atoms.cend())
                    overlapping_atoms.insert(at_start_p->second.cbegin(), at_start_p->second.cend());
                if (const auto at_end_p = ending_atoms.find(p); at_end_p != ending_atoms.cend())
                    for (const auto &a : at_end_p->second)
                        overlapping_atoms.erase(a);

                if (overlapping_atoms.size() > 1) // we have a 'peak'..
                    for (const auto &as : combinations(std::vector<atom *>(overlapping_atoms.cbegin(), overlapping_atoms.cend()), 2))
                    { // state-variable MCSs are made of two atoms..
                        std::set<atom *> mcs(as.cbegin(), as.cend());
                        if (!sv_flaws.count(mcs))
                        { // we create a new state-variable flaw..
                            sv_flaw *flw = new sv_flaw(*this, mcs);
                            sv_flaws.insert({mcs, flw});
                            store_flaw(*flw); // we store the flaw for retrieval when at root-level..
                        }

                        std::vector<std::pair<lit, double>> choices;
                        arith_expr a0_start = as[0]->get(START);
                        arith_expr a0_end = as[0]->get(END);
                        arith_expr a1_start = as[1]->get(START);
                        arith_expr a1_end = as[1]->get(END);

                        if (auto a0_it = leqs.find(as[0]); a0_it != leqs.cend())
                            if (auto a0_a1_it = a0_it->second.find(as[1]); a0_a1_it != a0_it->second.cend())
                                if (get_solver().get_sat_core().value(a0_a1_it->second) != False)
                                {
#ifdef DL_TN
                                    const auto [min, max] = get_solver().get_rdl_theory().distance(a0_end->l, a1_start->l);
                                    const auto commit = is_infinite(min) || is_infinite(max) ? 0.5 : (std::min(static_cast<double>(max.get_rational()), 0.0) - std::min(static_cast<double>(min.get_rational()), 0.0)) / (static_cast<double>(max.get_rational()) - static_cast<double>(min.get_rational()));
#elif LA_TN
                                    const auto work = (get_solver().arith_value(a1_end).get_rational() - get_solver().arith_value(a1_start).get_rational()) * (get_solver().arith_value(a0_end).get_rational() - get_solver().arith_value(a1_start).get_rational());
                                    const auto commit = work == rational::ZERO ? -std::numeric_limits<double>::max() : 1l - 1l / (static_cast<double>(work.numerator()) / work.denominator());
#endif
                                    choices.push_back({a0_a1_it->second, commit});
                                }

                        if (auto a1_it = leqs.find(as[1]); a1_it != leqs.cend())
                            if (auto a1_a0_it = a1_it->second.find(as[0]); a1_a0_it != a1_it->second.cend())
                                if (get_solver().get_sat_core().value(a1_a0_it->second) != False)
                                {
#ifdef DL_TN
                                    const auto [min, max] = get_solver().get_rdl_theory().distance(a1_end->l, a0_start->l);
                                    const auto commit = is_infinite(min) || is_infinite(max) ? 0.5 : (std::min(static_cast<double>(max.get_rational()), 0.0) - std::min(static_cast<double>(min.get_rational()), 0.0)) / (static_cast<double>(max.get_rational()) - static_cast<double>(min.get_rational()));
#elif LA_TN
                                    const auto work = (get_solver().arith_value(a0_end).get_rational() - get_solver().arith_value(a0_start).get_rational()) * (get_solver().arith_value(a1_end).get_rational() - get_solver().arith_value(a0_start).get_rational());
                                    const auto commit = work == rational::ZERO ? -std::numeric_limits<double>::max() : 1l - 1l / (static_cast<double>(work.numerator()) / work.denominator());
#endif
                                    choices.push_back({a1_a0_it->second, commit});
                                }

                        expr a0_tau = as[0]->get(TAU);
                        expr a1_tau = as[1]->get(TAU);
                        var_item *a0_tau_itm = dynamic_cast<var_item *>(&*a0_tau);
                        var_item *a1_tau_itm = dynamic_cast<var_item *>(&*a1_tau);
                        if (a0_tau_itm && a1_tau_itm)
                        { // we have two non-singleton variables..
                            auto a0_vals = get_solver().enum_value(a0_tau_itm);
                            auto a1_vals = get_solver().enum_value(a1_tau_itm);
                            for (const auto &plc : plcs.at({as[0], as[1]}))
                                if (get_solver().get_sat_core().value(plc.first) == Undefined)
                                    choices.push_back({plc.first, 1l - 2l / static_cast<double>(a0_vals.size() + a1_vals.size())});
                        }
                        else if (a0_tau_itm)
                        { // only 'a1_tau' is a singleton variable..
                            if (auto a0_vals = get_solver().enum_value(a0_tau_itm); a0_vals.count(&*a1_tau))
                                if (get_solver().get_sat_core().value(get_solver().get_ov_theory().allows(static_cast<var_item *>(a0_tau_itm)->ev, *a1_tau)) == Undefined)
                                    choices.push_back({!get_solver().get_ov_theory().allows(static_cast<var_item *>(a0_tau_itm)->ev, *a1_tau), 1l - 1l / static_cast<double>(a0_vals.size())});
                        }
                        else if (a1_tau_itm)
                        { // only 'a0_tau' is a singleton variable..
                            if (auto a1_vals = get_solver().enum_value(a1_tau_itm); a1_vals.count(&*a0_tau))
                                if (get_solver().get_sat_core().value(get_solver().get_ov_theory().allows(static_cast<var_item *>(a1_tau_itm)->ev, *a0_tau)) == Undefined)
                                    choices.push_back({!get_solver().get_ov_theory().allows(static_cast<var_item *>(a1_tau_itm)->ev, *a0_tau), 1l - 1l / static_cast<double>(a1_vals.size())});
                        }
                        incs.push_back(choices);
                    }
            }
        }
        return incs;
    }

    void state_variable::new_predicate(predicate &pred) noexcept
    {
        // each state-variable predicate is also an interval-predicate..
        new_supertypes(pred, {const_cast<predicate *>(&int_pred)});
        // each state-variable predicate has a tau parameter indicating on which state-variables the atoms insist on..
        new_fields(pred, {new field(static_cast<type &>(pred.get_scope()), TAU)});
    }

    void state_variable::new_atom(atom_flaw &f)
    {
        atom &atm = f.get_atom();
        if (f.is_fact)
        { // we apply interval-predicate whenever the fact becomes active..
            set_ni(lit(atm.get_sigma()));
            int_pred.apply_rule(atm);
            restore_ni();
        }

        // we store the variables for on-line flaw resolution..
        for (const auto &c_atm : atoms)
            store_variables(atm, *c_atm.first);

        // we store, for the atom, its atom listener..
        atoms.push_back({&atm, new sv_atom_listener(*this, atm)});

        // we filter out those atoms which are not strictly active..
        if (atm.get_core().get_sat_core().value(atm.get_sigma()) == True)
        {
            expr c_scope = atm.get(TAU);
            if (var_item *enum_scope = dynamic_cast<var_item *>(&*c_scope))                  // the 'tau' parameter is a variable..
                for (const auto &val : atm.get_core().get_ov_theory().value(enum_scope->ev)) // we check for all its allowed values..
                    to_check.insert(static_cast<const item *>(val));
            else // the 'tau' parameter is a constant..
                to_check.insert(&*c_scope);
        }
    }

    void state_variable::store_variables(atom &atm0, atom &atm1)
    {
        arith_expr a0_start = atm0.get(START);
        arith_expr a0_end = atm0.get(END);
        arith_expr a1_start = atm1.get(START);
        arith_expr a1_end = atm1.get(END);

        expr a0_tau = atm0.get(TAU);
        expr a1_tau = atm1.get(TAU);
        var_item *a0_tau_itm = dynamic_cast<var_item *>(&*a0_tau);
        var_item *a1_tau_itm = dynamic_cast<var_item *>(&*a1_tau);
        if (a0_tau_itm && a1_tau_itm)
        { // we have two non-singleton variables..
            auto a0_vals = get_solver().enum_value(a0_tau_itm);
            auto a1_vals = get_solver().enum_value(a1_tau_itm);

            bool found = false;
            for (const auto &v0 : a0_vals)
                if (a1_vals.count(v0))
                {
                    if (!found)
                    { // we store the ordering variables..
#ifdef DL_TN
                        leqs[&atm0][&atm1] = get_solver().get_rdl_theory().new_leq(a0_end->l, a1_start->l);
                        leqs[&atm1][&atm0] = get_solver().get_rdl_theory().new_leq(a1_end->l, a0_start->l);
#elif LA_TN
                        leqs[&atm0][&atm1] = get_solver().get_lra_theory().new_leq(a0_end->l, a1_start->l);
                        leqs[&atm1][&atm0] = get_solver().get_lra_theory().new_leq(a1_end->l, a0_start->l);
                        // we boost propagation..
                        bool nc = get_solver().get_sat_core().new_clause({!leqs[&atm0][&atm1], !leqs[&atm1][&atm0]});
                        assert(nc);
#endif
                        found = true;
                    }
                    plcs[{&atm0, &atm1}].push_back({get_solver().get_sat_core().new_conj({get_solver().get_ov_theory().allows(a0_tau_itm->ev, *v0), !get_solver().get_ov_theory().allows(a1_tau_itm->ev, *v0)}), static_cast<const item *>(v0)});
                }
        }
        else if (a0_tau_itm)
        { // only 'a1_tau' is a singleton variable..
            if (auto a0_vals = get_solver().enum_value(a0_tau_itm); a0_vals.count(&*a1_tau))
            { // we store the ordering variables..
#ifdef DL_TN
                leqs[&atm0][&atm1] = get_solver().get_rdl_theory().new_leq(a0_end->l, a1_start->l);
                leqs[&atm1][&atm0] = get_solver().get_rdl_theory().new_leq(a1_end->l, a0_start->l);
#elif LA_TN
                leqs[&atm0][&atm1] = get_solver().get_lra_theory().new_leq(a0_end->l, a1_start->l);
                leqs[&atm1][&atm0] = get_solver().get_lra_theory().new_leq(a1_end->l, a0_start->l);
                // we boost propagation..
                bool nc = get_solver().get_sat_core().new_clause({!leqs[&atm0][&atm1], !leqs[&atm1][&atm0]});
                assert(nc);
#endif
            }
        }
        else if (a1_tau_itm)
        { // only 'a0_tau' is a singleton variable..
            if (auto a1_vals = get_solver().enum_value(a1_tau_itm); a1_vals.count(&*a0_tau))
            { // we store the ordering variables..
#ifdef DL_TN
                leqs[&atm0][&atm1] = get_solver().get_rdl_theory().new_leq(a0_end->l, a1_start->l);
                leqs[&atm1][&atm0] = get_solver().get_rdl_theory().new_leq(a1_end->l, a0_start->l);
#elif LA_TN
                leqs[&atm0][&atm1] = get_solver().get_lra_theory().new_leq(a0_end->l, a1_start->l);
                leqs[&atm1][&atm0] = get_solver().get_lra_theory().new_leq(a1_end->l, a0_start->l);
                // we boost propagation..
                bool nc = get_solver().get_sat_core().new_clause({!leqs[&atm0][&atm1], !leqs[&atm1][&atm0]});
                assert(nc);
#endif
            }
        }
        else if (&*a0_tau == &*a1_tau)
        { // the two atoms are on the same state-variable: we store the ordering variables..
#ifdef DL_TN
            leqs[&atm0][&atm1] = get_solver().get_rdl_theory().new_leq(a0_end->l, a1_start->l);
            leqs[&atm1][&atm0] = get_solver().get_rdl_theory().new_leq(a1_end->l, a0_start->l);
#elif LA_TN
            leqs[&atm0][&atm1] = get_solver().get_lra_theory().new_leq(a0_end->l, a1_start->l);
            leqs[&atm1][&atm0] = get_solver().get_lra_theory().new_leq(a1_end->l, a0_start->l);
            // we boost propagation..
            bool nc = get_solver().get_sat_core().new_clause({!leqs[&atm0][&atm1], !leqs[&atm1][&atm0]});
            assert(nc);
#endif
        }
    }

    state_variable::sv_constructor::sv_constructor(state_variable &sv) : constructor(sv.get_solver(), sv, {}, {}, {}) {}
    state_variable::sv_constructor::~sv_constructor() {}

    state_variable::sv_atom_listener::sv_atom_listener(state_variable &sv, atom &atm) : atom_listener(atm), sv(sv) { something_changed(); }
    state_variable::sv_atom_listener::~sv_atom_listener() {}

    void state_variable::sv_atom_listener::something_changed()
    {
        // we filter out those atoms which are not strictly active..
        if (atm.get_core().get_sat_core().value(atm.get_sigma()) == True)
        {
            expr c_scope = atm.get(TAU);
            if (var_item *enum_scope = dynamic_cast<var_item *>(&*c_scope))                  // the 'tau' parameter is a variable..
                for (const auto &val : atm.get_core().get_ov_theory().value(enum_scope->ev)) // we check for all its allowed values..
                    sv.to_check.insert(static_cast<const item *>(val));
            else // the 'tau' parameter is a constant..
                sv.to_check.insert(&*c_scope);
        }
    }

    state_variable::sv_flaw::sv_flaw(state_variable &sv, const std::set<atom *> &atms) : flaw(sv.get_solver(), smart_type::get_resolvers(sv.get_solver(), atms), {}), sv(sv), overlapping_atoms(atms) {}
    state_variable::sv_flaw::~sv_flaw() {}

    std::string state_variable::sv_flaw::get_label() const
    {
        std::string lbl = "{\"type\":\"sv-flaw\", \"phi\":\"" + to_string(get_phi()) + "\", \"position\":" + std::to_string(get_position()) + ", \"atoms\":[";
        for (auto as_it = overlapping_atoms.cbegin(); as_it != overlapping_atoms.cend(); ++as_it)
        {
            if (as_it != overlapping_atoms.cbegin())
                lbl += ", ";
            lbl += "\"" + std::to_string(reinterpret_cast<uintptr_t>(*as_it)) + "\"";
        }
        lbl += "]}";
        return lbl;
    }

    void state_variable::sv_flaw::compute_resolvers()
    {
        const auto cs = combinations(std::vector<atom *>(overlapping_atoms.cbegin(), overlapping_atoms.cend()), 2);
        for (const auto &as : cs)
        {
            if (auto a0_it = sv.leqs.find(as[0]); a0_it != sv.leqs.cend())
                if (auto a0_a1_it = a0_it->second.find(as[1]); a0_a1_it != a0_it->second.cend())
                    if (get_solver().get_sat_core().value(a0_a1_it->second) != False)
                        add_resolver(*new order_resolver(*this, a0_a1_it->second, *as[0], *as[1]));

            if (auto a1_it = sv.leqs.find(as[1]); a1_it != sv.leqs.cend())
                if (auto a1_a0_it = a1_it->second.find(as[0]); a1_a0_it != a1_it->second.cend())
                    if (get_solver().get_sat_core().value(a1_a0_it->second) != False)
                        add_resolver(*new order_resolver(*this, a1_a0_it->second, *as[1], *as[0]));

            expr a0_tau = as[0]->get(TAU);
            expr a1_tau = as[1]->get(TAU);
            var_item *a0_tau_itm = dynamic_cast<var_item *>(&*a0_tau);
            var_item *a1_tau_itm = dynamic_cast<var_item *>(&*a1_tau);
            if (a0_tau_itm && !a1_tau_itm)
                add_resolver(*new forbid_resolver(*this, *as[0], *a1_tau));
            else if (!a0_tau_itm && a1_tau_itm)
                add_resolver(*new forbid_resolver(*this, *as[1], *a0_tau));
            else if (auto a0_a1_it = sv.plcs.find({as[0], as[1]}); a0_a1_it != sv.plcs.cend())
                for (const auto &a0_a1_disp : a0_a1_it->second)
                    if (get_solver().get_sat_core().value(a0_a1_disp.first) != False)
                        add_resolver(*new place_resolver(*this, a0_a1_disp.first, *as[0], *a0_a1_disp.second, *as[1]));
        }
    }

    state_variable::order_resolver::order_resolver(sv_flaw &flw, const lit &r, const atom &before, const atom &after) : resolver(flw.get_solver(), r, rational::ZERO, flw), before(before), after(after) {}
    state_variable::order_resolver::~order_resolver() {}

    std::string state_variable::order_resolver::get_label() const { return "{\"type\":\"order\", \"rho\":\"" + to_string(get_rho()) + "\", \"before_sigma\":" + std::to_string(before.get_sigma()) + ", \"after_sigma\":" + std::to_string(after.get_sigma()) + ", \"before_atom\":\"" + std::to_string(reinterpret_cast<uintptr_t>(&before)) + "\", \"after_atom\":\"" + std::to_string(reinterpret_cast<uintptr_t>(&after)) + "\"}"; }

    void state_variable::order_resolver::apply() {}

    state_variable::place_resolver::place_resolver(sv_flaw &flw, const lit &r, atom &plc_atm, const item &plc_itm, atom &frbd_atm) : resolver(flw.get_solver(), r, rational::ZERO, flw), plc_atm(plc_atm), plc_itm(plc_itm), frbd_atm(frbd_atm) {}
    state_variable::place_resolver::~place_resolver() {}

    std::string state_variable::place_resolver::get_label() const { return "{\"type\":\"place\", \"rho\":\"" + to_string(get_rho()) + "\", \"place_sigma\":" + std::to_string(plc_atm.get_sigma()) + ", \"forbid_sigma\":" + std::to_string(frbd_atm.get_sigma()) + ", \"place_atom\":\"" + std::to_string(reinterpret_cast<uintptr_t>(&plc_atm)) + "\", \"forbid_atom\":\"" + std::to_string(reinterpret_cast<uintptr_t>(&frbd_atm)) + "\"}"; }

    void state_variable::place_resolver::apply() {}

    state_variable::forbid_resolver::forbid_resolver(sv_flaw &flw, atom &atm, item &itm) : resolver(flw.get_solver(), rational::ZERO, flw), atm(atm), itm(itm) {}
    state_variable::forbid_resolver::~forbid_resolver() {}

    std::string state_variable::forbid_resolver::get_label() const { return "{\"type\":\"forbid\", \"rho\":\"" + to_string(get_rho()) + "\", \"atom_sigma\":" + std::to_string(atm.get_sigma()) + ", \"atom\":\"" + std::to_string(reinterpret_cast<uintptr_t>(&atm)) + "\"}"; }

    void state_variable::forbid_resolver::apply() { get_solver().get_sat_core().new_clause({!get_rho(), !get_solver().get_ov_theory().allows(static_cast<var_item *>(&*atm.get(TAU))->ev, itm)}); }
} // namespace ratio