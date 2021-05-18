#include "reusable_resource.h"
#include "solver.h"
#include "atom.h"
#include "field.h"
#include "atom_flaw.h"
#include "combinations.h"
#include "core_parser.h"
#include <stdexcept>
#include <list>

using namespace smt;

namespace ratio
{
    reusable_resource::reusable_resource(solver &slv) : smart_type(slv, slv, REUSABLE_RESOURCE_NAME)
    {
        new_fields(*this, {new field(slv.get_type(STRING_KEYWORD), REUSABLE_RESOURCE_CAPACITY)}); // we add the 'capacity' field..
        new_constructors({new rr_constructor(*this)});                                            // we add a constructor..
        new_predicates({new use_predicate(*this)}, false);                                        // we add the 'Use' predicate, without notifying neither the resource nor its supertypes..
    }
    reusable_resource::~reusable_resource() {}

    std::vector<std::vector<std::pair<lit, double>>> reusable_resource::get_current_incs()
    {
        std::vector<std::vector<std::pair<lit, double>>> incs;
        // we partition atoms for each reusable-resource they might insist on..
        std::unordered_map<const item *, std::vector<atom *>> rr_instances;
        for (const auto &[atm, atm_lstnr] : atoms)
            if (get_core().get_sat_core().value(atm->get_sigma()) == True) // we filter out those which are not strictly active..
            {
                expr c_scope = atm->get(TAU);
                if (var_item *enum_scope = dynamic_cast<var_item *>(&*c_scope))
                {
                    for (const auto &val : get_core().get_ov_theory().value(enum_scope->ev))
                        if (to_check.count(static_cast<const item *>(val)))
                            rr_instances[static_cast<const item *>(val)].push_back(atm);
                }
                else if (to_check.count(static_cast<item *>(&*c_scope)))
                    rr_instances[static_cast<item *>(&*c_scope)].push_back(atm);
            }

        // we detect inconsistencies for each of the instances..
        for (const auto &[rr, atms] : rr_instances)
        {
            // for each pulse, the atoms starting at that pulse..
            std::map<inf_rational, std::set<atom *>> starting_atoms;
            // for each pulse, the atoms ending at that pulse..
            std::map<inf_rational, std::set<atom *>> ending_atoms;
            // all the pulses of the timeline..
            std::set<inf_rational> pulses;
            // the resource capacity..
            arith_expr capacity = rr->get(REUSABLE_RESOURCE_CAPACITY);
            inf_rational c_capacity = get_core().arith_value(capacity);

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

                inf_rational c_usage; // the concurrent resource usage..
                for (const auto &a : overlapping_atoms)
                {
                    arith_expr amount = a->get(REUSABLE_RESOURCE_USE_AMOUNT_NAME);
                    c_usage += get_core().arith_value(amount);
                }

                if (c_usage > c_capacity) // we have a 'peak'..
                {
                    // we extract minimal conflict sets (MCSs)..
                    // we sort the overlapping atoms, according to their resource usage, in descending order..
                    std::vector<atom *> inc_atoms(overlapping_atoms.cbegin(), overlapping_atoms.cend());
                    std::sort(inc_atoms.begin(), inc_atoms.end(), [this](const auto &atm0, const auto &atm1) {
                        arith_expr amnt0 = atm0->get(REUSABLE_RESOURCE_USE_AMOUNT_NAME);
                        arith_expr amnt1 = atm1->get(REUSABLE_RESOURCE_USE_AMOUNT_NAME);
                        return get_core().arith_value(amnt0) > get_core().arith_value(amnt1);
                    });

                    inf_rational mcs_usage;  // the concurrent resource usage of the mcs..
                    std::list<atom *> c_mcs; // the current mcs..
                    auto mcs_begin = inc_atoms.cbegin();
                    auto mcs_end = inc_atoms.cbegin();
                    while (mcs_end != inc_atoms.cend())
                    {
                        // we increase the size of the current mcs..
                        while (mcs_usage <= c_capacity && mcs_end != inc_atoms.cend())
                        {
                            c_mcs.push_back(*mcs_end);
                            arith_expr amount = (*mcs_end)->get(REUSABLE_RESOURCE_USE_AMOUNT_NAME);
                            mcs_usage += get_core().arith_value(amount);
                            ++mcs_end;
                        }

                        if (mcs_usage > c_capacity)
                        { // we have a new mcs..
                            std::set<atom *> mcs(c_mcs.cbegin(), c_mcs.cend());
                            if (!rr_flaws.count(mcs))
                            { // we create a new reusable-resource flaw..
                                rr_flaw *flw = new rr_flaw(*this, mcs);
                                rr_flaws.insert({mcs, flw});
                                store_flaw(*flw); // we store the flaw for retrieval when at root-level..
                            }

                            std::vector<std::pair<lit, double>> choices;
                            for (const auto &as : combinations(std::vector<atom *>(c_mcs.cbegin(), c_mcs.cend()), 2))
                            {
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
                            }
                            incs.push_back(choices);

                            // we decrease the size of the mcs..
                            arith_expr amount = c_mcs.front()->get(REUSABLE_RESOURCE_USE_AMOUNT_NAME);
                            mcs_usage -= get_core().arith_value(amount);
                            assert(mcs_usage <= c_capacity);
                            c_mcs.pop_front();
                            ++mcs_begin;
                        }
                    }
                }
            }
        }
        return incs;
    }

    void reusable_resource::new_predicate(predicate &) noexcept { assert(false); }

    void reusable_resource::new_atom(atom_flaw &f)
    {
        assert(f.is_fact);
        atom &atm = f.get_atom();
        // we apply interval-predicate whenever the fact becomes active..
        set_ni(lit(atm.get_sigma()));
        static_cast<predicate &>(get_predicate(REUSABLE_RESOURCE_USE_PREDICATE_NAME)).apply_rule(atm);
        restore_ni();

        // we avoid unification..
        if (!get_core().get_sat_core().new_clause({!f.get_phi(), lit(atm.get_sigma())}))
            throw unsolvable_exception();

        // we store the variables for on-line flaw resolution..
        for (const auto &c_atm : atoms)
            store_variables(atm, *c_atm.first);

        // we store, for the atom, its atom listener..
        atoms.push_back({&atm, new rr_atom_listener(*this, atm)});

        // we filter out those atoms which are not strictly active..
        if (get_core().get_sat_core().value(atm.get_sigma()) == True)
        {
            expr c_scope = atm.get(TAU);
            if (var_item *enum_scope = dynamic_cast<var_item *>(&*c_scope))              // the 'tau' parameter is a variable..
                for (const auto &val : get_core().get_ov_theory().value(enum_scope->ev)) // we check for all its allowed values..
                    to_check.insert(static_cast<const item *>(val));
            else // the 'tau' parameter is a constant..
                to_check.insert(&*c_scope);
        }
    }

    void reusable_resource::store_variables(atom &atm0, atom &atm1)
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
        { // the two atoms are on the same reusable-resource: we store the ordering variables..
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

    reusable_resource::rr_constructor::rr_constructor(reusable_resource &rr) : constructor(rr.get_core(), rr, {new field(rr.get_core().get_type(REAL_KEYWORD), REUSABLE_RESOURCE_CAPACITY)}, {{REUSABLE_RESOURCE_CAPACITY, {static_cast<const ast::expression *>(new ast::id_expression({riddle::id_token(0, 0, 0, 0, REUSABLE_RESOURCE_CAPACITY)}))}}}, {static_cast<const ast::statement *>(new ast::expression_statement(static_cast<const ast::expression *>(new ast::geq_expression(static_cast<const ast::expression *>(new ast::id_expression({riddle::id_token(0, 0, 0, 0, REUSABLE_RESOURCE_CAPACITY)})), static_cast<const ast::expression *>(new ast::real_literal_expression(riddle::real_token(0, 0, 0, 0, rational::ZERO)))))))}) {}
    reusable_resource::rr_constructor::~rr_constructor() {}

    reusable_resource::use_predicate::use_predicate(reusable_resource &rr) : predicate(rr.get_core(), rr, REUSABLE_RESOURCE_USE_PREDICATE_NAME, {new field(rr.get_core().get_type(REAL_KEYWORD), REUSABLE_RESOURCE_USE_AMOUNT_NAME)}, {static_cast<const ast::statement *>(new ast::expression_statement(static_cast<const ast::expression *>(new ast::geq_expression(static_cast<const ast::expression *>(new ast::id_expression({riddle::id_token(0, 0, 0, 0, REUSABLE_RESOURCE_USE_AMOUNT_NAME)})), static_cast<const ast::expression *>(new ast::real_literal_expression(riddle::real_token(0, 0, 0, 0, rational::ZERO)))))))}) { new_supertypes({&rr.get_core().get_predicate("Interval")}); }
    reusable_resource::use_predicate::~use_predicate() {}

    reusable_resource::rr_atom_listener::rr_atom_listener(reusable_resource &rr, atom &atm) : atom_listener(atm), rr(rr) {}
    reusable_resource::rr_atom_listener::~rr_atom_listener() {}

    void reusable_resource::rr_atom_listener::something_changed()
    {
        // we filter out those atoms which are not strictly active..
        if (atm.get_core().get_sat_core().value(atm.get_sigma()) == True)
        {
            expr c_scope = atm.get(TAU);
            if (var_item *enum_scope = dynamic_cast<var_item *>(&*c_scope))                  // the 'tau' parameter is a variable..
                for (const auto &val : atm.get_core().get_ov_theory().value(enum_scope->ev)) // we check for all its allowed values..
                    rr.to_check.insert(static_cast<const item *>(val));
            else // the 'tau' parameter is a constant..
                rr.to_check.insert(&*c_scope);
        }
    }

    reusable_resource::rr_flaw::rr_flaw(reusable_resource &rr, const std::set<atom *> &atms) : flaw(rr.get_solver(), smart_type::get_resolvers(rr.get_solver(), atms), {}), rr(rr), overlapping_atoms(atms) {}
    reusable_resource::rr_flaw::~rr_flaw() {}

    std::string reusable_resource::rr_flaw::get_label() const
    {
        std::string lbl = "{\"type\":\"rr-flaw\", \"phi\":\"" + to_string(get_phi()) + "\", \"position\":" + std::to_string(get_position()) + ", \"atoms\":[";
        for (auto as_it = overlapping_atoms.cbegin(); as_it != overlapping_atoms.cend(); ++as_it)
        {
            if (as_it != overlapping_atoms.cbegin())
                lbl += ", ";
            lbl += "\"" + std::to_string(reinterpret_cast<uintptr_t>(*as_it)) + "\"";
        }
        lbl += "]}";
        return lbl;
    }

    void reusable_resource::rr_flaw::compute_resolvers()
    {
        const auto cs = combinations(std::vector<atom *>(overlapping_atoms.cbegin(), overlapping_atoms.cend()), 2);
        for (const auto &as : cs)
        {
            if (auto a0_it = rr.leqs.find(as[0]); a0_it != rr.leqs.cend())
                if (auto a0_a1_it = a0_it->second.find(as[1]); a0_a1_it != a0_it->second.cend())
                    if (get_solver().get_sat_core().value(a0_a1_it->second) != False)
                        add_resolver(*new order_resolver(*this, a0_a1_it->second, *as[0], *as[1]));

            if (auto a1_it = rr.leqs.find(as[1]); a1_it != rr.leqs.cend())
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
            else if (auto a0_a1_it = rr.plcs.find({as[0], as[1]}); a0_a1_it != rr.plcs.cend())
                for (const auto &a0_a1_disp : a0_a1_it->second)
                    if (get_solver().get_sat_core().value(a0_a1_disp.first) != False)
                        add_resolver(*new place_resolver(*this, a0_a1_disp.first, *as[0], *a0_a1_disp.second, *as[1]));
        }
    }

    reusable_resource::order_resolver::order_resolver(rr_flaw &flw, const lit &r, const atom &before, const atom &after) : resolver(flw.get_solver(), r, rational::ZERO, flw), before(before), after(after) {}
    reusable_resource::order_resolver::~order_resolver() {}

    std::string reusable_resource::order_resolver::get_label() const { return "{\"type\":\"order\", \"rho\":\"" + to_string(get_rho()) + "\", \"before_sigma\":" + std::to_string(before.get_sigma()) + ", \"after_sigma\":" + std::to_string(after.get_sigma()) + ", \"before_atom\":\"" + std::to_string(reinterpret_cast<uintptr_t>(&before)) + "\", \"after_atom\":\"" + std::to_string(reinterpret_cast<uintptr_t>(&after)) + "\"}"; }

    void reusable_resource::order_resolver::apply() {}

    reusable_resource::place_resolver::place_resolver(rr_flaw &flw, const lit &r, atom &plc_atm, const item &plc_itm, atom &frbd_atm) : resolver(flw.get_solver(), r, rational::ZERO, flw), plc_atm(plc_atm), plc_itm(plc_itm), frbd_atm(frbd_atm) {}
    reusable_resource::place_resolver::~place_resolver() {}

    std::string reusable_resource::place_resolver::get_label() const { return "{\"type\":\"place\", \"rho\":\"" + to_string(get_rho()) + "\", \"place_sigma\":" + std::to_string(plc_atm.get_sigma()) + ", \"forbid_sigma\":" + std::to_string(frbd_atm.get_sigma()) + ", \"place_atom\":\"" + std::to_string(reinterpret_cast<uintptr_t>(&plc_atm)) + "\", \"forbid_atom\":\"" + std::to_string(reinterpret_cast<uintptr_t>(&frbd_atm)) + "\"}"; }

    void reusable_resource::place_resolver::apply() {}

    reusable_resource::forbid_resolver::forbid_resolver(rr_flaw &flw, atom &atm, item &itm) : resolver(flw.get_solver(), rational::ZERO, flw), atm(atm), itm(itm) {}
    reusable_resource::forbid_resolver::~forbid_resolver() {}

    std::string reusable_resource::forbid_resolver::get_label() const { return "{\"type\":\"forbid\", \"rho\":\"" + to_string(get_rho()) + "\", \"atom_sigma\":" + std::to_string(atm.get_sigma()) + ", \"atom\":\"" + std::to_string(reinterpret_cast<uintptr_t>(&atm)) + "\"}"; }

    void reusable_resource::forbid_resolver::apply() { get_solver().get_sat_core().new_clause({!get_rho(), !get_solver().get_ov_theory().allows(static_cast<var_item *>(&*atm.get(TAU))->ev, itm)}); }
} // namespace ratio