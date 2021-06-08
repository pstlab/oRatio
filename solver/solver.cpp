#include "solver.h"
#include "init.h"
#if defined(H_MAX) || defined(H_ADD)
#include "h_1.h"
#define HEURISTIC *new h_1(*this)
#endif
#include "bool_flaw.h"
#include "disj_flaw.h"
#include "var_flaw.h"
#include "atom_flaw.h"
#include "disjunction_flaw.h"
#include "smart_type.h"
#include "riddle_lexer.h"
#include "atom.h"
#include "state_variable.h"
#include "reusable_resource.h"
#include "agent.h"
#include "propositional_state.h"
#ifdef BUILD_LISTENERS
#include "solver_listener.h"
#endif
#include <algorithm>
#include <math.h>
#include <cassert>

using namespace smt;

namespace ratio
{
    SOLVER_EXPORT solver::solver() : core(), theory(get_sat_core()), heur(HEURISTIC) {}
    SOLVER_EXPORT solver::~solver() { delete &heur; }

    SOLVER_EXPORT void solver::read(const std::string &script)
    {
        core::read(script);
        reset_smart_types();
    }
    SOLVER_EXPORT void solver::read(const std::vector<std::string> &files)
    {
        core::read(files);
        reset_smart_types();
    }

    SOLVER_EXPORT void solver::init() noexcept
    {
        read(INIT_STRING);
        new_types({new state_variable(*this),
                   new reusable_resource(*this),
                   new agent(*this),
                   new propositional_state(*this)});

        FIRE_STATE_CHANGED();
    }

    SOLVER_EXPORT void solver::solve()
    {
        FIRE_STARTED_SOLVING();

        // we set the gamma variable..
        heur.check();

        // we search for a consistent solution without flaws..
#ifdef CHECK_INCONSISTENCIES
        // we solve all the current inconsistencies..
        solve_inconsistencies();

        while (!flaws.empty())
        {
            assert(std::all_of(flaws.cbegin(), flaws.cend(), [this](const auto &f)
                               { return sat.value(f->phi) == True; })); // all the current flaws must be active..
            assert(std::all_of(flaws.cbegin(), flaws.cend(), [this](const auto &f)
                               { return std::none_of(f->resolvers.cbegin(), f->resolvers.cend(), [this](resolver *r)
                                                     { return sat.value(r->rho) == True; }); })); // none of the current flaws must have already been solved..

            // this is the next flaw (i.e. the most expensive one) to be solved..
            auto best_flaw = std::min_element(flaws.cbegin(), flaws.cend(), [](const auto &f0, const auto &f1)
                                              { return f0->get_estimated_cost() > f1->get_estimated_cost(); });
            assert(best_flaw != flaws.cend());
            FIRE_CURRENT_FLAW(**best_flaw);

            if (is_infinite((*best_flaw)->get_estimated_cost()))
            { // we don't know how to solve this flaw :(
                do
                { // we search..
                    next();
                } while (std::any_of(flaws.cbegin(), flaws.cend(), [this](const auto &f)
                                     { return is_infinite(f->get_estimated_cost()); }));
                // we solve all the current inconsistencies..
                solve_inconsistencies();
                continue;
            }

            // this is the next resolver (i.e. the cheapest one) to be applied..
            auto *best_res = (*best_flaw)->get_best_resolver();
            FIRE_CURRENT_RESOLVER(*best_res);

            assert(!is_infinite(best_res->get_estimated_cost()));

            // we apply the resolver..
            take_decision(best_res->get_rho());

            // we solve all the current inconsistencies..
            solve_inconsistencies();
        }
#else
        do
        {
            while (!flaws.empty())
            {
                assert(std::all_of(flaws.cbegin(), flaws.cend(), [this](const auto &f)
                                   { return sat.value(f->phi) == True; })); // all the current flaws must be active..
                assert(std::all_of(flaws.cbegin(), flaws.cend(), [this](const auto &f)
                                   { return std::none_of(f->resolvers.cbegin(), f->resolvers.cend(), [this](resolver *r)
                                                         { return sat.value(r->rho) == True; }); })); // none of the current flaws must have already been solved..

                // this is the next flaw (i.e. the most expensive one) to be solved..
                auto best_flaw = std::min_element(flaws.cbegin(), flaws.cend(), [](const auto &f0, const auto &f1)
                                                  { return f0->get_estimated_cost() > f1->get_estimated_cost(); });
                assert(best_flaw != flaws.cend());
                FIRE_CURRENT_FLAW(**best_flaw);

                if (is_infinite((*best_flaw)->get_estimated_cost()))
                { // we don't know how to solve this flaw :(
                    do
                    { // we search..
                        next();
                    } while (std::any_of(flaws.cbegin(), flaws.cend(), [this](const auto &f)
                                         { return is_infinite(f->get_estimated_cost()); }));
                    continue;
                }

                // this is the next resolver (i.e. the cheapest one) to be applied..
                auto *best_res = (*best_flaw)->get_best_resolver();
                FIRE_CURRENT_RESOLVER(*best_res);

                assert(!is_infinite(best_res->get_estimated_cost()));

                // we apply the resolver..
                take_decision(best_res->get_rho());
            }

            // we solve all the current inconsistencies..
            solve_inconsistencies();
        } while (!flaws.empty());
#endif
        // Hurray!! we have found a solution..
        LOG(std::to_string(trail.size()) << " (" << std::to_string(flaws.size()) << ")");
        FIRE_STATE_CHANGED();
        FIRE_SOLUTION_FOUND();
    }

    void solver::take_decision(const lit &ch)
    {
        assert(get_sat_core().value(ch) == Undefined);
        current_decision = ch;

        // we take the decision..
        if (!get_sat_core().assume(ch))
            throw unsolvable_exception();
        assert(std::all_of(phis.cbegin(), phis.cend(), [this](const auto &v_fs)
                           { return std::all_of(v_fs.second.cbegin(), v_fs.second.cend(), [this](const auto f)
                                                { return (sat.value(f->phi) != False && f->get_estimated_cost() == (f->get_best_resolver() ? f->get_best_resolver()->get_estimated_cost() : rational::POSITIVE_INFINITY)) || is_positive_infinite(f->get_estimated_cost()); }); }));
        assert(std::all_of(rhos.cbegin(), rhos.cend(), [this](const auto &v_rs)
                           { return std::all_of(v_rs.second.cbegin(), v_rs.second.cend(), [this](const auto r)
                                                { return is_positive_infinite(r->get_estimated_cost()) || sat.value(r->rho) != False; }); }));

        FIRE_STATE_CHANGED();

        // we check if we need to expand the graph..
        if (root_level())
            heur.check();
    }

    bool_expr solver::new_bool() noexcept
    {
        // we create a new boolean expression..
        bool_expr xp = new bool_item(*this, lit(get_sat_core().new_var()));
        // we create a new boolean flaw..
        new_flaw(*new bool_flaw(*this, get_cause(), *xp));
        return xp;
    }

    expr solver::new_enum(const type &tp, const std::vector<item *> &allowed_vals)
    {
        assert(allowed_vals.size() > 1);
        assert(tp.get_name().compare(BOOL_KEYWORD) != 0);
        assert(tp.get_name().compare(INT_KEYWORD) != 0);
        assert(tp.get_name().compare(REAL_KEYWORD) != 0);
        assert(tp.get_name().compare(TP_KEYWORD) != 0);
        // we create a new enum expression..
        // notice that we do not enforce the exct_one constraint!
        var_expr xp = new var_item(*this, tp, get_ov_theory().new_var(std::vector<var_value *>(allowed_vals.cbegin(), allowed_vals.cend()), false));
        if (allowed_vals.size() > 1) // we create a new var flaw..
            new_flaw(*new var_flaw(*this, get_cause(), *xp));
        return xp;
    }

    bool_expr solver::disj(const std::vector<bool_expr> &xprs) noexcept
    {
        // we create a new bool expression..
        std::vector<lit> lits;
        for (const auto &bex : xprs)
            lits.push_back(bex->l);
        bool_expr xp = new bool_item(*this, get_sat_core().new_disj(lits));

        if (xprs.size() > 1) // we create a new var flaw..
            new_flaw(*new disj_flaw(*this, get_cause(), lits));

        return xp;
    }

    void solver::new_atom(atom &atm, const bool &is_fact)
    {
        // we create a new atom flaw..
        atom_flaw *af = new atom_flaw(*this, get_cause(), atm, is_fact);
        new_flaw(*af);

        // we associate the flaw to the atom..
        reason.emplace(&atm, af);

        // we check if we need to notify the new atom to any smart types..
        if (&atm.get_type().get_scope() != this)
        {
            std::queue<type *> q;
            q.push(static_cast<type *>(&atm.get_type().get_scope()));
            while (!q.empty())
            {
                if (smart_type *st = dynamic_cast<smart_type *>(q.front()))
                    st->new_atom(*af);
                for (const auto &st : q.front()->get_supertypes())
                    q.push(st);
                q.pop();
            }
        }
    }

    void solver::new_disjunction(context &d_ctx, const std::vector<const conjunction *> &conjs)
    {
        // we create a new disjunction flaw..
        disjunction_flaw *df = new disjunction_flaw(*this, get_cause(), d_ctx, conjs);
        new_flaw(*df);
    }

    void solver::new_flaw(flaw &f, const bool &enqueue)
    {
        // we initialize the flaw..
        f.init(); // flaws' initialization requires being at root-level..
        FIRE_NEW_FLAW(f);

        if (enqueue) // we enqueue the flaw..
            heur.enqueue(f);
        else // we directly expand the flaw..
            expand_flaw(f);

        switch (sat.value(f.get_phi()))
        {
        case True: // we have a top-level (a landmark) flaw..
            if (enqueue || std::none_of(f.resolvers.cbegin(), f.resolvers.cend(), [this](const auto &r)
                                        { return sat.value(r->rho) == True; }))
                flaws.insert(&f); // the flaw has not yet already been solved (e.g. it has a single resolver)..
            break;
        case Undefined: // we listen for the flaw to become active..
            phis[variable(f.get_phi())].push_back(&f);
            bind(variable(f.get_phi()));
            break;
        }
    }

    void solver::new_resolver(resolver &r)
    {
        FIRE_NEW_RESOLVER(r);
        if (sat.value(r.rho) == Undefined) // we do not have a top-level (a landmark) resolver, nor an infeasible one..
        {
            // we listen for the resolver to become inactive..
            rhos[variable(r.rho)].push_back(&r);
            bind(variable(r.rho));
        }
    }

    void solver::new_causal_link(flaw &f, resolver &r)
    {
        FIRE_CAUSAL_LINK_ADDED(f, r);
        r.preconditions.push_back(&f);
        f.supports.push_back(&r);
        // activating the resolver requires the activation of the flaw..
        bool new_clause = sat.new_clause({!r.rho, f.get_phi()});
        assert(new_clause);
        // we introduce an ordering constraint..
        bool new_dist = sat.new_clause({!r.rho, get_idl_theory().new_distance(r.effect.position, f.position, 0)});
        assert(new_dist);
    }

    void solver::expand_flaw(flaw &f)
    {
        assert(!f.expanded);

        // we expand the flaw..
        f.expand();

        // we apply the flaw's resolvers..
        for (const auto &r : f.resolvers)
            apply_resolver(*r);

        if (!get_sat_core().propagate())
            throw unsolvable_exception();

        // we propagate the costs starting from the currently expanded flaw..
        heur.propagate_costs(f);

        // we clean up already solved flaws..
        if (sat.value(f.get_phi()) == True && std::any_of(f.resolvers.cbegin(), f.resolvers.cend(), [this](const auto &r)
                                                          { return sat.value(r->rho) == True; }))
            flaws.erase(&f); // this flaw has already been solved..
    }

    void solver::apply_resolver(resolver &r)
    {
        res = &r;
        set_ni(r.rho);
        try
        {
            r.apply();
        }
        catch (const inconsistency_exception &)
        { // the resolver is inapplicable..
            if (!get_sat_core().new_clause({!r.rho}))
                throw unsolvable_exception();
        }

        restore_ni();
        res = nullptr;
    }

    void solver::set_cost(flaw &f, const rational &cost)
    {
        assert(f.est_cost != cost);
        if (!trail.empty()) // we store the current flaw's estimated cost, if not already stored, for allowing backtracking..
            trail.back().old_f_costs.try_emplace(&f, f.est_cost);

        // we update the flaw's estimated cost..
        f.est_cost = cost;
        FIRE_FLAW_COST_CHANGED(f);
    }

    void solver::next()
    {
        LOG("next..");

        if (root_level())
            throw unsolvable_exception();

        std::vector<lit> no_good;
        no_good.reserve(trail.size());
        for (const auto &l : trail)
            no_good.push_back(!l.decision);
        get_sat_core().pop();

        assert(!no_good.empty());
        assert(get_sat_core().value(no_good.back()) == Undefined);

        // we reverse the no-good and store it..
        std::reverse(no_good.begin(), no_good.end());
        record(no_good);

        if (!get_sat_core().propagate())
            throw unsolvable_exception();
        assert(std::all_of(phis.cbegin(), phis.cend(), [this](const auto &v_fs)
                           { return std::all_of(v_fs.second.cbegin(), v_fs.second.cend(), [this](const auto *f)
                                                { return (sat.value(f->phi) != False && f->get_estimated_cost() == (f->get_best_resolver() ? f->get_best_resolver()->get_estimated_cost() : rational::POSITIVE_INFINITY)) || is_positive_infinite(f->get_estimated_cost()); }); }));
        assert(std::all_of(rhos.cbegin(), rhos.cend(), [this](const auto &v_rs)
                           { return std::all_of(v_rs.second.cbegin(), v_rs.second.cend(), [this](const auto *r)
                                                { return is_positive_infinite(r->get_estimated_cost()) || sat.value(r->rho) != False; }); }));

        FIRE_STATE_CHANGED();

        // we check if we need to expand the graph..
        if (root_level()) // since we are at root-level, we can reason about flaws..
            heur.check();
    }

    bool solver::propagate(const lit &p)
    {
        assert(cnfl.empty());
        assert(phis.count(variable(p)) || rhos.count(variable(p)));

        if (const auto at_phis_p = phis.find(variable(p)); at_phis_p != phis.cend())
            switch (sat.value(at_phis_p->first))
            {
            case True:
                for (const auto &f : at_phis_p->second)
                { // 'f' is an activated flaw..
                    //  FIRE_FLAW_STATE_CHANGED(*f);
                    assert(!flaws.count(f));
                    if (!root_level())
                        trail.back().new_flaws.insert(f);
                    if (std::none_of(f->resolvers.cbegin(), f->resolvers.cend(), [this](const auto &r)
                                     { return sat.value(r->rho) == True; }))
                        flaws.insert(f); // this flaw has been activated and not yet accidentally solved..
                    else if (!root_level())
                        trail.back().solved_flaws.insert(f); // this flaw has been accidentally solved..
                }
                if (root_level()) // since we are at root-level, we can perform some cleaning..
                    phis.erase(at_phis_p);
                break;
            case False:
                for (const auto &f : at_phis_p->second)
                { // 'f' will never appear in any incoming partial solutions..
                    //  FIRE_FLAW_STATE_CHANGED(*f);
                    assert(!flaws.count(f));
                    heur.propagate_costs(*f);
                }
                if (root_level()) // since we are at root-level, we can perform some cleaning..
                    phis.erase(at_phis_p);
                break;
            }

        if (const auto at_rhos_p = rhos.find(variable(p)); at_rhos_p != rhos.cend())
            switch (sat.value(at_rhos_p->first))
            {
            case True:
                for (const auto &r : at_rhos_p->second)
                {                                                 // 'r' is an activated resolver..
                                                                  //  FIRE_RESOLVER_STATE_CHANGED(*r);
                    if (flaws.erase(&r->effect) && !root_level()) // this resolver has been activated, hence its effect flaw has been resolved (notice that we remove its effect only in case it was already active)..
                        trail.back().solved_flaws.insert(&r->effect);
                }
                if (root_level()) // since we are at root-level, we can perform some cleaning..
                    rhos.erase(at_rhos_p);
                break;
            case False:
                for (const auto &r : at_rhos_p->second)
                {                                          // 'r' is a forbidden resolver..
                                                           //  FIRE_RESOLVER_STATE_CHANGED(*r);
                    if (sat.value(r->effect.phi) != False) // we update the cost of the resolver's effect..
                        heur.propagate_costs(r->effect);
                }
                if (root_level()) // since we are at root-level, we can perform some cleaning..
                    rhos.erase(at_rhos_p);
                break;
            }

        return true;
    }

    bool solver::check()
    {
        assert(cnfl.empty());
        assert(std::all_of(flaws.cbegin(), flaws.cend(), [this](const auto &f)
                           { return sat.value(f->phi) == True; }));
        assert(std::all_of(phis.cbegin(), phis.cend(), [this](const auto &v_fs)
                           { return std::all_of(v_fs.second.cbegin(), v_fs.second.cend(), [this](const auto &f)
                                                { return sat.value(f->phi) != True || (flaws.count(f) || std::any_of(trail.cbegin(), trail.cend(), [this, f](const auto &l)
                                                                                                                     { return l.solved_flaws.count(f); })); }); }));
        assert(std::all_of(phis.cbegin(), phis.cend(), [this](const auto &v_fs)
                           { return std::all_of(v_fs.second.cbegin(), v_fs.second.cend(), [this](const auto &f)
                                                { return sat.value(f->phi) != False || is_positive_infinite(f->get_estimated_cost()); }); }));
        assert(std::all_of(rhos.cbegin(), rhos.cend(), [this](const auto &v_rs)
                           { return std::all_of(v_rs.second.cbegin(), v_rs.second.cend(), [this](const auto &r)
                                                { return sat.value(r->rho) != False || is_positive_infinite(r->get_estimated_cost()); }); }));
        return true;
    }

    void solver::push()
    {
        LOG(std::to_string(trail.size()) << " (" << std::to_string(flaws.size()) << ")"
                                         << " +[" << to_string(current_decision) << "]");

        // we push the given decision into the trail..
        trail.push_back(layer(current_decision));
    }

    void solver::pop()
    {
        LOG(std::to_string(trail.size()) << " (" << std::to_string(flaws.size()) << ")"
                                         << " -[" << to_string(trail.back().decision) << "]");

        // we reintroduce the solved flaw..
        for (const auto &f : trail.back().solved_flaws)
            flaws.insert(f);

        // we erase the new flaws..
        for (const auto &f : trail.back().new_flaws)
            flaws.erase(f);

        // we restore the flaws' estimated costs..
        for (const auto &[f, cost] : trail.back().old_f_costs)
        {
            // assert(f.first->est_cost != cost);
            f->est_cost = cost;
            FIRE_FLAW_COST_CHANGED(*f);
        }

        trail.pop_back();

        LOG(std::to_string(trail.size()) << " (" << std::to_string(flaws.size()) << ")");
    }

    void solver::solve_inconsistencies()
    {
        // all the current inconsistencies..
        auto incs = get_incs();

        while (!incs.empty())
            if (const auto &uns_flw = std::find_if(incs.cbegin(), incs.cend(), [](const auto &v)
                                                   { return v.empty(); });
                uns_flw != incs.cend())
            { // we have an unsolvable flaw..
                // we backtrack..
                next();
                // we re-collect all the inconsistencies from all the smart-types..
                incs = get_incs();
            }
            else if (const auto &det_flw = std::find_if(incs.cbegin(), incs.cend(), [](const auto &v)
                                                        { return v.size() == 1; });
                     det_flw != incs.cend())
            { // we have deterministic flaw..
                assert(get_sat_core().value(det_flw->at(0).first) != False);
                if (get_sat_core().value(det_flw->at(0).first) == Undefined)
                {
                    // we learn something from it..
                    std::vector<lit> learnt;
                    learnt.reserve(trail.size() + 1);
                    learnt.push_back(det_flw->at(0).first);
                    for (const auto &l : trail)
                        learnt.push_back(!l.decision);
                    record(learnt);
                    if (!get_sat_core().propagate())
                        throw unsolvable_exception();
                }

                // we re-collect all the inconsistencies from all the smart-types..
                incs = get_incs();
            }
            else
            { // we have to take a decision..
                std::vector<std::pair<lit, double>> bst_inc;
                double k_inv = std::numeric_limits<double>::infinity();
                for (const auto &inc : incs)
                {
                    double bst_commit = std::numeric_limits<double>::infinity();
                    for (const auto &[choice, commit] : inc)
                        if (commit < bst_commit)
                            bst_commit = commit;
                    double c_k_inv = 0;
                    for (const auto &[choice, commit] : inc)
                        c_k_inv += 1l / (1l + (commit - bst_commit));
                    if (c_k_inv < k_inv)
                    {
                        k_inv = c_k_inv;
                        bst_inc = inc;
                    }
                }

                // we select the best choice (i.e. the least committing one) from those available for the best flaw..
                take_decision(std::min_element(bst_inc.cbegin(), bst_inc.cend(), [](const auto &ch0, const auto &ch1)
                                               { return ch0.second < ch1.second; })
                                  ->first);

                // we re-collect all the inconsistencies from all the smart-types..
                incs = get_incs();
            }
    }

    std::vector<std::vector<std::pair<lit, double>>> solver::get_incs()
    {
        std::vector<std::vector<std::pair<lit, double>>> incs;
        // we collect all the inconsistencies from all the smart-types..
        for (const auto &st : sts)
        {
            const auto c_incs = st->get_current_incs();
            incs.insert(incs.cend(), c_incs.cbegin(), c_incs.cend());
        }
        assert(std::all_of(incs.cbegin(), incs.cend(), [](const auto &inc)
                           { return std::all_of(inc.cbegin(), inc.cend(), [](const auto &ch)
                                                { return std::isfinite(ch.second); }); }));
        return incs;
    }

    void solver::reset_smart_types()
    {
        // some cleanings..
        sts.clear();
        std::queue<type *> q;
        for (const auto &[tp_name, tp] : get_types())
            if (!tp->is_primitive())
                q.push(tp);
        while (!q.empty())
        {
            if (smart_type *st = dynamic_cast<smart_type *>(q.front()))
                sts.push_back(st);
            for (const auto &st : q.front()->get_types())
                q.push(st.second);
            q.pop();
        }
    }

#ifdef BUILD_LISTENERS
    void solver::fire_new_flaw(const flaw &f) const
    {
        for (const auto &l : listeners)
            l->new_flaw(f);
    }
    void solver::fire_flaw_state_changed(const flaw &f) const
    {
        for (const auto &l : listeners)
            l->flaw_state_changed(f);
    }
    void solver::fire_flaw_cost_changed(const flaw &f) const
    {
        for (const auto &l : listeners)
            l->flaw_cost_changed(f);
    }
    void solver::fire_current_flaw(const flaw &f) const
    {
        for (const auto &l : listeners)
            l->current_flaw(f);
    }
    void solver::fire_new_resolver(const resolver &r) const
    {
        for (const auto &l : listeners)
            l->new_resolver(r);
    }
    void solver::fire_resolver_state_changed(const resolver &r) const
    {
        for (const auto &l : listeners)
            l->resolver_state_changed(r);
    }
    void solver::fire_current_resolver(const resolver &r) const
    {
        for (const auto &l : listeners)
            l->current_resolver(r);
    }
    void solver::fire_causal_link_added(const flaw &f, const resolver &r) const
    {
        for (const auto &l : listeners)
            l->causal_link_added(f, r);
    }
#endif
} // namespace ratio