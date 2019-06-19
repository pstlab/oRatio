#include "solver.h"
#include "graph.h"
#include "var_flaw.h"
#include "atom_flaw.h"
#include "disjunction_flaw.h"
#include "composite_flaw.h"
#include "smart_type.h"
#include "atom.h"
#include "state_variable.h"
#include "reusable_resource.h"
#include "propositional_state.h"
#include "propositional_agent.h"
#ifdef BUILD_GUI
#include "solver_listener.h"
#endif
#include <algorithm>
#include <stdexcept>
#include <math.h>
#include <cassert>

using namespace smt;

namespace ratio
{

solver::solver() : core(), theory(get_sat_core()), gr(*this) {}
solver::~solver() {}

void solver::init()
{
    read(std::vector<std::string>({"init.rddl"}));
    new_types({new state_variable(*this),
               new reusable_resource(*this),
               new propositional_state(*this),
               new propositional_agent(*this)});
}

void solver::solve()
{
    // some cleanings..
    sts.clear();
    std::queue<type *> q;
    for (const auto &t : get_types())
        if (!t.second->is_primitive())
            q.push(t.second);
    while (!q.empty())
    {
        if (smart_type *st = dynamic_cast<smart_type *>(q.front()))
            sts.push_back(st);
        for (const auto &st : q.front()->get_types())
            q.push(st.second);
        q.pop();
    }

    // we build the causal graph..
    gr.build();

#ifdef BUILD_GUI
    fire_state_changed();
#endif

    // we solve all the current inconsistencies..
    solve_inconsistencies();

    // we enter into the main solving loop..
    while (!flaws.empty())
    {
        assert(std::all_of(flaws.begin(), flaws.end(), [this](flaw *const f) { return sat.value(f->phi) == True; }));                                                                                         // all the current flaws must be active..
        assert(std::all_of(flaws.begin(), flaws.end(), [this](flaw *const f) { return std::none_of(f->resolvers.begin(), f->resolvers.end(), [this](resolver *r) { return sat.value(r->rho) == True; }); })); // none of the current flaws must have already been solved..

        // this is the next flaw (i.e. the most expensive one) to be solved..
        auto f_next = std::min_element(flaws.begin(), flaws.end(), [](flaw *const f0, flaw *const f1) { return f0->get_estimated_cost() > f1->get_estimated_cost(); });
        assert(f_next != flaws.end());

#ifdef BUILD_GUI
        fire_current_flaw(**f_next);
#endif
        if ((*f_next)->get_estimated_cost().is_infinite()) // we don't know how to solve this flaw: we search..
        {
            next();
            continue;
        }

        // this is the next resolver (i.e. the cheapest one) to be applied..
        auto *res = (*f_next)->get_best_resolver();
#ifdef BUILD_GUI
        fire_current_resolver(*res);
#endif
        assert(!res->get_estimated_cost().is_infinite());

        // we apply the resolver..
        take_decision(res->get_rho());

        // we solve all the current inconsistencies..
        solve_inconsistencies();
    }
    // Hurray!! we have found a solution..
#ifdef BUILD_GUI
    fire_state_changed();
#endif
}

expr solver::new_enum(const type &tp, const std::unordered_set<item *> &allowed_vals)
{
    assert(allowed_vals.size() > 1);
    assert(tp.get_name().compare(BOOL_KEYWORD) != 0);
    assert(tp.get_name().compare(INT_KEYWORD) != 0);
    assert(tp.get_name().compare(REAL_KEYWORD) != 0);
    // we create a new enum expression..
    // notice that we do not enforce the exct_one constraint!
    var_expr xp = new var_item(*this, tp, get_ov_theory().new_var(std::unordered_set<var_value *>(allowed_vals.begin(), allowed_vals.end()), false));
    if (allowed_vals.size() > 1) // we create a new var flaw..
        gr.new_flaw(*new var_flaw(gr, gr.res, *xp));
    return xp;
}

void solver::new_fact(atom &atm)
{
    // we create a new atom flaw representing a fact..
    atom_flaw *af = new atom_flaw(gr, gr.res, atm, true);
    gr.new_flaw(*af);

    // we associate the flaw to the atom..
    reason.emplace(&atm, af);

    // we check if we need to notify the new fact to any smart types..
    if (&atm.get_type().get_scope() != this)
    {
        std::queue<type *> q;
        q.push(static_cast<type *>(&atm.get_type().get_scope()));
        while (!q.empty())
        {
            if (smart_type *st = dynamic_cast<smart_type *>(q.front()))
                st->new_fact(*af);
            for (const auto &st : q.front()->get_supertypes())
                q.push(st);
            q.pop();
        }
    }
}

void solver::new_goal(atom &atm)
{
    // we create a new atom flaw representing a goal..
    atom_flaw *af = new atom_flaw(gr, gr.res, atm, false);
    gr.new_flaw(*af);

    // we associate the flaw to the atom..
    reason.emplace(&atm, af);

    // we check if we need to notify the new goal to any smart types..
    if (&atm.get_type().get_scope() != this)
    {
        std::queue<type *> q;
        q.push(static_cast<type *>(&atm.get_type().get_scope()));
        while (!q.empty())
        {
            if (smart_type *st = dynamic_cast<smart_type *>(q.front()))
                st->new_goal(*af);
            for (const auto &st : q.front()->get_supertypes())
                q.push(st);
            q.pop();
        }
    }
}

void solver::new_disjunction(context &d_ctx, const disjunction &disj)
{
    // we create a new disjunction flaw..
    disjunction_flaw *df = new disjunction_flaw(gr, gr.res, d_ctx, disj);
    gr.new_flaw(*df);
}

void solver::take_decision(const lit &ch)
{
    LOG("taking decision " << (ch.get_sign() ? std::to_string(ch.get_var()) : "!" + std::to_string(ch.get_var())));

    // we push the given resolver into the trail..
    trail.push_back(layer(ch));
    LOG("level " << std::to_string(trail.size()));

    // we take the decision..
    if (!get_sat_core().assume(ch) || !get_sat_core().check())
        throw std::runtime_error("the problem is unsolvable");

#ifdef BUILD_GUI
    fire_state_changed();
#endif
}

void solver::next()
{
    LOG("next..");

    if (root_level())
        throw std::runtime_error("the problem is unsolvable");

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

    if (!get_sat_core().check())
        throw std::runtime_error("the problem is unsolvable");

#ifdef BUILD_GUI
    fire_state_changed();
#endif

    // we check if we need to expand the graph..
    if (root_level())
        gr.check_gamma();
}

bool solver::propagate(const lit &p, std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    assert(gr.phis.count(p.get_var()) || gr.rhos.count(p.get_var()));

    if (const auto at_rhos_p = gr.rhos.find(p.get_var()); at_rhos_p != gr.rhos.end()) // a decision has been taken about the removal of some resolvers within the current partial solution..
        for (const auto &r : at_rhos_p->second)
            if (p.get_sign()) // this resolver has been applied hence its effect has been resolved..
            {
                if (flaws.erase(&r->effect))
                    if (!root_level())
                        trail.back().solved_flaws.insert(&r->effect);
            }
            else // since this resolver cannot be applied its cost is set to +inf..
                gr.set_estimated_cost(*r, rational::POSITIVE_INFINITY);

    if (const auto at_phis_p = gr.phis.find(p.get_var()); at_phis_p != gr.phis.end()) // a decision has been taken about the presence of some flaws within the current partial solution..
        for (const auto &f : at_phis_p->second)
            if (p.get_sign()) // this flaw has been added to the current partial solution..
            {
                if (std::none_of(f->resolvers.begin(), f->resolvers.end(), [this](resolver *r) { return sat.value(r->rho) == True; }))
                { // it has not yet already have been accidentally solved..
                    flaws.insert(f);
                    if (!root_level())
                        trail.back().new_flaws.insert(f);
                }
            }
            else // this flaw has been removed from the current partial solution..
            {
                assert(!flaws.count(f));
                if (f->est_cost != rational::POSITIVE_INFINITY)
                {
                    if (!root_level()) // we store the current flaw's estimated cost, if not already stored, for allowing backtracking (just for having consistency with resolvers)..
                        trail.back().old_f_costs.try_emplace(f, f->est_cost);
                    f->est_cost = rational::POSITIVE_INFINITY;
#ifdef BUILD_GUI
                    fire_flaw_cost_changed(*f);
#endif
                }
            }

    return true;
}

bool solver::check(std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    assert(std::all_of(gr.phis.begin(), gr.phis.end(), [this](std::pair<smt::var, std::vector<flaw *>> v_fs) { return std::all_of(v_fs.second.begin(), v_fs.second.end(), [this](flaw *f) { return sat.value(f->phi) != False || f->get_estimated_cost().is_positive_infinite(); }); }));
    assert(std::all_of(gr.rhos.begin(), gr.rhos.end(), [this](std::pair<smt::var, std::vector<resolver *>> v_rs) { return std::all_of(v_rs.second.begin(), v_rs.second.end(), [this](resolver *r) { return sat.value(r->rho) != False || r->get_estimated_cost().is_positive_infinite(); }); }));
    return true;
}

void solver::push() {}

void solver::pop()
{
    // we reintroduce the solved flaw..
    for (const auto &f : trail.back().solved_flaws)
        flaws.insert(f);

    // we erase the new flaws..
    for (const auto &f : trail.back().new_flaws)
        flaws.erase(f);

    // we restore the resolvers' estimated costs..
    for (const auto &r : trail.back().old_r_costs)
    {
        assert(r.first->est_cost != r.second);
        r.first->est_cost = r.second;
#ifdef BUILD_GUI
        fire_resolver_cost_changed(*r.first);
#endif
    }

    // we restore the flaws' estimated costs..
    for (const auto &f : trail.back().old_f_costs)
    {
        assert(f.first->est_cost != f.second);
        f.first->est_cost = f.second;
#ifdef BUILD_GUI
        fire_flaw_cost_changed(*f.first);
#endif
    }

    trail.pop_back();

    LOG("level " << std::to_string(trail.size()));
}

void solver::solve_inconsistencies()
{
    // all the current inconsistencies..
    std::vector<std::vector<std::pair<lit, double>>> incs = get_incs();

    while (!incs.empty())
        if (const auto uns_flw = std::find_if(incs.begin(), incs.end(), [](const std::vector<std::pair<lit, double>> &v) { return v.empty(); }); uns_flw != incs.end())
        { // we have an unsolvable flaw..
            // we backtrack..
            next();
            // we re-collect all the inconsistencies from all the smart-types..
            incs = get_incs();
        }
        else if (const auto det_flw = std::find_if(incs.begin(), incs.end(), [](const std::vector<std::pair<lit, double>> &v) { return v.size() == 1; }); det_flw != incs.end())
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
                if (!get_sat_core().check())
                    throw std::runtime_error("the problem is unsolvable");
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
                for (const auto &ch : inc)
                    if (ch.second < bst_commit)
                        bst_commit = ch.second;
                double c_k_inv = 0;
                for (const auto &ch : inc)
                    c_k_inv += 1l / (1l + (ch.second - bst_commit));
                if (c_k_inv < k_inv)
                {
                    k_inv = c_k_inv;
                    bst_inc = inc;
                }
            }

            // we select the best choice (i.e. the least committing one) from those available for the best flaw..
            take_decision(std::min_element(bst_inc.begin(), bst_inc.end(), [](std::pair<lit, double> const &ch0, std::pair<lit, double> const &ch1) { return ch0.second < ch1.second; })->first);

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
        incs.insert(incs.end(), c_incs.begin(), c_incs.end());
    }
    assert(std::all_of(incs.begin(), incs.end(), [](std::vector<std::pair<lit, double>> inc) { return std::all_of(inc.begin(), inc.end(), [](std::pair<lit, double> ch) { return std::isfinite(ch.second); }); }));

    if (root_level()) // since we are at root-level, we can reason about flaws..
    {
        for (const auto &st : sts)
            for (const auto &f : st->get_flaws())
                gr.new_flaw(*f, false); // we add the flaws to the planning graph..

        gr.check_gamma();
    }
    return incs;
}

#ifdef BUILD_GUI
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
void solver::fire_resolver_cost_changed(const resolver &r) const
{
    for (const auto &l : listeners)
        l->resolver_cost_changed(r);
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
