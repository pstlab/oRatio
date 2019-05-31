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
#include <stdexcept>
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
    // we build the causal graph..
    gr.build();

#ifdef BUILD_GUI
    fire_state_changed();
#endif

    // we solve all the current inconsistencies..
    solve_inconsistencies();

    // we create and set a new graph var..
    gr.set_new_gamma();

    // we enter into the main solving loop..
    while (true)
    {
        // this is the next flaw to be solved..
        flaw *f_next = select_flaw();
#ifdef BUILD_GUI
        if (f_next)
            fire_current_flaw(*f_next);
#endif

        if (f_next)
        {
            if (f_next->get_estimated_cost().is_infinite())
            {
                // we don't know how to solve this flaw: we search..
                next();
                continue;
            }

            // this is the next resolver to be applied..
            resolver *res = f_next->get_best_resolver();
#ifdef BUILD_GUI
            fire_current_resolver(*res);
#endif
            assert(!res->get_estimated_cost().is_infinite());

            // we apply the resolver..
            take_decision(res->get_rho());

            // we solve all the current inconsistencies..
            solve_inconsistencies();
        }
        else
        {
            // Hurray!! we have found a solution..
#ifdef BUILD_GUI
            fire_state_changed();
#endif
            return;
        }
    }
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

void solver::take_decision(const smt::lit &ch)
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

    if (get_sat_core().root_level())
        throw std::runtime_error("the problem is unsolvable");

    std::vector<smt::lit> no_good;
    no_good.reserve(trail.size());
    for (const auto &l : trail)
        no_good.push_back(!l.decision);
    get_sat_core().pop();

    assert(!no_good.empty());
    assert(get_sat_core().value(no_good.back()) == Undefined);
    record(no_good);
    if (!get_sat_core().check())
        throw std::runtime_error("the problem is unsolvable");

#ifdef BUILD_GUI
    fire_state_changed();
#endif

    // we check if we need to change the graph..
    if (get_sat_core().value(gr.gamma) == False)
    { // we do need to change the graph..
        assert(get_sat_core().root_level());
        if (gr.accuracy < gr.max_accuracy) // we have room for increasing the heuristic accuracy..
        {
            gr.increase_accuracy(); // so we increase the heuristic accuracy..
            gr.set_new_gamma();     // we create and set a new graph var..
        }
        else
        {
            gr.add_layer();     // we add a layer to the current graph..
            gr.set_new_gamma(); // we create and set a new graph var..
        }

#ifdef BUILD_GUI
        fire_state_changed();
#endif
    }
}

bool solver::propagate(const lit &p, std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    const auto at_phis_p = gr.phis.find(p.get_var());
    const auto at_rhos_p = gr.rhos.find(p.get_var());
    assert(at_phis_p != gr.phis.end() || at_rhos_p != gr.rhos.end());

    if (at_phis_p != gr.phis.end()) // a decision has been taken about the presence of some flaws within the current partial solution..
        for (const auto &f : at_phis_p->second)
        {
#ifdef BUILD_GUI
            fire_flaw_state_changed(*f);
#endif
            if (p.get_sign()) // this flaw has been added to the current partial solution..
            {
                flaws.insert(f);
                if (!trail.empty())
                    trail.back().new_flaws.insert(f);
            }
            else // this flaw has been removed from the current partial solution..
                assert(flaws.find(f) == flaws.end());
        }

    if (at_rhos_p != gr.rhos.end()) // a decision has been taken about the removal of some resolvers within the current partial solution..
        for (const auto &r : at_rhos_p->second)
        {
#ifdef BUILD_GUI
            fire_resolver_state_changed(*r);
#endif
            if (p.get_sign()) // this resolver has been applied hence its effect has been resolved..
            {
                if (flaws.erase(&r->effect))
                    trail.back().solved_flaws.insert(&r->effect);
            }
            else // since this resolver cannot be applied its cost is set to +inf..
                gr.set_estimated_cost(*r, rational::POSITIVE_INFINITY);
        }

    return true;
}

bool solver::check(std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
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
        r.first->est_cost = r.second;
#ifdef BUILD_GUI
        fire_resolver_cost_changed(*r.first);
#endif
    }

    // we restore the flaws' estimated costs..
    for (const auto &f : trail.back().old_f_costs)
    {
        f.first->est_cost = f.second;
#ifdef BUILD_GUI
        fire_flaw_cost_changed(*f.first);
#endif
    }

    trail.pop_back();

    LOG("level " << std::to_string(trail.size()));
}

flaw *solver::select_flaw()
{
    assert(std::all_of(flaws.begin(), flaws.end(), [&](flaw *const f) { return sat.value(f->phi) == True; }));
    // this is the next flaw to be solved (i.e. the most expensive one)..
    flaw *f_next = nullptr;
    for (auto it = flaws.begin(); it != flaws.end();)
        if (std::any_of((*it)->resolvers.begin(), (*it)->resolvers.end(), [&](resolver *r) { return sat.value(r->rho) == True; }))
        {
            // this flaw has an active resolver, hence it is already solved!
            // we remove the flaw from the current set of flaws..
            if (!trail.empty())
                trail.back().solved_flaws.insert((*it));
            flaws.erase(it++);
        }
        else
        {
            // the current flaw is not trivial nor already solved: let's see if it's better than the previous one..
            if (!f_next) // this is the first flaw we see..
                f_next = *it;
            else if (f_next->get_estimated_cost() < (*it)->get_estimated_cost()) // this flaw is actually better than the previous one..
                f_next = *it;
            ++it;
        }

    return f_next;
}

void solver::solve_inconsistencies()
{
    LOG("checking for inconsistencies..");
    std::vector<smart_type *> sts;
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

    // all the current inconsistencies..
    std::vector<std::vector<std::pair<lit, double>>> incs;

    // we collect all the inconsistencies from all the smart-types..
    for (const auto &st : sts)
    {
        std::vector<std::vector<std::pair<lit, double>>> c_incs = st->get_current_incs();
        incs.insert(incs.end(), c_incs.begin(), c_incs.end());

        if (trail.empty())
            // since we are at root-level, we can reason about flaws..
            for (const auto &f : st->get_flaws())
                gr.new_flaw(*f); // we add the flaws to the planning graph..
    }

    while (!incs.empty())
    {
        std::vector<std::pair<lit, double>> bst_inc;
        double k_inv = std::numeric_limits<double>::infinity();
        for (const auto &inc : incs)
        {
            switch (inc.size())
            {
            case 0: // we have an unsolvable flaw: we backtrack..
                next();
                return;
            case 1: // we have a deterministic flaw: we learn something from it..
            {
                std::vector<smt::lit> learnt;
                learnt.reserve(trail.size() + 1);
                learnt.push_back(inc.at(0).first);
                for (const auto &l : trail)
                    learnt.push_back(!l.decision);
                record(learnt);
                if (!get_sat_core().check())
                    throw std::runtime_error("the problem is unsolvable");
                break; // we check for further flaws..
            }
            default: // we have to take a decision..
                double bst_commit = std::numeric_limits<double>::infinity();
                for (const auto &ch : inc)
                    if (ch.second < bst_commit)
                        bst_commit = ch.second;
                double c_k_inv = 0;
                for (const auto &ch : inc)
                    c_k_inv += 1l / (1l + ch.second - bst_commit);
                if (c_k_inv < k_inv)
                {
                    k_inv = c_k_inv;
                    bst_inc = inc;
                }
                break;
            }
        }

        // we select the best choice (i.e. the least committing one) from those available for the best flaw..
        take_decision(std::min_element(bst_inc.begin(), bst_inc.end(), [](std::pair<lit, double> const &ch0, std::pair<lit, double> const &ch1) { return ch0.second < ch1.second; })->first);

        // we clear the current inconsistencies..
        incs.clear();
        // we re-collect all the inconsistencies from all the smart-types..
        for (const auto &st : sts)
        {
            std::vector<std::vector<std::pair<lit, double>>> c_incs = st->get_current_incs();
            incs.insert(incs.end(), c_incs.begin(), c_incs.end());

            if (trail.empty())
                // since we are at root-level, we can reason about flaws..
                for (const auto &f : st->get_flaws())
                    gr.new_flaw(*f); // we add the flaws to the planning graph..
        }
    }
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
