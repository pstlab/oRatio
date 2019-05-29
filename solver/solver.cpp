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

    // we enter into the main solving loop..
    while (true)
    {
        // we solve all the current inconsistencies..
        solve_inconsistencies();

        // this is the next flaw to be solved..
        flaw *f_next = select_flaw();

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
            assert(!res->get_estimated_cost().is_infinite());

            // we apply the resolver..
            take_decision(res->get_rho());
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
    // TODO: add code for taking a decision..
}

void solver::next()
{
    // TODO: add code for moving to the next solution..
}

bool solver::propagate(const lit &p, std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    const auto at_phis_p = gr.phis.find(p.get_var());
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

    const auto at_rhos_p = gr.rhos.find(p.get_var());
    if (at_rhos_p != gr.rhos.end() && !p.get_sign()) // a decision has been taken about the removal of some resolvers within the current partial solution..
        for (const auto &r : at_rhos_p->second)
        {
#ifdef BUILD_GUI
            fire_resolver_state_changed(*r);
#endif
            gr.set_estimated_cost(*r, rational::POSITIVE_INFINITY);
        }

    return true;
}

bool solver::check(std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    return true;
}

void solver::push()
{
}

void solver::pop()
{
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

    if (trail.empty()) // since we are at root-level, we can reason about flaws..
        // we collect the flaws from the smart-types and add them to the planning graph..
        for (const auto &st : sts)
            for (const auto &f : st->get_flaws())
                gr.new_flaw(*f); // we add the flaws to the planning graph..
    else
    { // since we are not at root-level, we need to reason about inconsistencies..
        std::vector<std::vector<std::pair<lit, double>>> incs;
        // we collect all the inconsistencies from all the smart-types..
        for (const auto &st : sts)
        {
            std::vector<std::vector<std::pair<lit, double>>> c_incs = st->get_current_incs();
            incs.insert(incs.end(), c_incs.begin(), c_incs.end());
        }

        if (incs.empty())
            return;
        else
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
