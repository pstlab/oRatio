#include "solver.h"
#include "graph.h"
#include "var_flaw.h"
#include "atom_flaw.h"
#include "disjunction_flaw.h"
#include "smart_type.h"
#include "atom.h"
#ifdef BUILD_GUI
#include "solver_listener.h"
#endif
#include <algorithm>
#include <cassert>

using namespace smt;

namespace ratio
{

solver::solver() : core(), theory(get_sat_core()) {}
solver::~solver() {}

void solver::init()
{
    read(std::vector<std::string>({"init.rddl"}));
}

void solver::solve() {}

expr solver::new_enum(const type &tp, const std::unordered_set<item *> &allowed_vals)
{
    // we create a new enum expression..
    var_expr xp = core::new_enum(tp, allowed_vals);
    if (allowed_vals.size() > 1)
    {
        // we create a new var flaw..
        var_flaw *ef = new var_flaw(*this, res, *xp);
        new_flaw(*ef);
    }
    return xp;
}

void solver::new_fact(atom &atm)
{
    // we create a new atom flaw representing a fact..
    atom_flaw *af = new atom_flaw(*this, res, atm, true);
    new_flaw(*af);

    if (&atm.get_type().get_scope() != this)
    { // we check if we need to notify the new fact to any smart types..
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
    atom_flaw *af = new atom_flaw(*this, res, atm, false);
    new_flaw(*af);

    if (&atm.get_type().get_scope() != this)
    { // we check if we need to notify the new goal to any smart types..
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
    disjunction_flaw *df = new disjunction_flaw(*this, res, d_ctx, disj);
    new_flaw(*df);
}

bool solver::propagate(const lit &p, std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    const auto at_phis_p = phis.find(p.get_var());
    if (at_phis_p != phis.end()) // a decision has been taken about the presence of some flaws within the current partial solution..
        for (const auto &f : at_phis_p->second)
            if (p.get_sign()) // this flaw has been added to the current partial solution..
            {
                flaws.insert(f);
                if (!trail.empty())
                    trail.back().new_flaws.insert(f);
#ifdef BUILD_GUI
                // we notify the listeners that the state of the flaw has changed..
                for (const auto &l : listeners)
                    l->flaw_state_changed(*f);
#endif
            }
            else // this flaw has been removed from the current partial solution..
                assert(flaws.find(f) == flaws.end());

    const auto at_rhos_p = rhos.find(p.get_var());
    if (at_rhos_p != rhos.end() && !p.get_sign()) // a decision has been taken about the removal of some resolvers within the current partial solution..
        for (const auto &r : at_rhos_p->second)
            set_estimated_cost(*r, rational::POSITIVE_INFINITY);

    return true;
}

bool solver::check(std::vector<lit> &cnfl)
{
    assert(cnfl.empty());
    return true;
}

void solver::push()
{
    // we push the given resolver into the trail..
    trail.push_back(layer(res));
    if (res)
    { // we just solved the resolver's effect..
        trail.back().solved_flaws.insert(&res->effect);
        flaws.erase(&res->effect);
    }
}

void solver::pop()
{
    // we reintroduce the solved flaw..
    for (const auto &f : trail.back().solved_flaws)
        flaws.insert(f);

    // we erase the new flaws..
    for (const auto &f : trail.back().new_flaws)
        flaws.erase(f);

    // we restore the resolvers' estimated costs..
    for (const auto &r : trail.back().old_costs)
        r.first->est_cost = r.second;

#ifdef BUILD_GUI
    // we notify the listeners that the cost of some resolvers has been restored..
    for (const auto &l : listeners)
        for (const auto &c : trail.back().old_costs)
            l->resolver_cost_changed(*c.first);
#endif

    trail.pop_back();
}

void solver::new_flaw(flaw &f)
{
    f.init(); // flaws' initialization requires being at root-level..
#ifdef BUILD_GUI
    // we notify the listeners that a new flaw has been created..
    for (const auto &l : listeners)
        l->new_flaw(f);
#endif
    switch (sat.value(f.phi))
    {
    case True: // we have a top-level (a landmark) flaw..
        flaws.insert(&f);
        break;
    case Undefined: // we listen for the flaw to become active..
        phis[f.phi].push_back(&f);
        bind(f.phi);
        break;
    }

    // we enqueue the flaw..
    flaw_q.push_back(&f);
}

void solver::new_resolver(resolver &r)
{
#ifdef BUILD_GUI
    // we notify the listeners that a new resolver has been created..
    for (const auto &l : listeners)
        l->new_resolver(r);
#endif
    if (sat.value(r.rho) == Undefined) // we do not have a top-level (a landmark) resolver..
    {
        // we listen for the resolver to become active..
        rhos[r.rho].push_back(&r);
        bind(r.rho);
    }
}

void solver::new_causal_link(flaw &f, resolver &r)
{
#ifdef BUILD_GUI
    // we notify the listeners that a new causal link has been created..
    for (const auto &l : listeners)
        l->causal_link_added(f, r);
#endif
    r.preconditions.push_back(&f);
    f.supports.push_back(&r);
    bool new_clause = sat.new_clause({lit(r.rho, false), f.phi});
    assert(new_clause);
}

void solver::set_estimated_cost(resolver &r, const rational &cst)
{
    if (r.est_cost != cst)
    {
        if (!trail.empty()) // we store the current cost for allowing backtracking..
            trail.back().old_costs.insert({&r, r.est_cost});
        // this is the current cost of the resolver's effect..
        rational f_cost = r.effect.get_estimated_cost();
        // we update the resolver's estimated cost..
        r.est_cost = cst;
#ifdef BUILD_GUI
        // we notify the listeners that a resolver cost has changed..
        for (const auto &l : listeners)
            l->resolver_cost_changed(r);
#endif

        if (f_cost != r.effect.get_estimated_cost()) // the cost of the resolver's effect has changed as a consequence of the resolver's cost update, hence, we propagate the update to all the supports of the resolver's effect..
        {
            // the resolver costs queue (for resolver cost propagation)..
            std::queue<resolver *> resolver_q;
            for (const auto &c_r : r.effect.supports)
                resolver_q.push(c_r);

            while (!resolver_q.empty())
            {
                resolver &c_res = *resolver_q.front(); // the current resolver whose cost might require an update..
                rational r_cost = evaluate(c_res.preconditions);
                if (c_res.est_cost != r_cost)
                {
                    if (!trail.empty())
                        trail.back().old_costs.insert({&c_res, c_res.est_cost});
                    // this is the current cost of the resolver's effect..
                    f_cost = c_res.effect.get_estimated_cost();
                    // we update the resolver's estimated cost..
                    c_res.est_cost = r_cost;
#ifdef BUILD_GUI
                    // we notify the listeners that a resolver cost has changed..
                    for (const auto &l : listeners)
                        l->resolver_cost_changed(c_res);
#endif

                    if (f_cost != c_res.effect.get_estimated_cost())  // the cost of the resolver's effect has changed as a consequence of the resolver's cost update..
                        for (const auto &c_r : c_res.effect.supports) // hence, we propagate the update to all the supports of the resolver's effect..
                            resolver_q.push(c_r);
                }
                resolver_q.pop();
            }
        }
    }
}

const smt::rational solver::evaluate(const std::vector<flaw *> &fs)
{
    rational c_cost;
#ifdef H_MAX
    c_cost = rational::NEGATIVE_INFINITY;
    for (const auto &f : fs)
    {
        rational c = f->get_estimated_cost();
        if (c > c_cost)
            c_cost = c;
    }
#endif
#ifdef H_ADD
    for (const auto &f : fs)
        c_cost += f->get_estimated_cost();
#endif
    return c_cost;
}

flaw *solver::select_flaw()
{
    assert(std::all_of(flaws.begin(), flaws.end(), [&](flaw *const f) { return sat.value(f->phi) == True; }));
    // this is the next flaw to be solved (i.e., the most expensive one)..
    flaw *f_next = nullptr;
    for (auto it = flaws.begin(); it != flaws.end();)
        if (std::any_of((*it)->resolvers.begin(), (*it)->resolvers.end(), [&](resolver *r) { return sat.value(r->rho) == True; }))
        {
            // we remove the flaw from the current flaws..
            if (!trail.empty())
                trail.back().solved_flaws.insert((*it));
            flaws.erase(it++);
        }
        else
        {
            // the current flaw is not trivial nor already solved: let's see if it's better than the previous one..
            if (!f_next) // this is the first flaw we see..
                f_next = *it;
            else if (f_next->structural && !(*it)->structural) // we prefere non-structural flaws (i.e., inconsistencies) to structural ones..
                f_next = *it;
            else if (f_next->structural == (*it)->structural && f_next->get_estimated_cost() < (*it)->get_estimated_cost()) // this flaw is actually better than the previous one..
                f_next = *it;
            ++it;
        }

    return f_next;
}
} // namespace ratio
