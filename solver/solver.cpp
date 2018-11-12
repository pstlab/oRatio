#include "solver.h"
#include "graph.h"
#include <cassert>

using namespace smt;

namespace ratio
{

solver::solver() : core(), theory(get_sat_core()) {}
solver::~solver() {}

void solver::solve() {}

expr solver::new_enum(const type &tp, const std::unordered_set<item *> &allowed_vals)
{
    // we create a new enum expression..
    var_expr xp = core::new_enum(tp, allowed_vals);
    // TODO: create a new enum flaw..
    return xp;
}

void solver::new_fact(atom &atm)
{
    // TODO: create a new fact flaw..
}

void solver::new_goal(atom &atm)
{
    // TODO: create a new goal flaw..
}

void solver::new_disjunction(context &d_ctx, const disjunction &disj)
{
    // TODO: create a new disjunction flaw..
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

void solver::push() {}

void solver::pop() {}

void solver::new_flaw(flaw &f)
{
    f.init(); // flaws' initialization requires being at root-level..

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
    if (sat.value(r.rho) == Undefined) // we do not have a top-level (a landmark) resolver..
    {
        // we listen for the resolver to become active..
        rhos[r.rho].push_back(&r);
        bind(r.rho);
    }
}

void solver::new_causal_link(flaw &f, resolver &r)
{
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
