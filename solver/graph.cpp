#include "graph.h"
#include "solver.h"
#include "combinations.h"
#include "composite_flaw.h"
#include <stdexcept>
#include <cassert>

using namespace smt;

namespace ratio
{
graph::graph(solver &slv) : slv(slv), gamma(slv.get_sat_core().new_var()) {}
graph::~graph() {}

void graph::new_flaw(flaw &f)
{
    // we initialize the flaw..
    f.init(); // flaws' initialization requires being at root-level..
#ifdef BUILD_GUI
    slv.fire_new_flaw(f);
#endif
    switch (slv.sat.value(f.phi))
    {
    case True: // we have a top-level (a landmark) flaw..
        slv.flaws.insert(&f);
        break;
    case Undefined: // we listen for the flaw to become active..
        phis[f.phi].push_back(&f);
        slv.bind(f.phi);
        break;
    }

    // we enqueue the flaw..
    flaw_q.push_back(&f);
}

void graph::new_resolver(resolver &r)
{
#ifdef BUILD_GUI
    slv.fire_new_resolver(r);
#endif
    if (slv.sat.value(r.rho) == Undefined) // we do not have a top-level (a landmark) resolver..
    {
        // we listen for the resolver to become inactive..
        rhos[r.rho].push_back(&r);
        slv.bind(r.rho);
    }
}

void graph::new_causal_link(flaw &f, resolver &r)
{
#ifdef BUILD_GUI
    slv.fire_causal_link_added(f, r);
#endif
    r.preconditions.push_back(&f);
    f.supports.push_back(&r);
    bool new_clause = slv.sat.new_clause({lit(r.rho, false), f.phi});
    assert(new_clause);
}

void graph::set_estimated_cost(resolver &r, const rational &cst)
{
    if (r.est_cost != cst)
    {
        if (!slv.trail.empty()) // we store the current resolver's estimated cost, if not already stored, for allowing backtracking..
            slv.trail.back().old_r_costs.try_emplace(&r, r.est_cost);

        // we update the resolver's estimated cost..
        r.est_cost = cst;
#ifdef BUILD_GUI
        slv.fire_resolver_cost_changed(r);
#endif

        resolver *bst_res = r.effect.get_best_resolver();
        // this is the new cost of the resolver's effect..
        rational efct_cost = bst_res ? bst_res->get_estimated_cost() : rational::POSITIVE_INFINITY;
        if (r.effect.est_cost != efct_cost) // the cost of the resolver's effect has changed as a consequence of the resolver's cost update, hence, we propagate the update to all the supports of the resolver's effect..
        {
            if (!slv.trail.empty()) // we store the current resolver's effect's estimated cost, if not already stored, for allowing backtracking..
                slv.trail.back().old_f_costs.try_emplace(&r.effect, r.effect.est_cost);

            // we update the resolver's effect's estimated cost..
            r.effect.est_cost = efct_cost;
#ifdef BUILD_GUI
            slv.fire_flaw_cost_changed(r.effect);
#endif

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
                    if (!slv.trail.empty()) // we store the current resolver's estimated cost, if not already stored, for allowing backtracking..
                        slv.trail.back().old_r_costs.try_emplace(&c_res, c_res.est_cost);

                    // we update the resolver's estimated cost..
                    c_res.est_cost = r_cost;
#ifdef BUILD_GUI
                    slv.fire_resolver_cost_changed(c_res);
#endif

                    bst_res = c_res.effect.get_best_resolver();
                    // this is the new cost of the resolver's effect..
                    efct_cost = bst_res ? bst_res->get_estimated_cost() : rational::POSITIVE_INFINITY;

                    if (c_res.effect.est_cost != efct_cost) // the cost of the resolver's effect has changed as a consequence of the resolver's cost update..
                    {
                        if (!slv.trail.empty()) // we store the current resolver's effect's estimated cost, if not already stored, for allowing backtracking..
                            slv.trail.back().old_f_costs.try_emplace(&c_res.effect, c_res.effect.est_cost);

                        // we update the resolver's effect's estimated cost..
                        c_res.effect.est_cost = efct_cost;
#ifdef BUILD_GUI
                        slv.fire_flaw_cost_changed(c_res.effect);
#endif

                        for (const auto &sup_r : c_res.effect.supports) // we propagate the update to all the supports of the resolver's effect..
                            resolver_q.push(sup_r);
                    }
                }
                resolver_q.pop();
            }
        }
    }
}

const smt::rational graph::evaluate(const std::vector<flaw *> &fs)
{
    rational c_cost;
#ifdef H_MAX
    c_cost = rational::NEGATIVE_INFINITY;
    for (const auto &f : fs)
        if (!f->is_expanded())
            return rational::POSITIVE_INFINITY;
        else
        {
            rational c = f->get_estimated_cost();
            if (c > c_cost)
                c_cost = c;
        }
#endif
#ifdef H_ADD
    for (const auto &f : fs)
        return rational::POSITIVE_INFINITY;
    else c_cost += f->get_estimated_cost();
#endif
    return c_cost;
}

void graph::build()
{
    assert(slv.get_sat_core().root_level());
    LOG("building the causal graph..");

    while (std::any_of(slv.flaws.begin(), slv.flaws.end(), [&](flaw *f) { return f->get_estimated_cost().is_positive_infinite(); }))
    {
        if (flaw_q.empty())
            throw std::runtime_error("the problem is inconsistent..");
#ifdef DEFERRABLE_FLAWS
        assert(!flaw_q.front()->expanded);
        if (slv.get_sat_core().value(flaw_q.front()->phi) != False)
            if (is_deferrable(*flaw_q.front())) // we have a deferrable flaw: we can postpone its expansion..
                flaw_q.push_back(flaw_q.front());
            else
                expand_flaw(*flaw_q.front()); // we expand the flaw..
        flaw_q.pop_front();
#else
        for (const auto &f : std::move(flaw_q))
            expand_flaw(*f); // we expand the flaw..
#endif
    }
}

void graph::add_layer()
{
    assert(slv.get_sat_core().root_level());
    LOG("adding a layer to the causal graph..");

    std::deque<flaw *> f_q(flaw_q);
    while (std::all_of(f_q.begin(), f_q.end(), [&](flaw *f) { return f->get_estimated_cost().is_infinite(); }))
    {
        if (flaw_q.empty())
            throw std::runtime_error("the problem is inconsistent..");
        std::deque<flaw *> c_q = std::move(flaw_q);
        for (const auto &f : c_q)
            if (slv.get_sat_core().value(f->phi) != False) // we expand the flaw..
                expand_flaw(*f);
    }
}

void graph::increase_accuracy()
{
    assert(slv.get_sat_core().root_level());
    accuracy++;
    LOG("current heuristic accuracy: " + std::to_string(accuracy));

    // we clean up composite flaws trivial flaws and already solved flaws..
    for (auto it = slv.flaws.begin(); it != slv.flaws.end();)
        if (composite_flaw *sf = dynamic_cast<composite_flaw *>(*it))
            // we remove the composite flaw from the current flaws..
            slv.flaws.erase(it++);
        else if (std::any_of((*it)->resolvers.begin(), (*it)->resolvers.end(), [&](resolver *r) { return slv.get_sat_core().value(r->rho) == True; }))
        {
            // we have either a trivial (i.e. has only one resolver) or an already solved flaw..
            assert(slv.get_sat_core().value((*std::find_if((*it)->resolvers.begin(), (*it)->resolvers.end(), [&](resolver *r) { return slv.get_sat_core().value(r->rho) != False; }))->rho) == True);
            // we remove the flaw from the current flaws..
            slv.flaws.erase(it++);
        }
        else
            ++it;

    flaw_q.clear();
    if (slv.flaws.size() >= accuracy)
    {
        std::vector<std::vector<flaw *>> fss = combinations(std::vector<flaw *>(slv.flaws.begin(), slv.flaws.end()), accuracy);
        for (const auto &fs : fss) // we create a new composite flaw..
            new_flaw(*new composite_flaw(*this, res, fs));
    }
    else // we create a new composite flaw..
        new_flaw(*new composite_flaw(*this, res, std::vector<flaw *>(slv.flaws.begin(), slv.flaws.end())));

    // we restart the building graph procedure..
    build();
}

void graph::expand_flaw(flaw &f)
{
    assert(!f.expanded);

    // we expand the flaw..
    if (composite_flaw *sf = dynamic_cast<composite_flaw *>(&f))
        // we expand the unexpanded enclosing flaws..
        for (const auto &c_f : sf->flaws)
            if (!c_f->expanded)
            {
                assert(!c_f->expanded);
#ifdef BUILD_GUI
                slv.fire_current_flaw(*c_f);
#endif
                // we expand the enclosing flaw..
                c_f->expand();
                // ..and remove it from the flaw queue..
                if (auto f_it = std::find(flaw_q.begin(), flaw_q.end(), c_f); f_it != flaw_q.end())
                    flaw_q.erase(f_it);

                // we apply the enclosing flaw's resolvers..
                for (const auto &r : c_f->resolvers)
                    apply_resolver(*r);
            }
    f.expand();

    for (const auto &r : f.resolvers)
        apply_resolver(*r);

    if (!slv.get_sat_core().check())
        throw std::runtime_error("the problem is inconsistent..");
}

void graph::apply_resolver(resolver &r)
{
    res = &r;
    slv.set_ni(r.rho);
    try
    {
        r.apply();
    }
    catch (const std::runtime_error &)
    {
        if (!slv.get_sat_core().new_clause({lit(r.rho, false)}))
            throw std::runtime_error("the problem is inconsistent..");
    }

    slv.restore_ni();
    res = nullptr;
    if (r.preconditions.empty() && slv.get_sat_core().value(r.rho) != False) // there are no requirements for this resolver..
        set_estimated_cost(r, 0);
}

#ifdef DEFERRABLE_FLAWS
bool graph::is_deferrable(flaw &f)
{
    std::queue<flaw *> q;
    q.push(&f);
    while (!q.empty())
    {
        assert(slv.get_sat_core().value(q.front()->phi) != False);
        if (q.front()->get_estimated_cost() < rational::POSITIVE_INFINITY) // we already have a possible solution for this flaw, thus we defer..
            return true;
        for (const auto &r : q.front()->causes)
            q.push(&r->effect);
        q.pop();
    }
    // we have an undeferrable flaw..
    return false;
}
#endif

void graph::set_new_gamma()
{
    // we create a new graph var..
    gamma = slv.get_sat_core().new_var();
    LOG("graph var is: γ" << std::to_string(gamma));
#ifdef GRAPH_PRUNING
    // these flaws have not been expanded, hence, cannot have a solution..
    for (const auto &f : flaw_q)
        if (!slv.get_sat_core().new_clause({lit(gamma, false), lit(f->phi, false)}))
            throw std::runtime_error("the problem is inconsistent..");
#endif
    // we use the new graph var to allow search within the new graph..
    LOG("assuming γ " << std::to_string(gamma));
    slv.take_decision(gamma);
}

flaw::flaw(graph &gr, const std::vector<resolver *> &causes, const bool &exclusive) : gr(gr), causes(causes), supports(causes), exclusive(exclusive)
{
}
flaw::~flaw() {}

resolver *flaw::get_best_resolver() const
{
    resolver *c_res = nullptr;
    rational c_cost = rational::POSITIVE_INFINITY;
    for (const auto &r : resolvers)
        if (gr.get_solver().get_sat_core().value(r->get_rho()) != False && r->get_estimated_cost() < c_cost)
        {
            c_res = r;
            c_cost = r->get_estimated_cost();
        }
    return c_res;
}

void flaw::init()
{
    assert(!expanded);
    assert(gr.get_solver().get_sat_core().root_level());

    // we add this flaw to the preconditions of its causes..
    for (const auto &r : causes)
        r->preconditions.push_back(this);

    if (causes.empty())
        // this flaw is necessarily active..
        phi = TRUE_var;
    else
    {
        // we create a new variable..
        std::vector<lit> cs;
        for (const auto &c : causes)
            cs.push_back(c->rho);

        // this flaw is active iff the conjunction of its causes is active..
        phi = gr.get_solver().get_sat_core().new_conj(cs);
    }
}

void flaw::expand()
{
    assert(!expanded);
    assert(gr.get_solver().get_sat_core().root_level());
    expanded = true; // the flaw is now expanded..

    // we compute the resolvers..
    compute_resolvers();

    // we add causal relations between the flaw and its resolvers (i.e., if the flaw is phi exactly one of its resolvers should be in plan)..
    if (resolvers.empty())
    {
        // there is no way for solving this flaw..
        if (!gr.get_solver().get_sat_core().new_clause({lit(phi, false)})) // we force the phi variable at false..
            throw std::runtime_error("the problem is inconsistent..");
    }
    else
    {
        // we need to take a decision for solving this flaw..
        std::vector<lit> r_chs;
        for (const auto &r : resolvers)
            r_chs.push_back(r->rho);
        // we link the phi variable to the resolvers' rho variables..
        if (!(exclusive ? gr.get_solver().get_sat_core().exct_one(r_chs, phi) : gr.get_solver().get_sat_core().disj(r_chs, phi)))
            throw std::runtime_error("the problem is inconsistent..");
    }
}

void flaw::add_resolver(resolver &r)
{
    resolvers.push_back(&r);
    gr.new_resolver(r);
}

resolver::resolver(graph &gr, const smt::rational &cost, flaw &eff) : resolver(gr, gr.get_solver().get_sat_core().new_var(), cost, eff) {}
resolver::resolver(graph &gr, const smt::var &r, const smt::rational &cost, flaw &eff) : gr(gr), rho(r), intrinsic_cost(cost), effect(eff) {}
resolver::~resolver() {}
} // namespace ratio
