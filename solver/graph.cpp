#include "graph.h"
#include "solver.h"
#include "flaw.h"
#include "resolver.h"
#include "combinations.h"
#include "composite_flaw.h"
#include <cassert>

using namespace smt;

namespace ratio
{
graph::graph(solver &slv) : slv(slv), gamma(slv.get_sat_core().new_var()) { LOG("graph var is γ" << std::to_string(gamma)); }
graph::~graph() {}

void graph::new_flaw(flaw &f, const bool &enqueue)
{
    // we initialize the flaw..
    f.init(); // flaws' initialization requires being at root-level..
#ifdef BUILD_GUI
    slv.fire_new_flaw(f);
#endif

    if (enqueue) // we enqueue the flaw..
        flaw_q.push_back(&f);
    else // we directly expand the flaw..
        expand_flaw(f);

    switch (slv.sat.value(f.phi))
    {
    case True: // we have a top-level (a landmark) flaw..
        if (enqueue || std::none_of(f.resolvers.begin(), f.resolvers.end(), [this](resolver *r) { return slv.sat.value(r->rho) == True; }))
            slv.flaws.insert(&f); // the flaw has not yet already been solved (e.g. it has a single resolver)..
        break;
    case Undefined: // we listen for the flaw to become active..
        phis[f.phi].push_back(&f);
        slv.bind(f.phi);
        break;
    }
}

void graph::new_resolver(resolver &r)
{
#ifdef BUILD_GUI
    slv.fire_new_resolver(r);
#endif
    if (slv.sat.value(r.rho) == Undefined) // we do not have a top-level (a landmark) resolver, nor an infeasible one..
    {
        // we listen for the resolver to become inactive..
        rhos[r.rho].push_back(&r);
        slv.bind(r.rho);
    }
}

void graph::new_causal_link(flaw &f, resolver &r)
{
    f.make_precondition_of(r);
    bool new_clause = slv.sat.new_clause({lit(r.rho, false), f.phi});
    assert(new_clause);
#ifdef BUILD_GUI
    slv.fire_causal_link_added(f, r);
#endif

#ifdef CHECK_CYCLES
    for (const auto &cycle : circuits(f, r))
    {
        std::vector<lit> no_cycle;
        no_cycle.reserve(cycle.size());
        for (const auto &res : cycle)
            no_cycle.push_back(lit(res->rho, false));
        if (!slv.get_sat_core().new_clause(no_cycle))
            throw unsolvable_exception();
    }
#endif
}

void graph::set_estimated_cost(flaw &f, std::unordered_set<flaw *> &visited)
{
    rational c_cost; // the current cost..
    if (slv.get_sat_core().value(f.phi) == False)
        c_cost = rational::POSITIVE_INFINITY;
    else
    {
        resolver *bst_res = f.get_best_resolver(); // the flaw's best resolver..
        c_cost = bst_res ? bst_res->get_estimated_cost() : rational::POSITIVE_INFINITY;
    }
    if (f.est_cost == c_cost)
        return; // nothing to propagate..
    else if (visited.count(&f))
    { // we are propagating costs within a cycle..
        c_cost = rational::POSITIVE_INFINITY;
        if (f.est_cost == c_cost)
            return; // nothing to propagate..
    }

    if (!slv.trail.empty()) // we store the current flaw's estimated cost, if not already stored, for allowing backtracking..
        slv.trail.back().old_f_costs.try_emplace(&f, f.est_cost);

    // we update the flaw's estimated cost..
    f.est_cost = c_cost;
#ifdef BUILD_GUI
    slv.fire_flaw_cost_changed(f);
#endif

    // we (try to) update the estimated costs of the supports' effects and enqueue them for cost propagation..
    visited.insert(&f);
    for (const auto &supp : f.supports)
        set_estimated_cost(supp->effect, visited);
    visited.erase(&f);
}

void graph::build()
{
    assert(slv.get_sat_core().root_level());
    LOG("building the causal graph..");

    while (std::any_of(slv.flaws.begin(), slv.flaws.end(), [](flaw *f) { return f->get_estimated_cost().is_positive_infinite(); }))
    {
        if (flaw_q.empty())
            throw unsolvable_exception();
#ifdef DEFERRABLE_FLAWS
        assert(!flaw_q.front()->expanded);
        if (slv.get_sat_core().value(flaw_q.front()->phi) != False)
            if (is_deferrable(*flaw_q.front())) // we have a deferrable flaw: we can postpone its expansion..
                flaw_q.push_back(flaw_q.front());
            else
                expand_flaw(*flaw_q.front()); // we expand the flaw..
        flaw_q.pop_front();
#else
        size_t q_size = flaw_q.size();
        for (size_t i = 0; i < q_size; ++i)
        {
            if (slv.get_sat_core().value(flaw_q.front()->phi) != False)
                expand_flaw(*flaw_q.front()); // we expand the flaw..
            flaw_q.pop_front();
        }
#endif
    }
}

void graph::add_layer()
{
    assert(slv.get_sat_core().root_level());
    LOG("adding a layer to the causal graph..");

    std::deque<flaw *> f_q(flaw_q);
    while (std::all_of(f_q.begin(), f_q.end(), [](flaw *f) { return f->get_estimated_cost().is_infinite(); }))
    {
        if (flaw_q.empty())
            throw unsolvable_exception();
        size_t q_size = flaw_q.size();
        for (size_t i = 0; i < q_size; ++i)
        {
            if (slv.get_sat_core().value(flaw_q.front()->phi) != False)
                expand_flaw(*flaw_q.front()); // we expand the flaw..
            flaw_q.pop_front();
        }
    }
}

void graph::set_accuracy(const unsigned short &acc)
{
    assert(slv.get_sat_core().root_level());
    accuracy = acc;
    LOG("current heuristic accuracy: " + std::to_string(accuracy));

    // we clean up composite flaws..
    for (auto it = slv.flaws.begin(); it != slv.flaws.end();)
        if (composite_flaw *sf = dynamic_cast<composite_flaw *>(*it))
            // we remove the composite flaw from the current flaws..
            it = slv.flaws.erase(it);
        else
            ++it;

    if (slv.flaws.size() >= accuracy)
    {
        const auto fss = combinations(std::vector<flaw *>(slv.flaws.begin(), slv.flaws.end()), accuracy);
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

    if (composite_flaw *sf = dynamic_cast<composite_flaw *>(&f))
        // we expand the unexpanded enclosing flaws..
        for (const auto &enc_f : sf->flaws)
            if (!enc_f->expanded)
            {
                if (auto f_it = std::find(flaw_q.begin(), flaw_q.end(), enc_f); f_it != flaw_q.end())
                    flaw_q.erase(f_it); // we remove the enclosing flaw from the flaw queue..
                // we expand the enclosing flaw..
                enc_f->expand();

                // we apply the enclosing flaw's resolvers..
                for (const auto &r : enc_f->resolvers)
                    apply_resolver(*r);

                std::unordered_set<flaw *> c_visited;
                set_estimated_cost(*enc_f, c_visited);
                assert(c_visited.empty());
            }

    // we expand the flaw..
    f.expand();

    // we apply the flaw's resolvers..
    for (const auto &r : f.resolvers)
        apply_resolver(*r);

    if (!slv.get_sat_core().check())
        throw unsolvable_exception();

    // we clean up already solved flaws..
    if (slv.sat.value(f.phi) == True && std::any_of(f.resolvers.begin(), f.resolvers.end(), [this](resolver *r) { return slv.sat.value(r->rho) == True; }))
        slv.flaws.erase(&f); // this flaw has already been solved..

    std::unordered_set<flaw *> c_visited;
    set_estimated_cost(f, c_visited);
    assert(c_visited.empty());
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
            throw unsolvable_exception();
    }

    slv.restore_ni();
    res = nullptr;

#ifdef GRAPH_REFINING
    if (r.preconditions.empty() && slv.get_sat_core().value(r.rho) == Undefined)
        empty_precs_resolvers.insert(&r);
#endif
}

#ifdef DEFERRABLE_FLAWS
bool graph::is_deferrable(flaw &f)
{
    if (slv.get_sat_core().value(f.phi) == True)
        return false;
    else if (composite_flaw *cf = dynamic_cast<composite_flaw *>(&f))
        if (std::any_of(cf->flaws.begin(), cf->flaws.end(), [this](flaw *sf) { return is_deferrable(*sf); }))
            return true;

    std::queue<flaw *> q;
    q.push(&f);
    while (!q.empty())
    {
        assert(slv.get_sat_core().value(q.front()->phi) != False);
        if (q.front()->get_estimated_cost() < rational::POSITIVE_INFINITY || std::any_of(q.front()->resolvers.begin(), q.front()->resolvers.end(), [this](resolver *r) { return slv.sat.value(r->rho) == True; }))
            return true; // we already have a possible solution for this flaw, thus we defer..
        else if (slv.get_sat_core().value(q.front()->phi) == True)
            return false; // we necessarily have to solve this flaw: it cannot be deferred..
        for (const auto &r : q.front()->causes)
            q.push(&r->effect);
        q.pop();
    }
    // we have an undeferrable flaw..
    return false;
}
#endif

#ifdef CHECK_CYCLES
// finds all the simple cycles in the graph using Johnson's algorithm..
// this algorithm is an adaptation of D.B.Johnson, "Finding all the elementary circuits of a directed graph", SIAM J. Comput., 4 (1975), pp. 77-84..
bool circuit(const flaw &s, const resolver &v, std::vector<resolver const *> &stack, std::unordered_set<flaw const *> &blocked_set, std::unordered_map<flaw const *, std::vector<flaw const *>> &blocked_map, std::vector<std::vector<resolver const *>> &circuits)
{
    bool f = false;
    stack.push_back(&v);
    blocked_set.emplace(&v.get_effect());
    for (const auto &supp : v.get_effect().get_supports())
        if (v.get_graph().get_solver().get_sat_core().value(supp->get_rho()) != False)
            if (&supp->get_effect() == &s)
            { // we have found a cycle..
                stack.push_back(supp);
                circuits.push_back(stack);
                stack.pop_back();
                f = true;
            }
            else if (!blocked_set.count(&supp->get_effect()) && circuit(s, *supp, stack, blocked_set, blocked_map, circuits)) // we explore this neighbour only if it is not in blocked_set..
                f = true;

    if (f)
    { // we unblock the vertex and all vertices which are dependent on this vertex..
        std::queue<flaw const *> q;
        q.push(&v.get_effect());
        while (!q.empty())
        {
            blocked_set.erase(q.front());
            if (const auto q_it = blocked_map.find(q.front()); q_it != blocked_map.end())
            {
                for (const auto &blckd : q_it->second)
                    q.push(blckd);
                blocked_map.erase(q_it);
            }
            q.pop();
        }
    }
    else // if any of these neighbours ever get unblocked, we unblock the current vertex as well..
        for (const auto &supp : v.get_effect().get_supports())
            if (v.get_graph().get_solver().get_sat_core().value(supp->get_rho()) != False)
                blocked_map[&supp->get_effect()].push_back(&v.get_effect());

    // we remove vertex from the stack..
    stack.pop_back();
    return f;
}

std::vector<std::vector<resolver const *>> graph::circuits(flaw &f, resolver &r) const
{
    std::vector<resolver const *> stack;
    std::unordered_set<flaw const *> blocking_set;
    std::unordered_map<flaw const *, std::vector<flaw const *>> blocking_map;
    std::vector<std::vector<resolver const *>> crts;
    circuit(f, r, stack, blocking_set, blocking_map, crts);
    return crts;
}
#endif

void graph::check_gamma()
{
    assert(slv.root_level());
    if (slv.get_sat_core().value(gamma) != False) // a unit clause, not gamma, has been deduced or, simply, gamma has never been set..
        build();                                  // the search procedure may have excluded those parts of the graph that could lead to a solution..
    else
    { // the graph has been invalidated..
        LOG("search has exhausted the graph..");
        // do we have room for increasing the heuristic accuracy?
        if (accuracy < MAX_ACCURACY)
            set_accuracy(accuracy + 1); // we increase the heuristic accuracy..
        else
            add_layer(); // we add a layer to the current graph..
        // we create a new graph var..
        gamma = slv.get_sat_core().new_var();
#ifdef GRAPH_PRUNING
        already_closed.clear();
#endif
        LOG("graph var is: γ" << std::to_string(gamma));
    }
#ifdef GRAPH_PRUNING
    // these flaws have not been expanded, hence, cannot have a solution..
    for (const auto &f : flaw_q)
        if (already_closed.insert(f->phi).second)
            if (!slv.get_sat_core().new_clause({lit(gamma, false), lit(f->phi, false)}))
                throw unsolvable_exception();
#endif
    slv.take_decision(gamma);
}
} // namespace ratio
