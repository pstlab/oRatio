#include "graph.h"
#include "solver.h"
#include "flaw.h"
#include "resolver.h"
#include "combinations.h"
#include "refinement_flaw.h"
#include "mutex_flaw.h"
#include "smart_type.h"
#include "atom_flaw.h"
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
    r.preconditions.push_back(&f);
    f.supports.push_back(&r);
    bool new_clause = slv.sat.new_clause({lit(r.rho, false), f.phi});
    assert(new_clause);
#ifdef BUILD_GUI
    slv.fire_causal_link_added(f, r);
#endif

#ifdef CHECK_CYCLES
    LOG("checking cycles..");
    std::vector<std::vector<resolver const *>> cs = circuits(f, r);
#ifdef BUILD_GUI
    if (!cs.empty())
        LOG("found " << std::to_string(cs.size()) << " cycles..");
#endif
    for (const auto &cycle : cs)
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
            assert(!flaw_q.front()->expanded);
            if (slv.get_sat_core().value(flaw_q.front()->phi) != False)
                expand_flaw(*flaw_q.front()); // we expand the flaw..
            flaw_q.pop_front();
        }
#endif
    }

    // we extract the inconsistencies (and translate them into flaws)..
    std::vector<std::vector<std::pair<lit, double>>> incs = slv.get_incs();
    for (const auto &f : slv.pending_flaws)
        new_flaw(*f, false); // we add the flaws to the planning graph..
    slv.pending_flaws.clear();

    // we perform some cleanings..
    slv.get_sat_core().simplify_db();
}

void graph::add_layer()
{
    assert(slv.get_sat_core().root_level());
    assert(std::none_of(slv.flaws.begin(), slv.flaws.end(), [](flaw *f) { return f->get_estimated_cost().is_positive_infinite(); }));
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

void graph::expand_flaw(flaw &f)
{
    assert(!f.expanded);

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
    catch (const inconsistency_exception &)
    {
        if (!slv.get_sat_core().new_clause({lit(r.rho, false)}))
            throw unsolvable_exception();
    }

    slv.restore_ni();
    res = nullptr;
}

#ifdef DEFERRABLE_FLAWS
bool graph::is_deferrable(flaw &f)
{
    if (slv.get_sat_core().value(f.phi) == True)
        return false;

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

#ifdef CHECK_GRAPH
void graph::check_graph()
{
    LOG("checking the graph..");
    std::unordered_set<resolver *> visited;
    std::unordered_set<flaw *> flaws(slv.flaws); // since the 'flaws' set can change within the check, we first make a copy..
    std::unordered_set<resolver *> inv_rs;

    while (!std::all_of(flaws.begin(), flaws.end(), [this, &visited, &inv_rs](flaw *f) { return check_flaw(*f, visited, inv_rs); }))
    {
        for (const auto &f : slv.pending_flaws)
        {
            new_flaw(*f, false);
            std::unordered_set<flaw *> c_visited;
            for (const auto &c : f->causes)
                set_estimated_cost(c->effect, c_visited);
        }
        slv.pending_flaws.clear();

        if (!slv.get_sat_core().check())
            throw unsolvable_exception();
        build();
    }
}

bool graph::check_flaw(flaw &f, std::unordered_set<resolver *> &visited, std::unordered_set<resolver *> &inv_rs)
{
    assert(f.expanded);
    assert(slv.get_sat_core().value(f.phi) == True);

#ifdef BUILD_GUI
    slv.fire_current_flaw(f);
#endif

    if (auto it = std::find_if(f.resolvers.begin(), f.resolvers.end(), [this](resolver *r) { return slv.get_sat_core().value(r->rho) == True; }); it != f.resolvers.end())
        // a resolver is already applied for the given flaw..
        return std::all_of((*it)->preconditions.begin(), (*it)->preconditions.end(), [this, &visited, &inv_rs](flaw *f) { return check_flaw(*f, visited, inv_rs); });
    else
    {
        // we sort the flaws' resolvers..
        std::sort(f.resolvers.begin(), f.resolvers.end(), [](resolver *r0, resolver *r1) { return r0->get_estimated_cost() < r1->get_estimated_cost(); });

        size_t c_lvl = slv.decision_level();
        for (const auto &r : f.resolvers)
            if (r->get_estimated_cost().is_infinite())
                break;
            else if (visited.insert(r).second)
            {
#ifdef BUILD_GUI
                slv.fire_current_resolver(*r);
#endif
                // we assume the resolver's rho variable..
                slv.current_decision = r->rho;
                if (!slv.get_sat_core().assume(r->rho) || !slv.get_sat_core().check())
                    throw unsolvable_exception();

                if (c_lvl < slv.decision_level())
                { // the resolver is applicable..
                    // we refine the graph taking into account mutexe resolvers..
                    for (const auto &f : to_enqueue)
                        if (&r->effect != f)
                        {
                            assert(std::any_of(f->resolvers.begin(), f->resolvers.end(), [this](resolver *c_r) { return slv.sat.value(c_r->rho) == False; }));
                            std::vector<resolver *> non_mtx_rs;
                            for (const auto &r : f->resolvers)
                                if (slv.sat.value(r->rho) != False)
                                    non_mtx_rs.push_back(r);
                            slv.pending_flaws.push_back(new mutex_flaw(*this, r, *f, non_mtx_rs));
                        }
                    to_enqueue.clear();

                    if (std::none_of(slv.flaws.begin(), slv.flaws.end(), [](flaw *f) { return f->get_estimated_cost().is_positive_infinite(); }))
                    { // an estimated solution for the given flaw has been found..
                        if (dynamic_cast<atom_flaw::unify_atom *>(r) || std::all_of(r->preconditions.begin(), r->preconditions.end(), [this, &visited, &inv_rs](flaw *f) { return check_flaw(*f, visited, inv_rs); }))
                        { // either 'r' is a valid unification or all of its preconditions are applicable..
                            visited.erase(r);
                            slv.get_sat_core().pop();
                            assert(c_lvl == slv.decision_level());
                            return true;
                        }
                        slv.get_sat_core().pop();
                        assert(c_lvl == slv.decision_level());
                    }
                    else
                    { // the resolver is applicable but incompatible with the current graph..
                        slv.get_sat_core().pop();
                        assert(c_lvl == slv.decision_level());
                    }
                }
                else
                    inv_rs.insert(r);

                visited.erase(r);
            }
    }

    // it is not possible to estimate a solution for the given flaw..
    return false;
}
#endif

#ifdef CHECK_MUTEXES
void graph::check_mutexes()
{
    LOG("checking for mutexes..");
    std::set<var> c_rhos;
    for (const auto &r : rhos)
        c_rhos.insert(r.first);

    for (const auto &r : c_rhos)
        if (slv.get_sat_core().value(r) == Undefined && std::none_of(rhos[r].begin(), rhos[r].end(), [](resolver *r) { return r->get_estimated_cost().is_positive_infinite(); }))
        {
            slv.current_decision = r;
            if (!slv.get_sat_core().assume(r) || !slv.get_sat_core().check())
                throw unsolvable_exception();
            if (slv.decision_level())
            { // the resolver is applicable..
                slv.get_sat_core().pop();
                assert(slv.root_level());
            }
        }
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
        // we create a new graph var..
        gamma = slv.get_sat_core().new_var();
        LOG("graph var is: γ" << std::to_string(gamma));
#if defined GRAPH_PRUNING || defined CHECK_GRAPH
        already_closed.clear();
#endif
        add_layer(); // we add a layer to the current graph..
    }

#ifdef CHECK_GRAPH
    check_graph();
#endif

#ifdef GRAPH_PRUNING
    LOG("pruning the graph..");
    // these flaws have not been expanded, hence, cannot have a solution..
    for (const auto &f : flaw_q)
        if (already_closed.insert(f->phi).second)
            if (!slv.get_sat_core().new_clause({lit(gamma, false), lit(f->phi, false)}))
                throw unsolvable_exception();
#endif

    slv.take_decision(gamma);
}
} // namespace ratio
