#include "graph.h"
#include "solver.h"
#include "flaw.h"
#include "resolver.h"
#include "combinations.h"
#include "refinement_flaw.h"
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
        flaw *c_f = flaw_q.front();
        flaw_q.pop_front();
#ifdef DEFERRABLE_FLAWS
        assert(!c_f->expanded);
        if (slv.get_sat_core().value(c_f->phi) != False)
        {
            std::unordered_set<flaw *> visited;
            if (is_deferrable(*c_f, visited)) // we have a deferrable flaw: we can postpone its expansion..
                flaw_q.push_back(c_f);
            else
                expand_flaw(*c_f); // we expand the flaw..
            assert(visited.empty());
        }
#else
        size_t q_size = flaw_q.size();
        for (size_t i = 0; i < q_size; ++i)
        {
            assert(!c_f->expanded);
            if (slv.get_sat_core().value(c_f->phi) != False)
                expand_flaw(*c_f); // we expand the flaw..
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
            flaw *c_f = flaw_q.front();
            flaw_q.pop_front();
            if (slv.get_sat_core().value(c_f->phi) != False)
                expand_flaw(*c_f); // we expand the flaw..
        }
    }

    // we extract the inconsistencies (and translate them into flaws)..
    std::vector<std::vector<std::pair<lit, double>>> incs = slv.get_incs();
    for (const auto &f : slv.pending_flaws)
        new_flaw(*f, false); // we add the flaws to the planning graph..
    slv.pending_flaws.clear();

    // we perform some cleanings..
    slv.get_sat_core().simplify_db();
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
bool graph::is_deferrable(flaw &f, std::unordered_set<flaw *> &visited)
{
    if (f.get_estimated_cost() < rational::POSITIVE_INFINITY || std::any_of(f.resolvers.begin(), f.resolvers.end(), [this](resolver *r) { return slv.sat.value(r->rho) == True; }))
        return true; // we already have a possible solution for this flaw, thus we defer..
    if (slv.get_sat_core().value(f.phi) == True || visited.count(&f))
        return false; // we necessarily have to solve this flaw: it cannot be deferred..
    visited.insert(&f);
    bool def = std::all_of(f.supports.begin(), f.supports.end(), [this, &visited](resolver *r) { return is_deferrable(r->get_effect(), visited); });
    visited.erase(&f);
    return def;
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
    checking = true;
    std::unordered_set<flaw *> visited;                                  // the visited flaws..
    std::unordered_set<resolver *> inv_rs;                               // these resolvers cannot be applied..
    std::unordered_set<resolver *> inc_rs;                               // these resolvers cannot be applied within the current grahp..
    std::vector<refinement_flaw *> ref_fs;                               // the enqueued refinement flaws, will be added as soon as we are at root-level..
    std::unordered_map<resolver *, std::unordered_set<flaw *>> csl_lnks; // a map of those resolvers (i.e., the key) to whome add deduced causal links..

    bool ok;
    do
    {
        ok = true;
        // since the 'flaws' set can change within the check, we first make a copy..
        for (const auto &f : std::unordered_set<flaw *>(slv.flaws))
            if (!check_flaw(*f, visited, inv_rs, inc_rs, ref_fs, csl_lnks))
                ok = false;
        if (!inv_rs.empty() || !ref_fs.empty() || !csl_lnks.empty())
            ok = false;

        // we prune away the invalid resolvers (what about their preconditions?)..
        for (const auto &inv_r : inv_rs)
            if (!slv.get_sat_core().new_clause({lit(inv_r->rho, false)}))
                throw unsolvable_exception();

        // we add the refinement flaws to the planning graph..
        for (const auto &ref_f : ref_fs)
        {
            new_flaw(*ref_f, false);
            // we update the estimated costs..
            std::unordered_set<flaw *> c_visited;
            for (const auto &c : ref_f->causes)
                set_estimated_cost(c->effect, c_visited);
        }

        // we add new deduced causal links..
        for (const auto &c_r : csl_lnks)
        {
            for (const auto &to_enq : c_r.second)
                new_causal_link(*to_enq, *c_r.first);
            // we update the estimated costs..
            std::unordered_set<flaw *> c_visited;
            set_estimated_cost(c_r.first->effect, c_visited);
        }

        if (!slv.get_sat_core().check())
            throw unsolvable_exception();

        build();

        // we clean up things..
        ref_fs.clear();
        csl_lnks.clear();
        inv_rs.clear();
        if (!ok)
            inc_rs.clear();
        else if (!inc_rs.empty())
        { // the graph is, till now, good, but we have some inconsistent resolvers..
            for (const auto &r : inc_rs)
                if (already_closed.insert(r->rho).second)
                    if (!slv.get_sat_core().new_clause({lit(r->rho, false), lit(gamma, false)}))
                        throw unsolvable_exception();

            if (!slv.get_sat_core().check())
                throw unsolvable_exception();

            slv.current_decision = gamma;
            if (!slv.get_sat_core().assume(gamma) || !slv.get_sat_core().check())
                throw unsolvable_exception();
            if (slv.root_level())
            {
                assert(slv.get_sat_core().value(gamma) == False);
                // the graph has been invalidated..
                LOG("search has exhausted the graph..");
                // we create a new graph var..
                gamma = slv.get_sat_core().new_var();
                LOG("graph var is: γ" << std::to_string(gamma));
#if defined GRAPH_PRUNING || defined CHECK_GRAPH
                already_closed.clear();
#endif
                add_layer(); // we add a layer to the current graph..
                ok = false;
            }
            else if (std::any_of(slv.flaws.begin(), slv.flaws.end(), [](flaw *f) { return f->get_estimated_cost().is_positive_infinite(); }))
            { // the inconsistent resolvers do not allow finding a solution..
                slv.get_sat_core().pop();
                add_layer();
                ok = false;
            }
            else // we go back to search..
                slv.get_sat_core().pop();
            assert(slv.root_level());
        }
    } while (!ok);

    checking = false;
}

bool graph::check_flaw(flaw &f, std::unordered_set<flaw *> &visited, std::unordered_set<resolver *> &inv_rs, std::unordered_set<resolver *> &inc_rs, std::vector<refinement_flaw *> &ref_fs, std::unordered_map<resolver *, std::unordered_set<flaw *>> &csl_lnks)
{
    assert(f.expanded);
    assert(slv.get_sat_core().value(f.phi) == True);

#ifdef BUILD_GUI
    slv.fire_current_flaw(f);
#endif

    if (visited.insert(&f).second)
    { // we have not seen this flaw yet..
        if (auto it = std::find_if(f.resolvers.begin(), f.resolvers.end(), [this](resolver *r) { return slv.get_sat_core().value(r->rho) == True; }); it != f.resolvers.end())
            // a resolver is already applied for the given flaw..
            // we check its active preconditions (notice that preconditions might include consistency-flaws which, depending also from other resolvers, might not be active)..
            return std::all_of((*it)->preconditions.begin(), (*it)->preconditions.end(), [this, &visited, &inv_rs, &inc_rs, &ref_fs, &csl_lnks](flaw *f) { return slv.get_sat_core().value(f->phi) == Undefined || check_flaw(*f, visited, inv_rs, inc_rs, ref_fs, csl_lnks); });
        else
        {
            // we sort the flaws' resolvers according to their estimated cost..
            std::sort(f.resolvers.begin(), f.resolvers.end(), [](resolver *r0, resolver *r1) { return r0->get_estimated_cost() < r1->get_estimated_cost(); });

            size_t c_lvl = slv.decision_level();
            for (const auto &r : f.resolvers)
            {
                if (r->get_estimated_cost().is_infinite())
                    continue; // either the resolver cannot be applied or it would not lead to a solution..

#ifdef BUILD_GUI
                slv.fire_current_resolver(*r);
#endif
                assert(to_enqueue.empty());
                // we assume the resolver's rho variable..
                slv.current_decision = r->rho;
                if (!slv.get_sat_core().assume(r->rho) || !slv.get_sat_core().check())
                    throw unsolvable_exception();

                // we check whether the resolver has actually been applied..
                if (c_lvl < slv.decision_level())
                { // the resolver has actually been applied..
                    assert(slv.get_sat_core().value(r->rho) == True);

                    // we make some cleanings..
                    for (const auto &v_f : visited)
                    {
                        to_enqueue.erase(v_f); // we remove ancestor flaws..
                        if (refinement_flaw *rf = dynamic_cast<refinement_flaw *>(v_f))
                            to_enqueue.erase(&rf->to_enqueue); // we remove flaws represented by refinement-flaws within the ancestors..
                    }
                    for (auto f_it = to_enqueue.begin(), last = to_enqueue.end(); f_it != last;)
                        if (auto slv_res = std::find_if((*f_it)->resolvers.begin(), (*f_it)->resolvers.end(), [this](resolver *c_r) { return slv.sat.value(c_r->rho) == True; }); slv_res != (*f_it)->resolvers.end())
                        {
                            f_it = to_enqueue.erase(f_it); // we remove already solved flaws..
                            for (const auto &pre_f : (*slv_res)->preconditions)
                                csl_lnks[r].insert(pre_f); // .. we add deduced preconditions to the current resolver..
                        }
                        else if (refinement_flaw *rf = dynamic_cast<refinement_flaw *>(*f_it))
                            if (std::any_of(visited.begin(), visited.end(), [this, rf](flaw *v_f) { if (refinement_flaw *vrf = dynamic_cast<refinement_flaw *>(v_f)) return &rf->to_enqueue == &vrf->to_enqueue; else return &rf->to_enqueue == v_f; }))
                                f_it = to_enqueue.erase(f_it); // we remove refinement-flaws representing flaws within the ancestors..
                            else
                                ++f_it;
                        else
                            ++f_it;

                    // we refine the graph taking into account mutex resolvers..
                    for (const auto &f : to_enqueue)
                    { // we have a new flaw to append..
                        assert(std::any_of(f->resolvers.begin(), f->resolvers.end(), [this](resolver *c_r) { return slv.sat.value(c_r->rho) == False; }));
                        assert(std::none_of(f->resolvers.begin(), f->resolvers.end(), [this](resolver *c_r) { return slv.sat.value(c_r->rho) == True; }));
                        // the still activable resolvers..
                        std::vector<resolver *> non_mtx_rs;
                        for (const auto &r : f->resolvers)
                            if (slv.sat.value(r->rho) != False)
                                non_mtx_rs.push_back(r);
                        assert(non_mtx_rs.size() > 1); // otherwise the flaw would have become singleton..
                        ref_fs.push_back(new refinement_flaw(*this, r, *f, non_mtx_rs));
                    }
                    to_enqueue.clear();

                    // we check whether an estimated solution for the given problem is still possible with the current assignments..
                    if (std::none_of(slv.flaws.begin(), slv.flaws.end(), [](flaw *f) { return f->get_estimated_cost().is_positive_infinite(); }))
                    { // we still have an estimated solution, hence we check for the resolver's active preconditions (again, resolver's preconditions might include consistency-flaws which, depending also from other resolvers, might not be active)..
                        if (dynamic_cast<atom_flaw::unify_atom *>(r) || std::all_of(r->preconditions.begin(), r->preconditions.end(), [this, &visited, &inv_rs, &inc_rs, &ref_fs, &csl_lnks](flaw *f) { return slv.get_sat_core().value(f->phi) == Undefined || check_flaw(*f, visited, inv_rs, inc_rs, ref_fs, csl_lnks); }))
                        { // either 'r' is a valid unification or all of its preconditions are applicable..
                            visited.erase(&f);
                            slv.get_sat_core().pop();
                            assert(c_lvl == slv.decision_level());
                            return true;
                        }
                        slv.get_sat_core().pop();
                        assert(c_lvl == slv.decision_level());
                    }
                    else
                    { // the resolver is applicable but incompatible with the current graph..
                        inc_rs.insert(r);
                        slv.get_sat_core().pop();
                        assert(c_lvl == slv.decision_level());
                    }
                }
                else
                { // 'r' is an invalid resolver (i.e. its constraints do not propagate)..
                    inv_rs.insert(r);
                    to_enqueue.clear(); // we clean the set of flaws affected by mutexes..
                }
            }
        }
        visited.erase(&f);
    }

    // it is not possible to estimate a solution for the given flaw..
    return false;
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
