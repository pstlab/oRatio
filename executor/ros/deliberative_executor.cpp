#include "deliberative_executor.h"
#include "deliberative_manager.h"
#include "predicate.h"
#include "atom.h"
#include "deliberative_tier/deliberative_state.h"
#include "deliberative_tier/timelines.h"
#include "deliberative_tier/can_start.h"
#include "deliberative_tier/start_task.h"
#include <ros/ros.h>
#include <sstream>

namespace ratio
{
    deliberative_executor::deliberative_executor(deliberative_manager &d_mngr, const uint64_t &id, const std::vector<std::string> &domain_files, const std::vector<std::string> &relevant_predicates) : d_mngr(d_mngr), reasoner_id(id), slv(), exec(slv, relevant_predicates), dcl(*this), dsl(*this), del(*this)
    {
        // we read the domain files..
        ROS_DEBUG("[%lu] Reading domain..", reasoner_id);
        for (const auto &domain_file : domain_files)
        {
            ROS_DEBUG("[%lu] %s", reasoner_id, domain_file.c_str());
        }

        slv.read(domain_files);

        set_state(deliberative_tier::deliberative_state::idle);
    }
    deliberative_executor::~deliberative_executor() {}

    void deliberative_executor::set_state(const unsigned int &st)
    {
        state = st;
        deliberative_tier::deliberative_state state_msg;
        state_msg.reasoner_id = reasoner_id;
        state_msg.deliberative_state = st;
        d_mngr.notify_state.publish(state_msg);
    }

    deliberative_executor::task deliberative_executor::to_task(const ratio::atom &atm)
    {
        uint64_t task_id = atm.get_sigma();
        std::string task_name = atm.get_type().get_name();
        std::vector<std::string> par_names;
        std::vector<std::string> par_values;
        for (const auto &xpr : atm.get_exprs())
            if (xpr.first != START && xpr.first != END && xpr.first != AT && xpr.first != TAU)
            {
                par_names.push_back(xpr.first);
                if (bool_item *bi = dynamic_cast<bool_item *>(&*xpr.second))
                    switch (atm.get_core().bool_value(bi))
                    {
                    case smt::True:
                        par_values.push_back("true");
                        break;
                    case smt::False:
                        par_values.push_back("false");
                        break;
                    default:
                        break;
                    }
                else if (arith_item *ai = dynamic_cast<arith_item *>(&*xpr.second))
                    par_values.push_back(to_string(atm.get_core().arith_value(ai).get_rational()));
                else if (string_item *si = dynamic_cast<string_item *>(&*xpr.second))
                    par_values.push_back(si->get_value());
            }
        return {task_id, task_name, par_names, par_values};
    }

    void deliberative_executor::deliberative_core_listener::started_solving()
    {
        ROS_DEBUG("[%lu] Started reasoning..", exec.reasoner_id);
        exec.set_state(deliberative_tier::deliberative_state::reasoning);
    }
    void deliberative_executor::deliberative_core_listener::solution_found()
    {
        ROS_DEBUG("[%lu] Solution found..", exec.reasoner_id);
        exec.set_state(deliberative_tier::deliberative_state::executing);

        deliberative_tier::timelines timelines_msg;
        timelines_msg.reasoner_id = exec.reasoner_id;
        timelines_msg.update = deliberative_tier::timelines::timelines_changed;
        const auto tls = exec.slv.extract_timelines();
        const smt::array_val &tls_array = static_cast<const smt::array_val &>(*tls);
        for (size_t i = 0; i < tls_array.size(); ++i)
        {
            std::stringstream ss;
            ss << tls_array.get(i);
            timelines_msg.timelines.push_back(ss.str());
        }
        arith_expr origin_expr = exec.slv.get("origin");
        const auto origin = exec.slv.arith_value(origin_expr);
        timelines_msg.origin.num = origin.get_rational().numerator();
        timelines_msg.origin.den = origin.get_rational().denominator();

        arith_expr horizon_expr = exec.slv.get("horizon");
        const auto horizon = exec.slv.arith_value(horizon_expr);
        timelines_msg.horizon.num = horizon.get_rational().numerator();
        timelines_msg.horizon.den = horizon.get_rational().denominator();

        exec.d_mngr.notify_timelines.publish(timelines_msg);
    }
    void deliberative_executor::deliberative_core_listener::inconsistent_problem()
    {
        ROS_DEBUG("[%lu] Inconsistent problem..", exec.reasoner_id);
        exec.set_state(deliberative_tier::deliberative_state::inconsistent);
    }

    void deliberative_executor::deliberative_executor_listener::tick(const smt::rational &time)
    {
        ROS_DEBUG("Current time: %s", to_string(time).c_str());
        deliberative_tier::timelines time_msg;
        time_msg.reasoner_id = exec.reasoner_id;
        time_msg.update = deliberative_tier::timelines::time_changed;
        time_msg.time.num = time.numerator();
        time_msg.time.den = time.denominator();
        exec.d_mngr.notify_timelines.publish(time_msg);

        arith_expr horizon = exec.slv.get("horizon");
        if (exec.slv.arith_value(horizon) <= exec.exec.get_current_time() && exec.current_tasks.empty())
        {
            ROS_DEBUG("[%lu] Exhausted plan..", exec.reasoner_id);
            exec.set_state(deliberative_tier::deliberative_state::finished);
        }
    }

    void deliberative_executor::deliberative_executor_listener::starting(const std::unordered_set<atom *> &atms)
    { // tell the executor the atoms which are not yet ready to start..
        std::unordered_set<ratio::atom *> dsy;
        deliberative_tier::can_start cs_srv;
        task t;
        for (const auto &atm : atms)
        {
            t = to_task(*atm);
            cs_srv.request.task_name = t.task_name;
            cs_srv.request.par_names = t.par_names;
            cs_srv.request.par_values = t.par_values;
            if (exec.d_mngr.can_start.call(cs_srv) && !cs_srv.response.can_start)
                dsy.insert(atm);
        }

        if (!dsy.empty())
            exec.exec.dont_start_yet(atms);
    }
    void deliberative_executor::deliberative_executor_listener::start(const std::unordered_set<atom *> &atms)
    { // these atoms are now started..
        deliberative_tier::start_task st_srv;
        task t;
        for (const auto &atm : atms)
        {
            ROS_DEBUG("[%lu] Starting task %s..", exec.reasoner_id, atm->get_type().get_name().c_str());
            t = to_task(*atm);
            st_srv.request.reasoner_id = exec.reasoner_id;
            st_srv.request.task_id = t.task_id;
            st_srv.request.task_name = t.task_name;
            st_srv.request.par_names = t.par_names;
            st_srv.request.par_values = t.par_values;
            if (exec.d_mngr.start_task.call(st_srv) && st_srv.response.started)
                exec.current_tasks.emplace(atm->get_sigma(), atm);
        }
    }

    void deliberative_executor::deliberative_executor_listener::ending(const std::unordered_set<atom *> &atms)
    { // tell the executor the atoms which are not yet ready to finish..
        std::unordered_set<ratio::atom *> dey;
        for (const auto &atm : atms)
            if (exec.current_tasks.count(atm->get_sigma()))
                dey.insert(atm);

        if (!dey.empty())
            exec.exec.dont_end_yet(atms);
    }
    void deliberative_executor::deliberative_executor_listener::end(const std::unordered_set<atom *> &atms)
    { // these atoms are now ended..
        for (const auto &atm : atms)
        {
            ROS_DEBUG("[%lu] Ended task %s..", exec.reasoner_id, atm->get_type().get_name().c_str());
        }
    }

    void deliberative_executor::finish_task(const smt::var &id, const bool &success)
    {
        ROS_DEBUG("[%lu] Ending task %s..", reasoner_id, current_tasks.at(id)->get_type().get_name().c_str());
        if (!success) // the task failed..
            exec.failure({current_tasks.at(id)});
        current_tasks.erase(id);
    }

    void deliberative_executor::deliberative_solver_listener::flaw_created(const flaw &f)
    {
        exec.flaws.insert(&f);

        deliberative_tier::flaw fc_msg;
        fc_msg.id = reinterpret_cast<std::uintptr_t>(&f);
        for (const auto &r : f.get_causes())
            fc_msg.causes.push_back(reinterpret_cast<std::uintptr_t>(r));
        fc_msg.label = f.get_label();
        fc_msg.state = slv.get_sat_core().value(f.get_phi());
        const auto [lb, ub] = slv.get_idl_theory().bounds(f.get_position());
        fc_msg.position.lb = lb, fc_msg.position.ub = ub;

        deliberative_tier::graph g_msg;
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.flaws.push_back(fc_msg);
        g_msg.update = deliberative_tier::graph::flaw_created;
        exec.d_mngr.notify_graph.publish(g_msg);
    }
    void deliberative_executor::deliberative_solver_listener::flaw_state_changed(const flaw &f)
    {
        deliberative_tier::flaw fsc_msg;
        fsc_msg.id = reinterpret_cast<std::uintptr_t>(&f);
        fsc_msg.state = slv.get_sat_core().value(f.get_phi());

        deliberative_tier::graph g_msg;
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.flaws.push_back(fsc_msg);
        g_msg.update = deliberative_tier::graph::flaw_state_changed;
        exec.d_mngr.notify_graph.publish(g_msg);
    }
    void deliberative_executor::deliberative_solver_listener::flaw_cost_changed(const flaw &f)
    {
        deliberative_tier::flaw fcc_msg;
        fcc_msg.id = reinterpret_cast<std::uintptr_t>(&f);
        const auto est_cost = f.get_estimated_cost();
        fcc_msg.cost.num = est_cost.numerator(), fcc_msg.cost.den = est_cost.denominator();

        deliberative_tier::graph g_msg;
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.flaws.push_back(fcc_msg);
        g_msg.update = deliberative_tier::graph::flaw_cost_changed;
        exec.d_mngr.notify_graph.publish(g_msg);
    }
    void deliberative_executor::deliberative_solver_listener::flaw_position_changed(const flaw &f)
    {
        deliberative_tier::flaw fpc_msg;
        fpc_msg.id = reinterpret_cast<std::uintptr_t>(&f);
        const auto [lb, ub] = slv.get_idl_theory().bounds(f.get_position());
        fpc_msg.position.lb = lb, fpc_msg.position.ub = ub;

        deliberative_tier::graph g_msg;
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.flaws.push_back(fpc_msg);
        g_msg.update = deliberative_tier::graph::flaw_position_changed;
        exec.d_mngr.notify_graph.publish(g_msg);
    }
    void deliberative_executor::deliberative_solver_listener::current_flaw(const flaw &f)
    {
        exec.current_flaw = &f;

        deliberative_tier::graph g_msg;
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.flaw = reinterpret_cast<std::uintptr_t>(&f);
        g_msg.update = deliberative_tier::graph::current_flaw;
        exec.d_mngr.notify_graph.publish(g_msg);
    }

    void deliberative_executor::deliberative_solver_listener::resolver_created(const resolver &r)
    {
        exec.resolvers.insert(&r);

        deliberative_tier::resolver rc_msg;
        rc_msg.id = reinterpret_cast<std::uintptr_t>(&r);
        rc_msg.effect = reinterpret_cast<std::uintptr_t>(&r.get_effect());
        rc_msg.label = r.get_label();
        rc_msg.state = slv.get_sat_core().value(r.get_rho());
        const auto est_cost = r.get_estimated_cost();
        rc_msg.cost.num = est_cost.numerator(), rc_msg.cost.den = est_cost.denominator();

        deliberative_tier::graph g_msg;
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.resolvers.push_back(rc_msg);
        g_msg.update = deliberative_tier::graph::resolver_created;
        exec.d_mngr.notify_graph.publish(g_msg);
    }
    void deliberative_executor::deliberative_solver_listener::resolver_state_changed(const resolver &r)
    {
        deliberative_tier::resolver rsc_msg;
        rsc_msg.id = reinterpret_cast<std::uintptr_t>(&r);
        rsc_msg.state = slv.get_sat_core().value(r.get_rho());

        deliberative_tier::graph g_msg;
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.resolvers.push_back(rsc_msg);
        g_msg.update = deliberative_tier::graph::resolver_state_changed;
        exec.d_mngr.notify_graph.publish(g_msg);
    }
    void deliberative_executor::deliberative_solver_listener::current_resolver(const resolver &r)
    {
        exec.current_resolver = &r;

        deliberative_tier::graph g_msg;
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.resolver = reinterpret_cast<std::uintptr_t>(&r);
        g_msg.update = deliberative_tier::graph::current_resolver;
        exec.d_mngr.notify_graph.publish(g_msg);
    }

    void deliberative_executor::deliberative_solver_listener::causal_link_added(const flaw &f, const resolver &r)
    {
        deliberative_tier::graph g_msg;
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.flaw = reinterpret_cast<std::uintptr_t>(&f);
        g_msg.resolver = reinterpret_cast<std::uintptr_t>(&r);
        g_msg.update = deliberative_tier::graph::causal_link_added;
        exec.d_mngr.notify_graph.publish(g_msg);
    }
} // namespace ratio