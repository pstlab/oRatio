#include "deliberative_executor.h"
#include "deliberative_manager.h"
#include "predicate.h"
#include "atom.h"
#include "deliberative_tier/deliberative_state.h"
#include "deliberative_tier/timelines.h"
#include "deliberative_tier/task_executor.h"
#include <ros/ros.h>
#include <sstream>

namespace ratio
{
    deliberative_executor::deliberative_executor(deliberative_manager &d_mngr, const uint64_t &id, const std::vector<std::string> &domain_files, const std::vector<std::string> &requirements) : d_mngr(d_mngr), reasoner_id(id), slv(), exec(slv), dcl(*this), dsl(*this), del(*this)
    {
        // a new reasoner has just been created..
        set_state(deliberative_tier::deliberative_state::created);

        try
        {
            // we read the domain files..
            ROS_DEBUG("[%lu] Reading domain..", reasoner_id);
            for (const auto &domain_file : domain_files)
            {
                ROS_DEBUG("[%lu] %s", reasoner_id, domain_file.c_str());
            }
            slv.read(domain_files);

            // we read the requirements..
            ROS_DEBUG("[%lu] Reading requirements..", reasoner_id);
            for (const auto &requirement : requirements)
            {
                ROS_DEBUG("[%lu] %s", reasoner_id, requirement.c_str());
                slv.read(requirement);
            }

            pending_requirements = true;
        }
        catch (const inconsistency_exception &e)
        { // the problem is inconsistent..
            ROS_DEBUG("[%lu] The problem is inconsistent..", reasoner_id);
            set_state(deliberative_tier::deliberative_state::inconsistent);
        }
        catch (const unsolvable_exception &e)
        { // the problem is unsolvable..
            ROS_DEBUG("[%lu] The problem is unsolvable..", reasoner_id);
            set_state(deliberative_tier::deliberative_state::inconsistent);
        }
    }

    void deliberative_executor::start_execution(const std::vector<std::string> &notify_start_ids, const std::vector<std::string> &notify_end_ids)
    {
        ROS_DEBUG("[%lu] Starting execution..", reasoner_id);

        notify_start.clear();
        for (const auto &pred : notify_start_ids)
            notify_start.insert(&get_predicate(pred));
        notify_end.clear();
        for (const auto &pred : notify_end_ids)
            notify_end.insert(&get_predicate(pred));

        restart_execution = true;
        set_state(deliberative_tier::deliberative_state::executing);
    }
    void deliberative_executor::tick()
    {
        if (pending_requirements)
        { // we solve the problem again..
            slv.solve();
            pending_requirements = false;
        }
        if (state == deliberative_tier::deliberative_state::executing)
            exec.tick();
    }
    void deliberative_executor::append_requirements(const std::vector<std::string> &requirements)
    {
        try
        {
            ROS_DEBUG("[%lu] Going at root level..", reasoner_id);
            while (!slv.root_level()) // we go at root level..
                slv.get_sat_core().pop();
            ROS_DEBUG("[%lu] Reading requirements..", reasoner_id);
            for (const auto &requirement : requirements)
            {
                ROS_DEBUG("[%lu] %s", reasoner_id, requirement.c_str());
                slv.read(requirement);
            }

            pending_requirements = true;
        }
        catch (const inconsistency_exception &e)
        { // the problem is inconsistent..
            ROS_DEBUG("[%lu] The problem is inconsistent..", reasoner_id);
            set_state(deliberative_tier::deliberative_state::inconsistent);
        }
        catch (const unsolvable_exception &e)
        { // the problem is unsolvable..
            ROS_DEBUG("[%lu] The problem is unsolvable..", reasoner_id);
            set_state(deliberative_tier::deliberative_state::inconsistent);
        }
    }
    void deliberative_executor::deliberative_executor_listener::tick(const smt::rational &time)
    {
        ROS_DEBUG("Current time: %s", to_string(time).c_str());
        exec.current_time = time;

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

    void deliberative_executor::set_state(const unsigned int &st)
    {
        state = st;
        deliberative_tier::deliberative_state state_msg;
        state_msg.reasoner_id = reasoner_id;
        state_msg.deliberative_state = st;
        d_mngr.notify_state.publish(state_msg);
    }

    predicate &deliberative_executor::get_predicate(const std::string &pred) const
    {
        std::vector<std::string> ids;
        size_t start = 0, end = 0;
        do
        {
            end = pred.find('.', start);
            if (end == std::string::npos)
                end = pred.length();
            std::string token = pred.substr(start, end - start);
            if (!token.empty())
                ids.push_back(token);
            start = end + 1;
        } while (end < pred.length() && start < pred.length());

        if (ids.size() == 1)
            return slv.get_predicate(ids[0]);
        else
        {
            type *tp = &slv.get_type(ids[0]);
            for (size_t i = 1; i < ids.size(); ++i)
                if (i == ids.size() - 1)
                    return tp->get_predicate(ids[i]);
                else
                    tp = &tp->get_type(ids[i]);
        }
        // not found
        throw std::out_of_range(pred);
    }

    deliberative_executor::task deliberative_executor::to_task(const ratio::atom &atm)
    {
        uint64_t task_id = atm.get_id();
        std::string task_name = atm.get_type().get_name();
        std::vector<std::string> par_names;
        std::vector<std::string> par_values;
        for (const auto &xpr : atm.get_exprs())
            if (xpr.first != RATIO_START && xpr.first != RATIO_END && xpr.first != RATIO_AT && xpr.first != TAU)
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
        exec.current_flaw = nullptr;
        exec.current_resolver = nullptr;

        deliberative_tier::timelines timelines_msg;
        timelines_msg.reasoner_id = exec.reasoner_id;
        timelines_msg.update = deliberative_tier::timelines::state_changed;

        std::stringstream sss;
        sss << exec.slv.to_json();
        timelines_msg.state = sss.str();

        const auto tls = exec.slv.extract_timelines();
        const smt::array_val &tls_array = static_cast<const smt::array_val &>(*tls);
        for (size_t i = 0; i < tls_array.size(); ++i)
        {
            std::stringstream ss;
            ss << tls_array.get(i);
            timelines_msg.timelines.push_back(ss.str());
        }

        timelines_msg.time.num = exec.current_time.numerator();
        timelines_msg.time.den = exec.current_time.denominator();

        for (const auto &atm : exec.executing)
            timelines_msg.executing.push_back(atm->get_id());

        exec.d_mngr.notify_timelines.publish(timelines_msg);

        exec.set_state(exec.restart_execution ? deliberative_tier::deliberative_state::executing : deliberative_tier::deliberative_state::idle);
    }
    void deliberative_executor::deliberative_core_listener::inconsistent_problem()
    {
        ROS_DEBUG("[%lu] Inconsistent problem..", exec.reasoner_id);

        exec.current_flaw = nullptr;
        exec.current_resolver = nullptr;

        exec.set_state(deliberative_tier::deliberative_state::inconsistent);
    }

    void deliberative_executor::deliberative_executor_listener::starting(const std::unordered_set<atom *> &atms)
    { // tell the executor the atoms which are not yet ready to start..
        std::unordered_map<const ratio::atom *, rational> dsy;
        deliberative_tier::task_executor cs_srv;
        task t;
        for (const auto &atm : atms)
            if (exec.notify_start.count(static_cast<predicate *>(&atm->get_type())))
            {
                t = to_task(*atm);
                cs_srv.request.task.task_name = t.task_name;
                cs_srv.request.task.par_names = t.par_names;
                cs_srv.request.task.par_values = t.par_values;
                if (exec.d_mngr.can_start.call(cs_srv) && !cs_srv.response.success)
                    dsy[atm] = cs_srv.response.delay.den == 0 ? smt::rational(1) : smt::rational(cs_srv.response.delay.num, cs_srv.response.delay.den);
            }

        if (!dsy.empty())
            exec.exec.dont_start_yet(dsy);
    }
    void deliberative_executor::deliberative_executor_listener::start(const std::unordered_set<atom *> &atms)
    { // these atoms are now started..
        deliberative_tier::task_executor st_srv;
        task t;
        for (const auto &atm : atms)
            if (exec.notify_start.count(static_cast<predicate *>(&atm->get_type())))
            {
                exec.executing.insert(atm);
                ROS_DEBUG("[%lu] Starting task %s..", exec.reasoner_id, atm->get_type().get_name().c_str());
                t = to_task(*atm);
                st_srv.request.task.reasoner_id = exec.reasoner_id;
                st_srv.request.task.task_id = t.task_id;
                st_srv.request.task.task_name = t.task_name;
                st_srv.request.task.par_names = t.par_names;
                st_srv.request.task.par_values = t.par_values;
                if (exec.d_mngr.start_task.call(st_srv) && st_srv.response.success)
                    exec.current_tasks.insert(atm);
            }
            else if (exec.notify_end.count(static_cast<predicate *>(&atm->get_type())))
                exec.current_tasks.insert(atm);

        deliberative_tier::timelines executing_msg;
        executing_msg.reasoner_id = exec.reasoner_id;
        executing_msg.update = deliberative_tier::timelines::executing_changed;
        for (const auto &atm : exec.executing)
            executing_msg.executing.push_back(reinterpret_cast<std::uintptr_t>(atm));
        exec.d_mngr.notify_timelines.publish(executing_msg);
    }

    void deliberative_executor::deliberative_executor_listener::ending(const std::unordered_set<atom *> &atms)
    { // tell the executor the atoms which are not yet ready to finish..
        std::unordered_map<const ratio::atom *, rational> dey;
        deliberative_tier::task_executor ce_srv;
        task t;
        for (const auto &atm : atms)
            if (exec.notify_end.count(static_cast<predicate *>(&atm->get_type())))
            {
                t = to_task(*atm);
                ce_srv.request.task.task_name = t.task_name;
                ce_srv.request.task.par_names = t.par_names;
                ce_srv.request.task.par_values = t.par_values;
                if (exec.d_mngr.can_end.call(ce_srv) && !ce_srv.response.success)
                    dey[atm] = ce_srv.response.delay.den == 0 ? smt::rational(1) : smt::rational(ce_srv.response.delay.num, ce_srv.response.delay.den);
            }
            else if (exec.current_tasks.count(atm->get_id()))
                dey[atm] = smt::rational(1);

        if (!dey.empty())
            exec.exec.dont_end_yet(dey);
    }
    void deliberative_executor::deliberative_executor_listener::end(const std::unordered_set<atom *> &atms)
    { // these atoms are now ended..
        for (const auto &atm : atms)
        {
            exec.executing.erase(atm);
            ROS_DEBUG("[%lu] Ended task %s..", exec.reasoner_id, atm->get_type().get_name().c_str());
        }

        deliberative_tier::timelines executing_msg;
        executing_msg.reasoner_id = exec.reasoner_id;
        executing_msg.update = deliberative_tier::timelines::executing_changed;
        for (const auto &atm : exec.executing)
            executing_msg.executing.push_back(reinterpret_cast<std::uintptr_t>(atm));
        exec.d_mngr.notify_timelines.publish(executing_msg);
    }

    void deliberative_executor::close_task(const smt::var &id, const bool &success)
    {
        ROS_DEBUG("[%lu] Closing task %s..", reasoner_id, current_tasks.at(id)->get_type().get_name().c_str());
        if (!success) // the task failed..
            exec.failure({current_tasks.at(id)});
        current_tasks.erase(id);
    }

    void deliberative_executor::deliberative_solver_listener::flaw_created(const flaw &f)
    {
        exec.flaws.insert(&f);

        deliberative_tier::flaw fc_msg;
        fc_msg.id = f.get_id();
        for (const auto &r : f.get_causes())
            fc_msg.causes.push_back(r->get_id());
        fc_msg.data = f.get_data();
        fc_msg.state = slv.get_sat_core().value(f.get_phi());
        const auto [lb, ub] = slv.get_idl_theory().bounds(f.get_position());
        fc_msg.pos.lb = lb, fc_msg.pos.ub = ub;

        deliberative_tier::graph g_msg;
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.flaws.push_back(fc_msg);
        g_msg.update = deliberative_tier::graph::flaw_created;
        exec.d_mngr.notify_graph.publish(g_msg);
    }
    void deliberative_executor::deliberative_solver_listener::flaw_state_changed(const flaw &f)
    {
        deliberative_tier::flaw fsc_msg;
        fsc_msg.id = f.get_id();
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
        fcc_msg.id = f.get_id();
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
        fpc_msg.id = f.get_id();
        const auto [lb, ub] = slv.get_idl_theory().bounds(f.get_position());
        fpc_msg.pos.lb = lb, fpc_msg.pos.ub = ub;

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
        g_msg.flaw_id = f.get_id();
        g_msg.update = deliberative_tier::graph::current_flaw;
        exec.d_mngr.notify_graph.publish(g_msg);
    }

    void deliberative_executor::deliberative_solver_listener::resolver_created(const resolver &r)
    {
        exec.resolvers.insert(&r);

        deliberative_tier::resolver rc_msg;
        rc_msg.id = r.get_id();
        for (const auto &p : r.get_preconditions())
            rc_msg.preconditions.push_back(p->get_id());
        rc_msg.effect = r.get_effect().get_id();
        rc_msg.data = r.get_data();
        rc_msg.state = slv.get_sat_core().value(r.get_rho());
        const auto est_cost = r.get_intrinsic_cost();
        rc_msg.intrinsic_cost.num = est_cost.numerator(), rc_msg.intrinsic_cost.den = est_cost.denominator();

        deliberative_tier::graph g_msg;
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.resolvers.push_back(rc_msg);
        g_msg.update = deliberative_tier::graph::resolver_created;
        exec.d_mngr.notify_graph.publish(g_msg);
    }
    void deliberative_executor::deliberative_solver_listener::resolver_state_changed(const resolver &r)
    {
        deliberative_tier::resolver rsc_msg;
        rsc_msg.id = r.get_id();
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
        g_msg.resolver_id = r.get_id();
        g_msg.update = deliberative_tier::graph::current_resolver;
        exec.d_mngr.notify_graph.publish(g_msg);
    }

    void deliberative_executor::deliberative_solver_listener::causal_link_added(const flaw &f, const resolver &r)
    {
        deliberative_tier::graph g_msg;
        g_msg.reasoner_id = exec.get_reasoner_id();
        g_msg.flaw_id = f.get_id();
        g_msg.resolver_id = r.get_id();
        g_msg.update = deliberative_tier::graph::causal_link_added;
        exec.d_mngr.notify_graph.publish(g_msg);
    }
} // namespace ratio