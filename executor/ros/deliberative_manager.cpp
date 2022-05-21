#include "deliberative_manager.h"
#include "deliberative_executor.h"
#include "deliberative_tier/deliberative_state.h"
#include "deliberative_tier/timelines.h"
#include "deliberative_tier/task_executor.h"

namespace ratio
{
    deliberative_manager::deliberative_manager(ros::NodeHandle &h) : handle(h),
                                                                     create_reasoner_server(h.advertiseService("create_reasoner", &deliberative_manager::create_reasoner, this)),
                                                                     start_execution_server(h.advertiseService("start_execution", &deliberative_manager::start_execution, this)),
                                                                     destroy_reasoner_server(h.advertiseService("destroy_reasoner", &deliberative_manager::destroy_reasoner, this)),
                                                                     new_requirements_server(h.advertiseService("new_requirements", &deliberative_manager::new_requirements, this)),
                                                                     close_task_server(h.advertiseService("close_task", &deliberative_manager::close_task, this)),
                                                                     state_server(h.advertiseService("get_state", &deliberative_manager::get_state, this)),
                                                                     notify_state(handle.advertise<deliberative_tier::deliberative_state>("deliberative_state", 10, true)),
                                                                     notify_graph(handle.advertise<deliberative_tier::graph>("graph", 10)),
                                                                     notify_timelines(handle.advertise<deliberative_tier::timelines>("timelines", 10)),
                                                                     can_start(h.serviceClient<deliberative_tier::task_executor>("can_start")),
                                                                     start_task(h.serviceClient<deliberative_tier::task_executor>("start_task")),
                                                                     can_end(h.serviceClient<deliberative_tier::task_executor>("can_end")),
                                                                     end_task(h.serviceClient<deliberative_tier::task_executor>("end_task"))
    {
        can_start.waitForExistence();
        start_task.waitForExistence();
        can_end.waitForExistence();
        end_task.waitForExistence();
    }
    deliberative_manager::~deliberative_manager() {}

    void deliberative_manager::tick()
    {
        for (auto &exec : executors)
            exec.second->tick();
    }

    bool deliberative_manager::create_reasoner(deliberative_tier::reasoner_creator::Request &req, deliberative_tier::reasoner_creator::Response &res)
    {
        res.reasoner_id = executors.size();
        ROS_DEBUG("Creating new reasoner %lu..", res.reasoner_id);

        executors[res.reasoner_id] = new deliberative_executor(*this, res.reasoner_id, req.domain_files, req.requirements);
        res.consistent = executors[res.reasoner_id]->state != deliberative_tier::deliberative_state::inconsistent;

        deliberative_tier::deliberative_state state_msg;
        state_msg.reasoner_id = res.reasoner_id;
        state_msg.deliberative_state = deliberative_tier::deliberative_state::created;
        notify_state.publish(state_msg);
        return true;
    }

    bool deliberative_manager::start_execution(deliberative_tier::executor::Request &req, deliberative_tier::executor::Response &res)
    {
        ROS_DEBUG("Staarting execution for reasoner %lu..", req.reasoner_id);
        if (const auto exec = executors.find(req.reasoner_id); exec != executors.end())
        {
            exec->second->start_execution(req.notify_start, req.notify_end);
            res.new_state = exec->second->state;
        }
        else
        {
            ROS_WARN("Reasoner %lu does not exists..", req.reasoner_id);
            res.new_state = deliberative_tier::deliberative_state::destroyed;
        }
        return true;
    }

    bool deliberative_manager::destroy_reasoner(deliberative_tier::reasoner_destroyer::Request &req, deliberative_tier::reasoner_destroyer::Response &res)
    {
        ROS_DEBUG("Destroying reasoner %lu..", req.reasoner_id);
        if (const auto exec = executors.find(req.reasoner_id); exec != executors.end())
        {
            delete exec->second;
            executors.erase(exec);
            res.destroyed = true;

            deliberative_tier::deliberative_state state_msg;
            state_msg.reasoner_id = req.reasoner_id;
            state_msg.deliberative_state = deliberative_tier::deliberative_state::destroyed;
            notify_state.publish(state_msg);
        }
        else
        {
            ROS_WARN("Reasoner %lu does not exists..", req.reasoner_id);
            res.destroyed = false;
        }
        return true;
    }

    bool deliberative_manager::new_requirements(deliberative_tier::requirement_creator::Request &req, deliberative_tier::requirement_creator::Response &res)
    {
        ROS_DEBUG("Adding new requirements to reasoner %lu..", req.reasoner_id);
        if (const auto exec = executors.find(req.reasoner_id); exec != executors.end())
        {
            exec->second->append_requirements(req.requirements);
            res.consistent = exec->second->state != deliberative_tier::deliberative_state::inconsistent;
        }
        else
        {
            ROS_WARN("Reasoner %lu does not exist..", req.reasoner_id);
            res.consistent = false;
        }
        return true;
    }

    bool deliberative_manager::close_task(deliberative_tier::task_closer::Request &req, deliberative_tier::task_closer::Response &res)
    {
        ROS_DEBUG("Ending task %lu for reasoner %lu..", req.task.task_id, req.task.reasoner_id);
        if (const auto exec = executors.find(req.task.reasoner_id); exec != executors.end())
        {
            exec->second->close_task(req.task.task_id, req.success);
            res.ended = true;
        }
        else
        {
            ROS_WARN("Reasoner %lu does not exist..", req.task.reasoner_id);
            res.ended = false;
        }
        return true;
    }

    bool deliberative_manager::get_state(deliberative_tier::state_collector::Request &req, deliberative_tier::state_collector::Response &res)
    {
        ROS_DEBUG("Getting Deliberative Manager state");
        for (const auto &exec : executors)
        {
            deliberative_tier::graph g_msg;
            g_msg.reasoner_id = exec.first;
            for (const auto &f : exec.second->flaws)
            {
                deliberative_tier::flaw f_msg;
                f_msg.id = f->get_id();
                for (const auto &r : f->get_causes())
                    f_msg.causes.push_back(r->get_id());
                f_msg.data = f->get_data();
                f_msg.state = f->get_solver().get_sat_core().value(f->get_phi());
                const auto [lb, ub] = f->get_solver().get_idl_theory().bounds(f->get_position());
                f_msg.pos.lb = lb, f_msg.pos.ub = ub;
                const auto est_cost = f->get_estimated_cost();
                f_msg.cost.num = est_cost.numerator(), f_msg.cost.den = est_cost.denominator();
                g_msg.flaws.push_back(f_msg);
            }

            for (const auto &r : exec.second->resolvers)
            {
                deliberative_tier::resolver r_msg;
                r_msg.id = r->get_id();
                for (const auto &p : r->get_preconditions())
                    r_msg.preconditions.push_back(p->get_id());
                r_msg.effect = r->get_effect().get_id();
                r_msg.data = r->get_data();
                r_msg.state = r->get_solver().get_sat_core().value(r->get_rho());
                const auto est_cost = r->get_intrinsic_cost();
                r_msg.intrinsic_cost.num = est_cost.numerator(), r_msg.intrinsic_cost.den = est_cost.denominator();
                g_msg.resolvers.push_back(r_msg);
            }

            if (exec.second->current_flaw)
                g_msg.flaw_id = exec.second->current_flaw->get_id();

            if (exec.second->current_resolver)
                g_msg.resolver_id = exec.second->current_resolver->get_id();

            res.graphs.push_back(g_msg);

            deliberative_tier::timelines tls_msg;
            tls_msg.reasoner_id = exec.first;

            std::stringstream sss;
            sss << exec.second->get_solver().to_json();
            tls_msg.state = sss.str();

            const auto tls = exec.second->get_solver().extract_timelines();
            const smt::array_val &tls_array = static_cast<const smt::array_val &>(*tls);
            for (size_t i = 0; i < tls_array.size(); ++i)
            {
                std::stringstream ss;
                ss << tls_array.get(i);
                tls_msg.timelines.push_back(ss.str());
            }

            const auto c_time = exec.second->get_executor().get_current_time();
            tls_msg.time.num = c_time.numerator();
            tls_msg.time.den = c_time.denominator();

            for (const auto &atm : exec.second->executing)
                tls_msg.executing.push_back(reinterpret_cast<std::uintptr_t>(atm));

            res.timelines.push_back(tls_msg);
        }
        return true;
    }
} // namespace ratio
