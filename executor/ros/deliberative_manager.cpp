#include "deliberative_manager.h"
#include "deliberative_executor.h"
#include "deliberative_tier/deliberative_state.h"
#include "deliberative_tier/timelines.h"
#include "deliberative_tier/can_start.h"
#include "deliberative_tier/start_task.h"
#include "std_msgs/UInt64.h"

namespace ratio
{
    deliberative_manager::deliberative_manager(ros::NodeHandle &h) : handle(h),
                                                                     create_reasoner_server(h.advertiseService("create_reasoner", &deliberative_manager::create_reasoner, this)),
                                                                     destroy_reasoner_server(h.advertiseService("destroy_reasoner", &deliberative_manager::destroy_reasoner, this)),
                                                                     new_requirement_server(h.advertiseService("new_requirement", &deliberative_manager::new_requirement, this)),
                                                                     task_finished_server(h.advertiseService("task_finished", &deliberative_manager::task_finished, this)),
                                                                     graph_server(h.advertiseService("get_graph", &deliberative_manager::get_graph, this)),
                                                                     timelines_server(h.advertiseService("get_timelines", &deliberative_manager::get_timelines, this)),
                                                                     reasoner_created(handle.advertise<std_msgs::UInt64>("reasoner_created", 10, true)),
                                                                     reasoner_destroyed(handle.advertise<std_msgs::UInt64>("reasoner_destroyed", 10, true)),
                                                                     notify_state(handle.advertise<deliberative_tier::deliberative_state>("deliberative_state", 10, true)),
                                                                     notify_graph(handle.advertise<deliberative_tier::graph>("graph", 10, true)),
                                                                     notify_timelines(handle.advertise<deliberative_tier::timelines>("timelines", 10, true)),
                                                                     can_start(h.serviceClient<deliberative_tier::can_start>("can_start")),
                                                                     start_task(h.serviceClient<deliberative_tier::start_task>("start_task"))
    {
        can_start.waitForExistence();
        start_task.waitForExistence();
    }
    deliberative_manager::~deliberative_manager() {}

    void deliberative_manager::tick()
    {
        if (!pending_requirements.empty())
        {
            ROS_DEBUG("Disposing pending requirements..");
            for (auto &req : pending_requirements)
            {
                while (!executors.at(req.first)->get_solver().root_level()) // we go at root level..
                    executors.at(req.first)->get_solver().get_sat_core().pop();
                while (!req.second.empty())
                { // we read the pending requirement..
                    const std::string c_req = req.second.front();
                    ROS_DEBUG("[%lu] %s", req.first, c_req.c_str());
                    executors.at(req.first)->get_solver().read(c_req);
                    req.second.pop();
                }
                // we solve the problem again..
                executors.at(req.first)->get_solver().solve();
            }
            pending_requirements.clear();
        }
        for (auto &exec : executors)
            exec.second->get_executor().tick();
    }

    bool deliberative_manager::create_reasoner(deliberative_tier::create_reasoner::Request &req, deliberative_tier::create_reasoner::Response &res)
    {
        ROS_DEBUG("Creating new reasoner %lu..", req.reasoner_id);
        if (executors.find(req.reasoner_id) != executors.end())
        {
            ROS_WARN("Reasoner %lu already exists..", req.reasoner_id);
            res.created = false;
        }
        else
        {
            std::vector<std::string> relevant_predicates;
            relevant_predicates.insert(relevant_predicates.end(), req.notify_start.begin(), req.notify_start.end());

            executors[req.reasoner_id] = new deliberative_executor(*this, req.reasoner_id, req.domain_files, relevant_predicates);

            for (const auto &r : req.requirements)
                pending_requirements[req.reasoner_id].push(r);

            res.created = true;

            std_msgs::UInt64 r_created_msg;
            r_created_msg.data = req.reasoner_id;
            reasoner_created.publish(r_created_msg);
        }
        return true;
    }

    bool deliberative_manager::destroy_reasoner(deliberative_tier::destroy_reasoner::Request &req, deliberative_tier::destroy_reasoner::Response &res)
    {
        ROS_DEBUG("Destroying reasoner %lu..", req.reasoner_id);
        if (executors.find(req.reasoner_id) == executors.end())
        {
            ROS_WARN("Reasoner %lu does not exists..", req.reasoner_id);
            res.destroyed = false;
        }
        else
        {
            delete executors.at(req.reasoner_id);
            executors.erase(req.reasoner_id);
            res.destroyed = true;

            std_msgs::UInt64 r_destroyed_msg;
            r_destroyed_msg.data = req.reasoner_id;
            reasoner_destroyed.publish(r_destroyed_msg);
        }
        return true;
    }

    bool deliberative_manager::new_requirement(deliberative_tier::new_requirement::Request &req, deliberative_tier::new_requirement::Response &res)
    {
        ROS_DEBUG("Adding new requirement to reasoner %lu..", req.reasoner_id);
        if (executors.find(req.reasoner_id) == executors.end())
        {
            ROS_WARN("Reasoner %lu does not exist..", req.reasoner_id);
            res.consistent = false;
        }
        else
        {
            pending_requirements[req.reasoner_id].push(req.requirement);
            res.consistent = true;
        }
        return true;
    }

    bool deliberative_manager::task_finished(deliberative_tier::task_finished::Request &req, deliberative_tier::task_finished::Response &res)
    {
        ROS_DEBUG("Ending task %lu for reasoner %lu..", req.task_id, req.reasoner_id);
        if (executors.find(req.reasoner_id) == executors.end())
        {
            ROS_WARN("Reasoner %lu does not exist..", req.reasoner_id);
            res.ended = false;
        }
        else
        {
            executors.at(req.reasoner_id)->finish_task(req.task_id, req.success);
            res.ended = true;
        }
        return true;
    }

    bool deliberative_manager::get_graph(deliberative_tier::get_graph::Request &req, deliberative_tier::get_graph::Response &res)
    {
        ROS_DEBUG("Getting causal graph for reasoner %lu..", req.reasoner_id);
        if (executors.find(req.reasoner_id) == executors.end())
        {
            ROS_WARN("Reasoner %lu does not exist..", req.reasoner_id);
        }
        else
        {
            for (const auto &f : executors.at(req.reasoner_id)->flaws)
            {
                deliberative_tier::flaw f_msg;
                f_msg.id = reinterpret_cast<std::uintptr_t>(&f);
                for (const auto &r : f->get_causes())
                    f_msg.causes.push_back(reinterpret_cast<std::uintptr_t>(r));
                f_msg.data = f->get_data();
                f_msg.state = f->get_solver().get_sat_core().value(f->get_phi());
                const auto [lb, ub] = f->get_solver().get_idl_theory().bounds(f->get_position());
                const auto est_cost = f->get_estimated_cost();
                f_msg.cost.num = est_cost.numerator(), f_msg.cost.den = est_cost.denominator();
                f_msg.position.lb = lb, f_msg.position.ub = ub;
                res.graph.flaws.push_back(f_msg);
            }

            for (const auto &r : executors.at(req.reasoner_id)->resolvers)
            {
                deliberative_tier::resolver r_msg;
                r_msg.id = reinterpret_cast<std::uintptr_t>(&r);
                r_msg.effect = reinterpret_cast<std::uintptr_t>(&r->get_effect());
                r_msg.data = r->get_data();
                r_msg.state = r->get_solver().get_sat_core().value(r->get_rho());
                const auto est_cost = r->get_estimated_cost();
                r_msg.cost.num = est_cost.numerator(), r_msg.cost.den = est_cost.denominator();
                res.graph.resolvers.push_back(r_msg);
            }

            if (executors.at(req.reasoner_id)->current_flaw)
                res.graph.flaw = reinterpret_cast<std::uintptr_t>(executors.at(req.reasoner_id)->current_flaw);

            if (executors.at(req.reasoner_id)->current_resolver)
                res.graph.resolver = reinterpret_cast<std::uintptr_t>(executors.at(req.reasoner_id)->current_resolver);
        }
        return true;
    }
    bool deliberative_manager::get_timelines(deliberative_tier::get_timelines::Request &req, deliberative_tier::get_timelines::Response &res)
    {
        ROS_DEBUG("Getting timelines for reasoner %lu..", req.reasoner_id);
        if (executors.find(req.reasoner_id) == executors.end())
        {
            ROS_WARN("Reasoner %lu does not exist..", req.reasoner_id);
        }
        else
        {
            const auto tls = executors.at(req.reasoner_id)->get_solver().extract_timelines();
            const smt::array_val &tls_array = static_cast<const smt::array_val &>(*tls);
            for (size_t i = 0; i < tls_array.size(); ++i)
            {
                std::stringstream ss;
                ss << tls_array.get(i);
                res.timelines.timelines.push_back(ss.str());
            }
            arith_expr origin_expr = executors.at(req.reasoner_id)->get_solver().get("origin");
            const auto origin = executors.at(req.reasoner_id)->get_solver().arith_value(origin_expr);
            res.timelines.origin.num = origin.get_rational().numerator();
            res.timelines.origin.den = origin.get_rational().denominator();

            arith_expr horizon_expr = executors.at(req.reasoner_id)->get_solver().get("horizon");
            const auto horizon = executors.at(req.reasoner_id)->get_solver().arith_value(horizon_expr);
            res.timelines.horizon.num = horizon.get_rational().numerator();
            res.timelines.horizon.den = horizon.get_rational().denominator();

            const auto c_time = executors.at(req.reasoner_id)->get_executor().get_current_time();
            res.timelines.time.num = c_time.numerator();
            res.timelines.time.den = c_time.denominator();
        }
        return true;
    }
} // namespace ratio
