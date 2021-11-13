#include "deliberative_manager.h"
#include "deliberative_executor.h"
#include "msgs/deliberative_state.h"
#include "msgs/can_start.h"
#include "msgs/start_task.h"

namespace sir
{
    deliberative_manager::deliberative_manager(ros::NodeHandle &h) : handle(h),
                                                                     create_reasoner_server(h.advertiseService("create_reasoner", &deliberative_manager::create_reasoner, this)),
                                                                     destroy_reasoner_server(h.advertiseService("destroy_reasoner", &deliberative_manager::destroy_reasoner, this)),
                                                                     new_requirement_server(h.advertiseService("new_requirement", &deliberative_manager::new_requirement, this)),
                                                                     task_finished_server(h.advertiseService("task_finished", &deliberative_manager::task_finished, this)),
                                                                     notify_state(handle.advertise<msgs::deliberative_state>("deliberative_state", 10, true)),
                                                                     can_start(h.serviceClient<msgs::can_start>("can_start")),
                                                                     start_task(h.serviceClient<msgs::start_task>("start_task"))
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
                while (!req.second.empty())
                {
                    const std::string c_req = req.second.front();
                    ROS_DEBUG("[%lu] %s", req.first, c_req.c_str());
                    executors.at(req.first)->get_solver().read(c_req);
                    executors.at(req.first)->get_solver().solve();
                    req.second.pop();
                }
            pending_requirements.clear();
        }
        for (auto &exec : executors)
            exec.second->get_executor().tick();
    }

    bool deliberative_manager::create_reasoner(msgs::create_reasoner::Request &req, msgs::create_reasoner::Response &res)
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
        }
        return true;
    }

    bool deliberative_manager::destroy_reasoner(msgs::destroy_reasoner::Request &req, msgs::destroy_reasoner::Response &res)
    {
        ROS_DEBUG("Destroying reasoner %lu..", req.reasoner_id);
        if (executors.find(req.reasoner_id) == executors.end())
        {
            ROS_WARN("Reasoner %lu does not exists..", req.reasoner_id);
            res.destroyed = false;
        }
        else
        {
            executors.erase(req.reasoner_id);
            res.destroyed = true;
        }
        return true;
    }

    bool deliberative_manager::new_requirement(msgs::new_requirement::Request &req, msgs::new_requirement::Response &res)
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

    bool deliberative_manager::task_finished(msgs::task_finished::Request &req, msgs::task_finished::Response &res)
    {
        ROS_DEBUG("Ending task %lu for reasoner %lu..", req.reasoner_id, req.task_id);
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
} // namespace sir
