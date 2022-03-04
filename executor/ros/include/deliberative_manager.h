#pragma once

#include "deliberative_tier/create_reasoner.h"
#include "deliberative_tier/destroy_reasoner.h"
#include "deliberative_tier/task_finished.h"
#include "deliberative_tier/new_requirement.h"
#include "deliberative_tier/get_state.h"
#include <ros/ros.h>
#include <unordered_map>
#include <queue>

namespace ratio
{
  class deliberative_executor;
  class deliberative_executor_listener;
  class deliberative_solver_listener;

  class deliberative_manager
  {
    friend class deliberative_executor;
    friend class deliberative_solver_listener;

  public:
    deliberative_manager(ros::NodeHandle &handle);
    ~deliberative_manager();

    ros::NodeHandle &get_handle() { return handle; }

    void tick();

  private:
    bool create_reasoner(deliberative_tier::create_reasoner::Request &req, deliberative_tier::create_reasoner::Response &res);
    bool destroy_reasoner(deliberative_tier::destroy_reasoner::Request &req, deliberative_tier::destroy_reasoner::Response &res);
    bool new_requirement(deliberative_tier::new_requirement::Request &req, deliberative_tier::new_requirement::Response &res);
    bool task_finished(deliberative_tier::task_finished::Request &req, deliberative_tier::task_finished::Response &res);
    bool get_state(deliberative_tier::get_state::Request &req, deliberative_tier::get_state::Response &res);

  private:
    ros::NodeHandle &handle;
    ros::ServiceServer create_reasoner_server;
    ros::ServiceServer destroy_reasoner_server;
    ros::ServiceServer new_requirement_server;
    ros::ServiceServer task_finished_server;
    ros::ServiceServer state_server;
    ros::Publisher reasoner_created;
    ros::Publisher reasoner_destroyed;
    ros::Publisher notify_state;
    ros::Publisher notify_graph;
    ros::Publisher notify_timelines;
    ros::ServiceClient can_start;
    ros::ServiceClient start_task;
    ros::ServiceClient can_end;
    ros::ServiceClient end_task;
    std::unordered_map<uint64_t, deliberative_executor *> executors;
    std::unordered_map<uint64_t, std::queue<std::string>> pending_requirements;
  };
} // namespace ratio
