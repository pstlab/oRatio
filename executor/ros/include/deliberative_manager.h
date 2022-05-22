#pragma once

#include "deliberative_tier/reasoner_creator.h"
#include "deliberative_tier/executor.h"
#include "deliberative_tier/reasoner_destroyer.h"
#include "deliberative_tier/task_closer.h"
#include "deliberative_tier/requirement_creator.h"
#include "deliberative_tier/state_collector.h"
#include <ros/ros.h>
#include <unordered_map>

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
    bool create_reasoner(deliberative_tier::reasoner_creator::Request &req, deliberative_tier::reasoner_creator::Response &res);
    bool start_execution(deliberative_tier::executor::Request &req, deliberative_tier::executor::Response &res);
    bool destroy_reasoner(deliberative_tier::reasoner_destroyer::Request &req, deliberative_tier::reasoner_destroyer::Response &res);
    bool new_requirements(deliberative_tier::requirement_creator::Request &req, deliberative_tier::requirement_creator::Response &res);
    bool close_task(deliberative_tier::task_closer::Request &req, deliberative_tier::task_closer::Response &res);
    bool get_state(deliberative_tier::state_collector::Request &req, deliberative_tier::state_collector::Response &res);

  private:
    ros::NodeHandle &handle;
    ros::ServiceServer create_reasoner_server;
    ros::ServiceServer start_execution_server;
    ros::ServiceServer destroy_reasoner_server;
    ros::ServiceServer new_requirements_server;
    ros::ServiceServer close_task_server;
    ros::ServiceServer state_server;
    ros::Publisher notify_state;
    ros::Publisher notify_graph;
    ros::Publisher notify_timelines;
    ros::ServiceClient can_start;
    ros::ServiceClient start_task;
    ros::ServiceClient can_end;
    ros::ServiceClient end_task;
    std::unordered_map<uint64_t, deliberative_executor *> executors;
  };
} // namespace ratio
