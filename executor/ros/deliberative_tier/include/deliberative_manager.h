#pragma once

#include "msgs/create_reasoner.h"
#include "msgs/destroy_reasoner.h"
#include "msgs/task_finished.h"
#include "msgs/new_requirement.h"
#include <ros/ros.h>
#include <unordered_map>
#include <queue>

namespace sir
{
  class deliberative_executor;

  class deliberative_manager
  {
    friend class deliberative_executor;

  public:
    deliberative_manager(ros::NodeHandle &handle);
    ~deliberative_manager();

    ros::NodeHandle &get_handle() { return handle; }

    void tick();

  private:
    bool create_reasoner(msgs::create_reasoner::Request &req, msgs::create_reasoner::Response &res);
    bool destroy_reasoner(msgs::destroy_reasoner::Request &req, msgs::destroy_reasoner::Response &res);
    bool new_requirement(msgs::new_requirement::Request &req, msgs::new_requirement::Response &res);
    bool task_finished(msgs::task_finished::Request &req, msgs::task_finished::Response &res);

  private:
    ros::NodeHandle &handle;
    ros::ServiceServer create_reasoner_server;
    ros::ServiceServer destroy_reasoner_server;
    ros::ServiceServer new_requirement_server;
    ros::ServiceServer task_finished_server;
    ros::Publisher notify_state;
    ros::ServiceClient can_start;
    ros::ServiceClient start_task;
    std::unordered_map<uint64_t, deliberative_executor *> executors;
    std::unordered_map<uint64_t, std::queue<std::string>> pending_requirements;
  };
} // namespace sir
