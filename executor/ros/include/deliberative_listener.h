#pragma once

#include "solver_listener.h"
#include <ros/ros.h>

namespace ratio
{
  class deliberative_manager;
  class deliberative_executor;

  class deliberative_listener : public solver_listener
  {
  public:
    deliberative_listener(deliberative_manager &d_mngr, deliberative_executor &d_exec);
    ~deliberative_listener();

  private:
    void flaw_created(const flaw &f) override;
    void flaw_state_changed(const flaw &f) override;
    void flaw_cost_changed(const flaw &f) override;
    void flaw_position_changed(const flaw &f) override;
    void current_flaw(const flaw &f) override;

    void resolver_created(const resolver &r) override;
    void resolver_state_changed(const resolver &r) override;
    void current_resolver(const resolver &r) override;

    void causal_link_added(const flaw &f, const resolver &r) override;

  private:
    deliberative_manager &d_mngr;
    deliberative_executor &d_exec;
    ros::Publisher flaw_created_pub;
    ros::Publisher flaw_state_changed_pub;
    ros::Publisher flaw_cost_changed_pub;
    ros::Publisher flaw_position_changed_pub;
    ros::Publisher current_flaw_pub;
    ros::Publisher resolver_created_pub;
    ros::Publisher resolver_state_changed_pub;
    ros::Publisher current_resolver_pub;
    ros::Publisher causal_link_added_pub;
  };
} // namespace ratio
