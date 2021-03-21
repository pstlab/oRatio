#pragma once

#include <torch/torch.h>

namespace rl
{
  class td3_environment
  {
  public:
    td3_environment(const size_t &state_dim, const size_t &action_dim, const double &max_action, const torch::Tensor &init_state) : state_dim(state_dim), action_dim(action_dim), max_action(max_action), state(init_state) {}
    virtual ~td3_environment() {}

    size_t get_state_dim() const noexcept { return state_dim; }
    size_t get_action_dim() const noexcept { return action_dim; }
    double get_max_action() const noexcept { return max_action; }

    torch::Tensor get_state() const noexcept { return state; }
    void set_state(const torch::Tensor &c_state) noexcept { state = c_state; }

    struct step
    {
      double reward;
      bool done;
    };
    virtual step execute_action(const torch::Tensor &action) noexcept { return {0, true}; }

  protected:
    const size_t state_dim, action_dim;
    const double max_action;

  private:
    torch::Tensor state;
  };
} // namespace rl
