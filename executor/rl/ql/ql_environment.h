#pragma once

namespace rl
{
  class ql_environment
  {
  public:
    ql_environment(const size_t &state_dim, const size_t &action_dim, const size_t &init_state = 0) : state_dim(state_dim), action_dim(action_dim), state(init_state) {}
    virtual ~ql_environment() {}

    size_t get_state_dim() const noexcept { return state_dim; }
    size_t get_action_dim() const noexcept { return action_dim; }

    size_t get_state() const noexcept { return state; }
    void set_state(const size_t &c_state) noexcept { state = c_state; }

    struct step
    {
      double reward;
      bool done;
    };
    virtual step execute_action(const size_t &action) noexcept { return {0, true}; }

  protected:
    const size_t state_dim, action_dim;

  private:
    size_t state;
  };
} // namespace rl
