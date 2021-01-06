#pragma once

#include <random>
#include <vector>
#include <tuple>
#include <iostream>

namespace rl
{
  class ql_agent
  {
  public:
    ql_agent(const size_t &state_dim, const size_t &action_dim, const size_t &init_state);
    ~ql_agent();

    size_t get_state() const noexcept { return state; }
    void set_state(const size_t &c_state) noexcept { state = c_state; }
    size_t get_state_dim() const noexcept { return state_dim; }
    size_t get_action_dim() const noexcept { return action_dim; }

    double evaluate(const size_t &init_state, const size_t &max_steps, const size_t &eval_episodes = 10) noexcept;

    size_t select_action(const bool &count_step = true) noexcept;

  private:
    size_t select_best_action();
    size_t select_random_action();
    size_t select_softmax_action(const double &t = 1);

  public:
    virtual std::tuple<size_t, double, bool> execute_action(const size_t &action) noexcept { return {0, 0, true}; }

    void train(const size_t &iterations, const double &gamma = 0.95, const double &alpha = 0.005, const double &eps_decay = 0.001) noexcept;

    friend std::ostream &operator<<(std::ostream &os, const ql_agent &ql);

  protected:
    const size_t state_dim, action_dim;
    std::default_random_engine gen;
    std::uniform_real_distribution<double> unif = std::uniform_real_distribution<double>(0, 1);

  private:
    const float eps_start = 0.9, eps_end = 0.05, eps_decay = 200;
    size_t steps_done = 0;
    size_t state;
    std::vector<std::vector<double>> q_table;
  };
} // namespace rl
