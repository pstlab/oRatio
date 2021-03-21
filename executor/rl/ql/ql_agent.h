#pragma once

#include <random>
#include <map>

namespace rl
{
  class ql_environment;

  class ql_agent final
  {
  public:
    ql_agent(ql_environment &env);
    ~ql_agent();

    size_t select_best_action() noexcept;
    size_t select_random_action() noexcept;
    size_t select_softmax_action(const double &t = 1) noexcept;

  public:
    void train(const size_t &iterations, const double &gamma = 0.95, const double &alpha = 0.005, const double &eps_decay = 200) noexcept;

    double evaluate(const size_t &init_state, const size_t &max_steps, const size_t &eval_episodes = 10) noexcept;

    friend std::ostream &operator<<(std::ostream &os, const ql_agent &ql);

  private:
    std::default_random_engine gen;
    std::uniform_real_distribution<double> unif = std::uniform_real_distribution<double>(0, 1);
    ql_environment &env;
    std::map<size_t, std::vector<double>> q_table;
    const float eps_start = 0.9, eps_end = 0.05;
    size_t steps_done = 0;
  };
} // namespace rl
