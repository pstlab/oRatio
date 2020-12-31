#pragma once

#include <random>
#include <vector>

namespace drl
{
  class rl_agent
  {
  public:
    rl_agent(const size_t &state_dim, const size_t &action_dim, const size_t &c_state);
    ~rl_agent();

    size_t select_action() noexcept;
    virtual std::pair<size_t, double> execute_action(const size_t &action) noexcept { return {0, 0}; }

    void train(const size_t &iterations, const double &discount = 0.99, const double &alpha = 0.005) noexcept;

  private:
    std::default_random_engine gen;
    double eps = 1;
    size_t c_state;
    std::vector<std::vector<double>> q_table;
  };
} // namespace drl
