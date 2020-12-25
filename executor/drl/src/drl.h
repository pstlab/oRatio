#pragma once

#include <vector>
#include <torch/torch.h>

namespace drl
{
  class transition final
  {
  public:
    transition(const std::vector<double> &state, const std::vector<double> &next_state, const std::vector<double> &action, const double &reward) : state(state), next_state(next_state), action(action), reward(reward) {}
    ~transition() {}

    std::vector<double> state;
    std::vector<double> next_state;
    std::vector<double> action;
    double reward;
  };

  class transition_batch final
  {
  public:
    transition_batch(std::vector<std::vector<double>> states, std::vector<std::vector<double>> next_states, std::vector<std::vector<double>> actions, std::vector<double> rewards) : states(states), next_states(next_states), actions(actions), rewards(rewards) {}
    ~transition_batch() {}

    std::vector<std::vector<double>> states;
    std::vector<std::vector<double>> next_states;
    std::vector<std::vector<double>> actions;
    std::vector<double> rewards;
  };

  class reply_buffer final
  {
  public:
    reply_buffer(const size_t &size = 1e6);
    ~reply_buffer();

    void add(const transition &tr);

    transition_batch sample(const size_t &batch_size) const;

  private:
    const size_t size;
    size_t ptr = 0;
    std::vector<transition> storage;
  };
} // namespace drl
