#pragma once

#include <torch/torch.h>

namespace rl
{
  struct agentImpl : torch::nn::Module
  {
    agentImpl(const size_t &state_dim, const size_t &action_dim) : layer_0(torch::nn::Linear(state_dim, 400)), layer_1(torch::nn::Linear(400, 300)), layer_2(torch::nn::Linear(300, action_dim))
    {
      register_module("layer_0", layer_0);
      register_module("layer_1", layer_1);
      register_module("layer_2", layer_2);
    }

    torch::Tensor forward(torch::Tensor x)
    {
      x = torch::relu(layer_0(x));
      x = torch::relu(layer_1(x));
      x = layer_2(x);
      return x;
    }

  private:
    torch::nn::Linear layer_0, layer_1, layer_2;
  };
  TORCH_MODULE(agent);

  class dql_agent final
  {
  public:
    dql_agent(const size_t &state_dim, const size_t &action_dim, const torch::Tensor &init_state);
    ~dql_agent();

    size_t select_action();
    virtual std::pair<std::vector<double>, double> execute_action(const size_t &action) noexcept { return {std::vector<double>(state_dim, 0), 0}; }

    void train(const size_t &iterations, const size_t &batch_size = 100, const double &discount = 0.99, const double &alpha = 0.005, const size_t &policy_freq = 2);

    void save() const;
    void load();

  private:
    class transition final
    {
    public:
      transition(const std::vector<double> &state, const std::vector<double> &next_state, const long &action, const double &reward) : state(state), next_state(next_state), action(action), reward(reward) {}
      ~transition() {}

      std::vector<double> state;
      std::vector<double> next_state;
      long action;
      double reward;
    };

    class transition_batch final
    {
    public:
      transition_batch(const std::vector<std::vector<double>> &states, const std::vector<std::vector<double>> &next_states, const std::vector<long> &actions, const std::vector<double> &rewards) : states(states), next_states(next_states), actions(actions), rewards(rewards) {}
      ~transition_batch() {}

      std::vector<std::vector<double>> states;
      std::vector<std::vector<double>> next_states;
      std::vector<long> actions;
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

  private:
    const size_t state_dim, action_dim;
    torch::Device device;
    torch::Tensor state;
    double eps = 1;
    agent policy;
    torch::optim::Adam policy_optimizer;
    agent target;
    reply_buffer buffer;
  };
} // namespace rl
