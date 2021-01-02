#pragma once

#include <torch/torch.h>
#include <tuple>
#include <random>

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

  class dql_agent
  {
    class reply_buffer;

  public:
    dql_agent(const size_t &state_dim, const size_t &action_dim, const torch::Tensor &init_state);
    ~dql_agent();

    torch::Tensor get_state() const noexcept { return state; }
    void set_state(const torch::Tensor &c_state) noexcept { state = c_state; }
    size_t get_state_dim() const noexcept { return state_dim; }
    size_t get_action_dim() const noexcept { return action_dim; }

    reply_buffer &get_buffer() noexcept { return buffer; }

    size_t select_action();
    virtual std::tuple<torch::Tensor, double, bool> execute_action(const size_t &action) noexcept { return {torch::tensor(std::vector<double>(state_dim, 0)), 0, true}; }

    void train(const size_t &iterations, const size_t &batch_size = 100, const double &discount = 0.99, const double &alpha = 0.005, const size_t &policy_freq = 2);

    void save() const;
    void load();

  private:
    class transition final
    {
    public:
      transition(const torch::Tensor &state, const long &action, const torch::Tensor &next_state, const double &reward, const bool &done) : state(state), next_state(next_state), action(action), reward(reward), done(done) {}
      ~transition() {}

      torch::Tensor state;
      long action;
      torch::Tensor next_state;
      double reward;
      bool done;
    };

    class transition_batch final
    {
    public:
      transition_batch(const std::vector<torch::Tensor> &states, const std::vector<long> &actions, const std::vector<torch::Tensor> &next_states, const std::vector<double> &rewards, const std::vector<bool> &dones) : states(states), next_states(next_states), actions(actions), rewards(rewards), dones(dones) {}
      ~transition_batch() {}

      std::vector<torch::Tensor> states;
      std::vector<long> actions;
      std::vector<torch::Tensor> next_states;
      std::vector<double> rewards;
      std::vector<bool> dones;
    };

    class reply_buffer final
    {
    public:
      reply_buffer(const size_t &size = 1e3);
      ~reply_buffer();

      size_t get_size() const noexcept { return size; }

      void add(const torch::Tensor &state, const long &action, const torch::Tensor &next_state, const double &reward, const bool &done);
      void add(const transition &tr);

      transition_batch sample(const size_t &batch_size) const;

    private:
      const size_t size;
      size_t ptr = 0;
      std::vector<transition> storage;
    };

  protected:
    const size_t state_dim, action_dim;
    std::default_random_engine gen;
    std::uniform_real_distribution<double> unif = std::uniform_real_distribution<double>(0, 1);

  private:
    torch::Device device;
    torch::Tensor state;
    double eps = 1;
    agent policy;
    torch::optim::Adam policy_optimizer;
    agent target;
    reply_buffer buffer;
  };
} // namespace rl
