#pragma once

#include <torch/torch.h>
#include <random>

namespace rl
{
  class dql_environment;

  struct agentImpl : torch::nn::Module
  {
    agentImpl(const size_t &state_dim, const size_t &action_dim) : layer_0(torch::nn::Linear(state_dim, 40)), layer_1(torch::nn::Linear(40, 30)), layer_2(torch::nn::Linear(30, action_dim))
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
    class reply_buffer;

  public:
    dql_agent(dql_environment &env, const size_t &buffer_size = 1e3);
    ~dql_agent();

    reply_buffer &get_buffer() noexcept { return buffer; }

    size_t select_best_action() noexcept;
    size_t select_random_action() noexcept;
    size_t select_softmax_action(const double &t = 1) noexcept;

  private:
    void optimize(const size_t &batch_size, const double &gamma) noexcept;

  public:
    void train(const torch::Tensor &init_state, const size_t &iterations, const size_t &batch_size = 100, const double &gamma = 0.95, const double &eps_decay = 200, const size_t &policy_freq = 5) noexcept;

    double evaluate(const torch::Tensor &init_state, const size_t &max_steps, const size_t &eval_episodes = 10) noexcept;

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
      transition_batch(const std::vector<torch::Tensor> &states, const std::vector<long> &actions, const std::vector<torch::Tensor> &next_states, const std::vector<double> &rewards, const std::vector<int> &dones) : states(states), next_states(next_states), actions(actions), rewards(rewards), dones(dones) {}
      ~transition_batch() {}

      std::vector<torch::Tensor> states;
      std::vector<long> actions;
      std::vector<torch::Tensor> next_states;
      std::vector<double> rewards;
      std::vector<int> dones;
    };

    class reply_buffer final
    {
    public:
      reply_buffer(const size_t &size = 1e3);
      ~reply_buffer();

      size_t get_size() const noexcept { return size; }
      bool is_full() const noexcept { return storage.size() == size; }

      void add(const torch::Tensor &state, const long &action, const torch::Tensor &next_state, const double &reward, const bool &done);
      void add(const transition &tr);

      transition_batch sample(const size_t &batch_size) const;

    private:
      const size_t size;
      size_t ptr = 0;
      std::vector<transition> storage;
    };

  private:
    std::default_random_engine gen;
    std::uniform_real_distribution<double> unif = std::uniform_real_distribution<double>(0, 1);
    dql_environment &env;
    torch::Device device = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;
    agent policy;
    torch::optim::Adam policy_optimizer;
    agent target;
    reply_buffer buffer;
    const float eps_start = 0.9, eps_end = 0.05;
    size_t steps_done = 0;
  };
} // namespace rl
