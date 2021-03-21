#pragma once

#include <torch/torch.h>
#include <random>

namespace rl
{
  class td3_environment;

  struct actorImpl : torch::nn::Module
  {
    actorImpl(const size_t &state_dim, const size_t &action_dim) : layer_0(torch::nn::Linear(state_dim, 400)), layer_1(torch::nn::Linear(400, 300)), layer_2(torch::nn::Linear(300, action_dim))
    {
      register_module("layer_0", layer_0);
      register_module("layer_1", layer_1);
      register_module("layer_2", layer_2);
    }

    torch::Tensor forward(torch::Tensor x)
    {
      x = torch::relu(layer_0(x));
      x = torch::relu(layer_1(x));
      x = torch::tanh(layer_2(x));
      return x;
    }

  private:
    torch::nn::Linear layer_0, layer_1, layer_2;
  };
  TORCH_MODULE(actor);

  struct criticImpl : torch::nn::Module
  {
    criticImpl(const size_t &state_dim, const size_t &action_dim) : layer_0(torch::nn::Linear(state_dim + action_dim, 400)), layer_1(torch::nn::Linear(400, 300)), layer_2(torch::nn::Linear(300, 1)), layer_3(torch::nn::Linear(state_dim + action_dim, 400)), layer_4(torch::nn::Linear(400, 300)), layer_5(torch::nn::Linear(300, 1))
    {
      register_module("layer_0", layer_0);
      register_module("layer_1", layer_1);
      register_module("layer_2", layer_2);
      register_module("layer_3", layer_3);
      register_module("layer_4", layer_4);
      register_module("layer_5", layer_5);
    }

    std::pair<torch::Tensor, torch::Tensor> forward(torch::Tensor x, torch::Tensor y)
    {
      auto xy = torch::cat({x, y}, 1);
      x = torch::relu(layer_0(xy));
      x = torch::relu(layer_1(x));
      x = layer_2(x);
      y = torch::relu(layer_3(xy));
      y = torch::relu(layer_4(x));
      y = layer_5(x);
      return {x, y};
    }

    torch::Tensor q1(torch::Tensor x, torch::Tensor y)
    {
      auto xy = torch::cat({x, y}, 1);
      x = torch::relu(layer_0(xy));
      x = torch::relu(layer_1(x));
      x = layer_2(x);
      return x;
    }

  private:
    torch::nn::Linear layer_0, layer_1, layer_2;
    torch::nn::Linear layer_3, layer_4, layer_5;
  };
  TORCH_MODULE(critic);

  class td3_agent final
  {
    class reply_buffer;

  public:
    td3_agent(td3_environment &env, const size_t &buffer_size = 1e3);
    ~td3_agent();

    reply_buffer &get_buffer() noexcept { return buffer; }

    torch::Tensor select_action() noexcept;

    void train(const size_t &iterations, const size_t &batch_size = 100, const double &gamma = 0.99, const double &alpha = 0.005, const double &policy_noise = 0.2, const double &noise_clip = 0.5, const size_t &policy_freq = 2);

    double evaluate(const torch::Tensor &init_state, const size_t &max_steps, const size_t &eval_episodes = 10) noexcept;

    void save() const;
    void load();

  private:
    class transition final
    {
    public:
      transition(const torch::Tensor &state, const torch::Tensor &action, const torch::Tensor &next_state, const double &reward, const bool &done) : state(state), next_state(next_state), action(action), reward(reward), done(done) {}
      ~transition() {}

      torch::Tensor state;
      torch::Tensor action;
      torch::Tensor next_state;
      double reward;
      bool done;
    };

    class transition_batch final
    {
    public:
      transition_batch(std::vector<torch::Tensor> states, std::vector<torch::Tensor> actions, std::vector<torch::Tensor> next_states, std::vector<double> rewards, const std::vector<bool> &dones) : states(states), next_states(next_states), actions(actions), rewards(rewards), dones(dones) {}
      ~transition_batch() {}

      std::vector<torch::Tensor> states;
      std::vector<torch::Tensor> actions;
      std::vector<torch::Tensor> next_states;
      std::vector<double> rewards;
      std::vector<bool> dones;
    };

    class reply_buffer final
    {
    public:
      reply_buffer(const size_t &size = 1e5);
      ~reply_buffer();

      size_t get_size() const noexcept { return size; }

      void add(const transition &tr);

      transition_batch sample(const size_t &batch_size) const;

    private:
      const size_t size;
      size_t ptr = 0;
      std::vector<transition> storage;
    };

  private:
    td3_environment &env;
    torch::Device device = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;
    actor actor_model, actor_target;
    torch::optim::Adam actor_optimizer;
    critic critic_model, critic_target;
    torch::optim::Adam critic_optimizer;
    reply_buffer buffer;
  };
} // namespace rl
