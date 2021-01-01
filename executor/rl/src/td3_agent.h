#pragma once

#include <torch/torch.h>

namespace rl
{
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
  public:
    td3_agent(const size_t &state_dim, const size_t &action_dim, const double &max_action, const torch::Tensor &init_state);
    ~td3_agent();

    torch::Tensor get_state() const { return state; }
    void set_state(const torch::Tensor &c_state) { state = c_state; }
    size_t get_state_dim() const { return state_dim; }
    size_t get_action_dim() const { return action_dim; }

    torch::Tensor select_action();
    virtual std::pair<std::vector<double>, double> execute_action(const torch::Tensor &action) noexcept { return {std::vector<double>(state_dim, 0), 0}; }

    void train(const size_t &iterations, const size_t &batch_size = 100, const double &discount = 0.99, const double &alpha = 0.005, const double &policy_noise = 0.2, const double &noise_clip = 0.5, const size_t &policy_freq = 2);

    void save() const;
    void load();

  private:
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

  protected:
    const size_t state_dim, action_dim;
    const double max_action;

  private:
    torch::Device device;
    torch::Tensor state;
    actor actor_model, actor_target;
    torch::optim::Adam actor_optimizer;
    critic critic_model, critic_target;
    torch::optim::Adam critic_optimizer;
    reply_buffer buffer;
  };
} // namespace rl
