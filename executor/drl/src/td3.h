#pragma once

#include "drl.h"

namespace drl
{
  struct actorImpl : torch::nn::Module
  {
    actorImpl(const size_t state_dim, const size_t action_dim) : layer_0(torch::nn::Linear(state_dim, 400)), layer_1(torch::nn::Linear(400, 300)), layer_2(torch::nn::Linear(300, action_dim))
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
    criticImpl(const size_t state_dim, const size_t action_dim) : layer_0(torch::nn::Linear(state_dim + action_dim, 400)), layer_1(torch::nn::Linear(400, 300)), layer_2(torch::nn::Linear(300, 1)), layer_3(torch::nn::Linear(state_dim + action_dim, 400)), layer_4(torch::nn::Linear(400, 300)), layer_5(torch::nn::Linear(300, 1))
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

  class td3 final
  {
  public:
    td3(const size_t &state_dim, const size_t &action_dim, const double &max_action);
    ~td3();

    torch::Tensor select_action(torch::Tensor state);

    void train(const size_t &iterations, const size_t &batch_size = 100, const double &discount = 0.99, const double &tau = 0.005, const double &policy_noise = 0.2, const double &noise_clip = 0.5, const size_t &policy_freq = 2);

  private:
    const double max_action;
    torch::Device device;
    actor actor_model, actor_target;
    critic critic_model, critic_target;
    reply_buffer buffer;
  };
} // namespace drl
