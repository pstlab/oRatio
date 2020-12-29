#pragma once

#include "drl.h"

namespace drl
{
  struct agentImpl : torch::nn::Module
  {
    agentImpl(const size_t state_dim, const size_t action_dim) : layer_0(torch::nn::Linear(state_dim, 400)), layer_1(torch::nn::Linear(400, 300)), layer_2(torch::nn::Linear(300, action_dim))
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

  class dqn final
  {
  public:
    dqn(const size_t state_dim, const size_t action_dim);
    ~dqn();

    torch::Tensor select_action(torch::Tensor state);

    void train(const size_t &iterations, const size_t &batch_size = 100, const double &discount = 0.99, const double &tau = 0.005, const size_t &policy_freq = 2);

    void save() const;
    void load();

  private:
    torch::Device device;
    agent policy, target;
    reply_buffer buffer;
  };
} // namespace drl
