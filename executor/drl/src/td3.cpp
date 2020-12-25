#include "td3.h"

using namespace torch;

namespace drl
{
    td3::td3(const size_t &state_dim, const size_t &action_dim, const double &max_action) : max_action(max_action), device(torch::cuda::is_available() ? kCUDA : kCPU), actor_model(state_dim, action_dim), actor_target(state_dim, action_dim), critic_model(state_dim, action_dim), critic_target(state_dim, action_dim)
    {
        save(actor_model, "actor.pt");
        load(actor_target, "actor.pt");
        save(critic_model, "critic.pt");
        load(critic_target, "critic.pt");
    }
    td3::~td3() {}

    Tensor td3::select_action(Tensor state) { return actor_model->forward(state).to(device); }

    void td3::train(const size_t &iterations, const size_t &batch_size, const double &discount, const double &tau, const double &policy_noise, const double &noise_clip, const size_t &policy_freq)
    {
        for (size_t it = 0; it < iterations; ++it)
        {
            // we get a sample from the experience replay memory..
            const auto c_sample = buffer.sample(batch_size);
            std::vector<Tensor> states;
            states.reserve(batch_size);
            std::vector<Tensor> next_states;
            next_states.reserve(batch_size);
            std::vector<Tensor> actions;
            actions.reserve(batch_size);
            std::vector<Tensor> rewards;
            rewards.reserve(batch_size);
            for (size_t i = 0; i < batch_size; ++i)
            {
                states.push_back(torch::tensor(c_sample.states.at(i)).to(device));
                next_states.push_back(torch::tensor(c_sample.next_states.at(i)).to(device));
                actions.push_back(torch::tensor(c_sample.actions.at(i)).to(device));
                rewards.push_back(torch::tensor(c_sample.rewards.at(i)).to(device));
            }
            const auto state = stack(states).to(device);
            const auto next_state = stack(next_states).to(device);
            const auto action = stack(actions).to(device);
            const auto reward = stack(rewards).to(device);

            // we select the best action for the next state..
            auto next_action = actor_target->forward(next_state).to(device);

            // we add some noice to the action so as to manage exploration/exploitation..
            auto noise = next_action.data().normal_(0, policy_noise).to(device).clamp(-noise_clip, noise_clip);
            next_action = (next_action + noise).clamp(-max_action, max_action);
        }
    }
} // namespace drl
