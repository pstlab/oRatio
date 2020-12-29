#include "dqn.h"

using namespace torch;

namespace drl
{
    dqn::dqn(const size_t state_dim, const size_t action_dim) : device(torch::cuda::is_available() ? kCUDA : kCPU), policy(state_dim, action_dim), target(state_dim, action_dim)
    {
        // we copy the policy parameters into the target network..
        const auto policy_pars = policy->parameters();
        const auto target_pars = target->parameters();
        for (size_t i = 0; i < policy_pars.size(); i++)
            target_pars.at(i).data().copy_(policy_pars.at(i));
    }
    dqn::~dqn() {}

    Tensor dqn::select_action(Tensor state) { return policy->forward(state).to(device); }

    void dqn::train(const size_t &iterations, const size_t &batch_size, const double &discount, const double &tau, const size_t &policy_freq)
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
            const auto next_action = target->forward(next_state).to(device);
        }
    }

    void dqn::save() const { torch::save(policy, "actor.pt"); }

    void dqn::load() { torch::load(policy, "actor.pt"); }
} // namespace drl
