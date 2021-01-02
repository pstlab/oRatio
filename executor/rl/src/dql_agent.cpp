#include "dql_agent.h"

using namespace torch;

namespace rl
{
    dql_agent::dql_agent(const size_t &state_dim, const size_t &action_dim, const torch::Tensor &init_state) : state_dim(state_dim), action_dim(action_dim), device(torch::cuda::is_available() ? kCUDA : kCPU), state(init_state), policy(state_dim, action_dim), policy_optimizer(policy->parameters()), target(state_dim, action_dim)
    {
        // we set the target network in eval mode (this network will not be trained)..
        target->eval();

        // we copy the policy parameters into the target network..
        const auto policy_pars = policy->parameters();
        const auto target_pars = target->parameters();
        for (size_t i = 0; i < policy_pars.size(); i++)
            target_pars.at(i).data().copy_(policy_pars.at(i));
    }
    dql_agent::~dql_agent() {}

    double dql_agent::evaluate(const torch::Tensor &init_state, const size_t &eval_episodes) noexcept
    {
        double avg_reward = 0;
        for (size_t i = 0; i < eval_episodes; ++i)
        {
            set_state(init_state);
            bool done = false;
            while (!done)
            {
                const auto action = select_action();
                const auto result = execute_action(action);
                avg_reward += std::get<1>(result);
                if (std::get<2>(result))
                { // we reset the initial state..
                    done = true;
                    set_state(init_state);
                }
                else // we move to the next state..
                    set_state(std::get<0>(result));
            }
        }
        return avg_reward / eval_episodes;
    }

    size_t dql_agent::select_action()
    {
        if (unif(gen) < eps) // we randomly select actions (exploration)..
            return std::uniform_int_distribution<size_t>(0, action_dim - 1)(gen);
        else // we select our currently best actions (exploitation)..
            return policy->forward(state).to(device).argmax().item<long long>();
    }

    void dql_agent::train(const size_t &iterations, const size_t &batch_size, const double &discount, const double &alpha, const double &eps_decay, const size_t &policy_freq)
    { // the Deep Q-Learning (DQL) algorithm..
        for (size_t it = 0; it < iterations; ++it)
        {
            // we get a sample from the experience replay memory..
            const auto c_sample = buffer.sample(batch_size);
            std::vector<Tensor> states;
            states.reserve(batch_size);
            std::vector<Tensor> actions;
            actions.reserve(batch_size);
            std::vector<Tensor> next_states;
            next_states.reserve(batch_size);
            std::vector<Tensor> rewards;
            rewards.reserve(batch_size);
            std::vector<Tensor> dones;
            dones.reserve(batch_size);
            for (size_t i = 0; i < batch_size; ++i)
            {
                states.push_back(c_sample.states.at(i));
                actions.push_back(torch::tensor(c_sample.actions.at(i)).to(device));
                next_states.push_back(c_sample.next_states.at(i));
                rewards.push_back(torch::tensor(c_sample.rewards.at(i)).to(device));
                dones.push_back(torch::tensor(c_sample.dones.at(i)).to(device));
            }
            const auto c_states = stack(states).to(device);
            const auto c_actions = stack(actions).to(device);
            const auto c_next_states = stack(next_states).to(device);
            const auto c_rewards = stack(rewards).to(device);
            const auto c_dones = stack(dones).to(device);

            // we get the current qs for the current states and actions..
            const auto current_qs = policy->forward(c_states).to(device).gather(-1, c_actions.unsqueeze(-1)).squeeze();
            const auto max_qs = std::get<0>(target->forward(c_next_states).to(device).max(1)).detach();
            const auto expected_qs = c_rewards + (1 - c_dones) * max_qs * discount;

            // we compute the loss..
            const auto policy_loss = torch::nn::functional::mse_loss(current_qs, expected_qs);

            // we use this loss to backpropagate through SGD..
            policy_optimizer.zero_grad(); // we first set the gradients at zero..
            policy_loss.backward();       // we then compute the gradients according to the loss..
            policy_optimizer.step();      // we finally update the parameters of the critic model..

            if (it % policy_freq == 0)
            { // we copy the policy parameters into the target network..
                const auto policy_pars = policy->parameters();
                const auto target_pars = target->parameters();
                for (size_t i = 0; i < policy_pars.size(); i++)
                    target_pars.at(i).data().copy_(policy_pars.at(i));
            }
        }
        eps *= 1 - eps_decay;
    }

    void dql_agent::save() const { torch::save(policy, "actor.pt"); }

    void dql_agent::load() { torch::load(policy, "actor.pt"); }

    dql_agent::reply_buffer::reply_buffer(const size_t &size) : size(size) { storage.reserve(size); }
    dql_agent::reply_buffer::~reply_buffer() {}

    void dql_agent::reply_buffer::add(const torch::Tensor &state, const long &action, const torch::Tensor &next_state, const double &reward, const bool &done)
    {
        if (storage.size() == size)
        {
            storage[ptr] = transition(state, action, next_state, reward, done);
            ptr = (ptr + 1) % size;
        }
        else
            storage.emplace_back(state, action, next_state, reward, done);
    }

    void dql_agent::reply_buffer::add(const transition &tr)
    {
        if (storage.size() == size)
        {
            storage[ptr] = tr;
            ptr = (ptr + 1) % size;
        }
        else
            storage.push_back(tr);
    }

    dql_agent::transition_batch dql_agent::reply_buffer::sample(const size_t &batch_size) const
    {
        std::vector<torch::Tensor> states;
        states.reserve(batch_size);
        std::vector<long> actions;
        actions.reserve(batch_size);
        std::vector<torch::Tensor> next_states;
        next_states.reserve(batch_size);
        std::vector<double> rewards;
        rewards.reserve(batch_size);
        std::vector<int> dones;
        dones.reserve(batch_size);

        const auto rnd_ids = torch::randint(size, batch_size).detach();
        for (size_t i = 0; i < batch_size; ++i)
        {
            const auto id = rnd_ids[i].item<int>();
            states.push_back(storage.at(id).state);
            actions.push_back(storage.at(id).action);
            next_states.push_back(storage.at(id).next_state);
            rewards.push_back(storage.at(id).reward);
            dones.push_back(storage.at(id).done);
        }
        return transition_batch(states, actions, next_states, rewards, dones);
    }
} // namespace rl
