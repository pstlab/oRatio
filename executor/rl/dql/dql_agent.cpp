#include "dql_agent.h"
#include "dql_environment.h"

using namespace torch;

namespace rl
{
    dql_agent::dql_agent(dql_environment &env, const size_t &buffer_size) : env(env), policy(env.get_state_dim(), env.get_action_dim()), policy_optimizer(policy->parameters()), target(env.get_state_dim(), env.get_action_dim()), buffer(buffer_size)
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

    size_t dql_agent::select_best_action() noexcept
    {
        torch::NoGradGuard no_grad;
        return policy->forward(env.get_state()).to(device).argmax().item<long long>();
    }

    size_t dql_agent::select_random_action() noexcept { return std::uniform_int_distribution<>(0, env.get_action_dim() - 1)(gen); }

    size_t dql_agent::select_softmax_action(const double &t) noexcept
    {
        torch::NoGradGuard no_grad;
        return (policy->forward(env.get_state()) / t).softmax(0).multinomial(1).item<long long>();
    }

    void dql_agent::optimize(const size_t &batch_size, const double &gamma) noexcept
    { // the Deep Q-Learning (DQL) algorithm..
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
        const auto expected_qs = c_rewards + (1 - c_dones) * max_qs * gamma;

        // we compute the loss..
        const auto policy_loss = torch::nn::functional::mse_loss(current_qs, expected_qs);

        // we use this loss to backpropagate through SGD..
        policy_optimizer.zero_grad(); // we first set the gradients at zero..
        policy_loss.backward();       // we then compute the gradients according to the loss..
        policy_optimizer.step();      // we finally update the parameters of the critic model..
    }

    void dql_agent::train(const torch::Tensor &init_state, const size_t &iterations, const size_t &batch_size, const double &gamma, const double &eps_decay, const size_t &policy_freq) noexcept
    {
        for (size_t i = 0; i < iterations; ++i)
        {
            // the current state..
            const auto c_state = env.get_state();
            // we select an action..
            size_t c_action;
            if (buffer.is_full())
            {
                const float eps_threshold = eps_end + (eps_start - eps_end) * std::exp(-1. * steps_done / eps_decay);
                steps_done++;
                c_action = unif(gen) < eps_threshold ? select_random_action() : select_best_action();
            }
            else
                c_action = select_random_action();
            // we execute the action..
            const auto result = env.execute_action(c_action);
            // we store the transition in the buffer..
            buffer.add(c_state, c_action, env.get_state(), result.reward, result.done);
            if (result.done) // we reset the initial state..
                env.set_state(init_state);

            if (buffer.is_full())
            { // we can optimize the policy network..
                // we perform an optimization step..
                optimize(batch_size, gamma);

                if (i != 0 && i % policy_freq == 0)
                { // we copy the policy parameters into the target network..
                    const auto policy_pars = policy->parameters();
                    const auto target_pars = target->parameters();
                    for (size_t i = 0; i < policy_pars.size(); i++)
                        target_pars.at(i).data().copy_(policy_pars.at(i));
                }
            }
        }
    }

    double dql_agent::evaluate(const torch::Tensor &init_state, const size_t &max_steps, const size_t &eval_episodes) noexcept
    {
        double avg_reward = 0;
        for (size_t i = 0; i < eval_episodes; ++i)
        {
            env.set_state(init_state);
            size_t c_step = 0;
            bool done = false;
            while (!done)
            {
                const auto action = select_best_action();
                const auto result = env.execute_action(action);
                avg_reward += result.reward;
                if (result.done || c_step++ == max_steps)
                { // we reset the initial state..
                    done = true;
                    env.set_state(init_state);
                }
            }
        }
        return avg_reward / eval_episodes;
    }

    void dql_agent::save() const { torch::save(policy, "actor.pt"); }

    void dql_agent::load() { torch::load(policy, "actor.pt"); }

    dql_agent::reply_buffer::reply_buffer(const size_t &size) : size(size) { storage.reserve(size); }
    dql_agent::reply_buffer::~reply_buffer() {}

    void dql_agent::reply_buffer::add(const torch::Tensor &state, const long &action, const torch::Tensor &next_state, const double &reward, const bool &done)
    {
        if (is_full())
        {
            storage[ptr] = transition(state, action, next_state, reward, done);
            ptr = (ptr + 1) % size;
        }
        else
            storage.emplace_back(state, action, next_state, reward, done);
    }

    void dql_agent::reply_buffer::add(const transition &tr)
    {
        if (is_full())
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
