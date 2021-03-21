#include "td3_agent.h"
#include "td3_environment.h"

using namespace torch;

namespace rl
{
    td3_agent::td3_agent(td3_environment &env, const size_t &buffer_size) : env(env), actor_model(env.get_state_dim(), env.get_action_dim()), actor_target(env.get_state_dim(), env.get_action_dim()), actor_optimizer(actor_model->parameters()), critic_model(env.get_state_dim(), env.get_action_dim()), critic_target(env.get_state_dim(), env.get_action_dim()), critic_optimizer(critic_model->parameters()), buffer(buffer_size)
    {
        // we set the actor target network and the actor critic network in eval mode (these networks will not be trained)..
        actor_target->eval();
        critic_target->eval();

        // we copy the actor model parameters into the actor target network..
        const auto actor_model_pars = actor_model->parameters();
        const auto actor_target_pars = actor_target->parameters();
        for (size_t i = 0; i < actor_model_pars.size(); i++)
            actor_target_pars.at(i).data().copy_(actor_model_pars.at(i));

        // we copy the critic model parameters into the critic target network..
        const auto critic_model_pars = critic_model->parameters();
        const auto critic_target_pars = critic_target->parameters();
        for (size_t i = 0; i < critic_model_pars.size(); i++)
            critic_target_pars.at(i).data().copy_(critic_model_pars.at(i));
    }
    td3_agent::~td3_agent() {}

    Tensor td3_agent::select_action() noexcept { return actor_model->forward(env.get_state()).to(device); }

    void td3_agent::train(const size_t &iterations, const size_t &batch_size, const double &gamma, const double &alpha, const double &policy_noise, const double &noise_clip, const size_t &policy_freq)
    { // the Twin-Delayed DDPG (TD3) algorithm..
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
                actions.push_back(c_sample.actions.at(i));
                next_states.push_back(c_sample.next_states.at(i));
                rewards.push_back(torch::tensor(c_sample.rewards.at(i)).to(device));
                dones.push_back(torch::tensor(c_sample.dones.at(i)).to(device));
            }
            const auto c_states = stack(states).to(device);
            const auto c_actions = stack(actions).to(device);
            const auto c_next_states = stack(next_states).to(device);
            const auto c_rewards = stack(rewards).to(device);
            const auto c_dones = stack(dones).to(device);

            // we select the best action for the next state..
            auto next_action = actor_target->forward(c_next_states).to(device);

            // we add some noice to the action so as to manage exploration/exploitation..
            const auto noise = next_action.data().normal_(0, policy_noise).to(device).clamp(-noise_clip, noise_clip);
            next_action = (next_action + noise).clamp(-env.get_max_action(), env.get_max_action());

            // we get the minimun of the predicted qs for the next actions executed in the next states..
            const auto next_qs = critic_target->forward(c_next_states, next_action);
            const auto next_q = c_rewards + ((1 - c_dones) * gamma * torch::min(next_qs.first, next_qs.second)).detach();

            // we get the currents qs..
            const auto current_qs = critic_model->forward(c_states, c_actions);

            // we compute the loss..
            const auto critic_loss = torch::nn::functional::mse_loss(current_qs.first, next_q) + torch::nn::functional::mse_loss(current_qs.second, next_q);

            // we use this loss to backpropagate through SGD..
            critic_optimizer.zero_grad(); // we first set the gradients at zero..
            critic_loss.backward();       // we then compute the gradients according to the loss..
            critic_optimizer.step();      // we finally update the parameters of the critic model..

            // once every 'policy_freq' iterations we update the actor model by performing gradient ascent..
            if (it != 0 && it % policy_freq == 0)
            { // The Deep Deterministic Policy Gradient..
                const auto actor_loss = -critic_model->q1(c_states, actor_model->forward(c_states)).mean();
                actor_optimizer.zero_grad(); // we first set the gradients at zero..
                actor_loss.backward();       // we then compute the gradients according to the loss..
                actor_optimizer.step();      // we finally update the parameters of the critic model..

                // we update the actor target's parameters by polyak avareging..
                const auto actor_model_pars = actor_model->parameters();
                const auto actor_target_pars = actor_target->parameters();
                for (size_t i = 0; i < actor_model_pars.size(); i++)
                    actor_target_pars.at(i).data().copy_(alpha * actor_model_pars.at(i) + (1 - alpha) * actor_target_pars.at(i));

                // we update the critic target's parameters by polyak avareging..
                const auto critic_model_pars = critic_model->parameters();
                const auto critic_target_pars = critic_target->parameters();
                for (size_t i = 0; i < critic_model_pars.size(); i++)
                    critic_target_pars.at(i).data().copy_(alpha * critic_model_pars.at(i) + (1 - alpha) * critic_target_pars.at(i));
            }
        }
    }

    double td3_agent::evaluate(const torch::Tensor &init_state, const size_t &max_steps, const size_t &eval_episodes) noexcept
    {
        double avg_reward = 0;
        for (size_t i = 0; i < eval_episodes; ++i)
        {
            env.set_state(init_state);
            size_t c_step = 0;
            bool done = false;
            while (!done)
            {
                const auto action = select_action();
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

    void td3_agent::save() const
    {
        torch::save(actor_model, "actor.pt");
        torch::save(critic_model, "critic.pt");
    }

    void td3_agent::load()
    {
        torch::load(actor_target, "actor.pt");
        torch::load(critic_target, "critic.pt");
    }

    td3_agent::reply_buffer::reply_buffer(const size_t &size) : size(size) { storage.reserve(size); }
    td3_agent::reply_buffer::~reply_buffer() {}

    void td3_agent::reply_buffer::add(const transition &tr)
    {
        if (storage.size() == size)
        {
            storage[ptr] = tr;
            ptr = (ptr + 1) % size;
        }
        else
            storage.push_back(tr);
    }

    td3_agent::transition_batch td3_agent::reply_buffer::sample(const size_t &batch_size) const
    {
        std::vector<torch::Tensor> states;
        states.reserve(batch_size);
        std::vector<torch::Tensor> actions;
        actions.reserve(batch_size);
        std::vector<torch::Tensor> next_states;
        next_states.reserve(batch_size);
        std::vector<double> rewards;
        rewards.reserve(batch_size);
        std::vector<bool> dones;
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
        return transition_batch(states, next_states, actions, rewards, dones);
    }
} // namespace rl
