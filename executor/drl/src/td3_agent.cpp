#include "td3_agent.h"

using namespace torch;

namespace drl
{
    td3_agent::td3_agent(const size_t &state_dim, const size_t &action_dim, const double &max_action, const torch::Tensor &init_state) : state_dim(state_dim), action_dim(action_dim), max_action(max_action), device(torch::cuda::is_available() ? kCUDA : kCPU), state(init_state), actor_model(state_dim, action_dim), actor_target(state_dim, action_dim), actor_optimizer(actor_model->parameters()), critic_model(state_dim, action_dim), critic_target(state_dim, action_dim), critic_optimizer(critic_model->parameters())
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

    Tensor td3_agent::select_action() { return actor_model->forward(state).to(device); }

    void td3_agent::train(const size_t &iterations, const size_t &batch_size, const double &discount, const double &alpha, const double &policy_noise, const double &noise_clip, const size_t &policy_freq)
    { // we implement the Twin-Delayed DDPG (td3_agent) algorithm..
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
            const auto noise = next_action.data().normal_(0, policy_noise).to(device).clamp(-noise_clip, noise_clip);
            next_action = (next_action + noise).clamp(-max_action, max_action);

            // we get the minimun of the predicted qs for the next actions executed in the next states..
            const auto next_qs = critic_target->forward(next_state, next_action);
            const auto next_q = reward + (discount * torch::min(next_qs.first, next_qs.second)).detach();

            // we get the currents qs..
            const auto current_qs = critic_model->forward(state, action);

            // we compute the loss..
            const auto critic_loss = torch::nn::functional::mse_loss(current_qs.first, next_q) + torch::nn::functional::mse_loss(current_qs.second, next_q);

            // we use this loss to backpropagate through SGD..
            critic_optimizer.zero_grad(); // we first set the gradients at zero..
            critic_loss.backward();       // we then compute the gradients according to the loss..
            critic_optimizer.step();      // we finally update the parameters of the critic model..

            // once every 'policy_freq' iterations we update the actor model by performing gradient ascent..
            if (it % policy_freq == 0)
            { // The Deep Deterministic Policy Gradient..
                const auto actor_loss = -critic_model->q1(state, actor_model->forward(state)).mean();
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

    td3_agent::reply_buffer::reply_buffer(const size_t &size) : size(size) {}
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
        std::vector<std::vector<double>> states;
        states.reserve(batch_size);
        std::vector<std::vector<double>> next_states;
        next_states.reserve(batch_size);
        std::vector<std::vector<double>> actions;
        actions.reserve(batch_size);
        std::vector<double> rewards;
        rewards.reserve(batch_size);

        const auto rnd_ids = torch::randint(size, batch_size).detach();
        size_t *ptr = reinterpret_cast<size_t *>(rnd_ids.data_ptr());
        for (size_t i = 0; i < batch_size; ++i)
        {
            states.push_back(storage.at(*ptr).state);
            next_states.push_back(storage.at(*ptr).next_state);
            actions.push_back(storage.at(*ptr).action);
            rewards.push_back(storage.at(*ptr).reward);
            ptr++;
        }
        return transition_batch(states, next_states, actions, rewards);
    }
} // namespace drl