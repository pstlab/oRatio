#include "td3.h"

using namespace torch;

namespace drl
{
    td3::td3(const size_t &state_dim, const size_t &action_dim, const double &max_action) : max_action(max_action), device(torch::cuda::is_available() ? kCUDA : kCPU), actor_model(state_dim, action_dim), actor_target(state_dim, action_dim), actor_optimizer(actor_model->parameters()), critic_model(state_dim, action_dim), critic_target(state_dim, action_dim), critic_optimizer(critic_model->parameters())
    {
        save(actor_model, "actor.pt");
        load(actor_target, "actor.pt");
        save(critic_model, "critic.pt");
        load(critic_target, "critic.pt");
    }
    td3::~td3() {}

    Tensor td3::select_action(Tensor state) { return actor_model->forward(state).to(device); }

    void td3::train(const size_t &iterations, const size_t &batch_size, const double &discount, const double &tau, const double &policy_noise, const double &noise_clip, const size_t &policy_freq)
    { // we implement the Twin-Delayed DDPG (TD3) algorithm..
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
            const auto current_qs = critic_model(state, action);

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
                    actor_target_pars.at(i).data().copy_(tau * actor_model_pars.at(i) + (1 - tau) * actor_target_pars.at(i));

                // we update the critic target's parameters by polyak avareging..
                const auto critic_model_pars = critic_model->parameters();
                const auto critic_target_pars = critic_target->parameters();
                for (size_t i = 0; i < critic_model_pars.size(); i++)
                    critic_target_pars.at(i).data().copy_(tau * critic_model_pars.at(i) + (1 - tau) * critic_target_pars.at(i));
            }
        }
    }
} // namespace drl
