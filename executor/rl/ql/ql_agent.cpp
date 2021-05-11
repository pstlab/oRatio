#include "ql_agent.h"
#include "ql_environment.h"
#include <iomanip>
#include <iostream>

namespace rl
{
    ql_agent::ql_agent(ql_environment &env) : env(env) { q_table.emplace(env.get_state(), std::vector<double>(env.get_action_dim(), 0)); }
    ql_agent::~ql_agent() {}

    size_t ql_agent::select_best_action() noexcept
    {
        auto it_a = q_table.find(env.get_state());
        if (it_a == q_table.cend())
            it_a = q_table.emplace(env.get_state(), std::vector<double>(env.get_action_dim(), 0)).first;
        size_t best_action = 0;
        double q = -std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < it_a->second.size(); ++i)
            if (it_a->second.at(i) > q)
            {
                q = it_a->second.at(i);
                best_action = i;
            }
        return best_action;
    }

    size_t ql_agent::select_random_action() noexcept { return std::uniform_int_distribution<>(0, env.get_action_dim() - 1)(gen); }

    size_t ql_agent::select_softmax_action(const double &t) noexcept
    {
        auto it_a = q_table.find(env.get_state());
        if (it_a == q_table.cend())
            it_a = q_table.emplace(env.get_state(), std::vector<double>(env.get_action_dim(), 0)).first;
        double den = 0;
        std::vector<double> softmax(env.get_action_dim(), 0);
        for (size_t i = 0; i < it_a->second.size(); ++i)
        {
            const double c_exp = std::exp(it_a->second.at(i) / t);
            softmax[i] = c_exp;
            den += c_exp;
        }
        for (size_t i = 0; i < it_a->second.size(); ++i)
            softmax[i] /= den;
        return std::discrete_distribution<>(softmax.cbegin(), softmax.cend())(gen);
    }

    void ql_agent::train(const size_t &iterations, const double &gamma, const double &alpha, const double &eps_decay) noexcept
    {
        for (size_t it = 0; it < iterations; ++it)
        {
            // the current state..
            const auto c_state = env.get_state();
            // we select an action..
            const float eps_threshold = eps_end + (eps_start - eps_end) * std::exp(-1. * steps_done / eps_decay);
            std::cout << eps_threshold << '\n';
            steps_done++;
            const auto c_action = unif(gen) < eps_threshold ? select_random_action() : select_best_action();
            // we execute the action, getting the resulting state and the reward..
            const auto result = env.execute_action(c_action);
            // this is the current q value for the current state and the selected action..
            double q = q_table.at(c_state).at(c_action);
            // this is the max among the qs for the resulting state and the selected action..
            auto it_a = q_table.find(env.get_state());
            if (it_a == q_table.cend())
                it_a = q_table.emplace(env.get_state(), std::vector<double>(env.get_action_dim(), 0)).first;
            double max_q = *std::max_element(it_a->second.cbegin(), it_a->second.cend());
            // this is the expected q..
            double expected_q = result.reward + max_q * gamma;
            // we update the q table..
            q_table.at(c_state)[c_action] += alpha * (expected_q - q);

            if (result.done)
                break; // we have done with our task..
        }
    }

    double ql_agent::evaluate(const size_t &init_state, const size_t &max_steps, const size_t &eval_episodes) noexcept
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

    std::ostream &operator<<(std::ostream &os, const ql_agent &ql)
    {
        for (const auto &c_actions : ql.q_table)
        {
            for (size_t i = 0; i < c_actions.second.size(); ++i)
                os << "q(" << c_actions.first << ", " << i << "):" << std::fixed << std::setprecision(2) << ql.q_table.at(c_actions.first).at(i) << ' ';
            os << std::endl;
        }

        return os;
    }
} // namespace rl
