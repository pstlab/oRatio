#include "ql_agent.h"
#include "ql_environment.h"
#include <iomanip>

namespace rl
{
    ql_agent::ql_agent(ql_environment &env) : env(env) {}
    ql_agent::~ql_agent() {}

    size_t ql_agent::select_best_action() noexcept
    {
        auto &c_actions = get_actions();
        size_t best_action = 0;
        double q = -std::numeric_limits<double>::infinity();
        for (const auto &c_act : c_actions)
            if (c_act.second > q)
            {
                q = c_act.second;
                best_action = c_act.first;
            }
        return best_action;
    }

    size_t ql_agent::select_random_action() noexcept { return std::uniform_int_distribution<>(0, env.get_action_dim() - 1)(gen); }

    size_t ql_agent::select_softmax_action(const double &t) noexcept
    {
        double den = 0;
        std::vector<double> softmax(env.get_action_dim(), 0);
        auto &c_actions = get_actions();
        for (size_t i = 0; i < c_actions.size(); ++i)
        {
            const double c_exp = std::exp(c_actions.at(i) / t);
            softmax[i] = c_exp;
            den += c_exp;
        }
        for (size_t i = 0; i < c_actions.size(); ++i)
            softmax[i] /= den;
        return std::discrete_distribution<>(softmax.begin(), softmax.end())(gen);
    }

    std::unordered_map<size_t, double> &ql_agent::get_actions() noexcept
    {
        auto &c_actions = q_table[env.get_state()];
        if (c_actions.empty())
            for (size_t i = 0; i < env.get_action_dim(); ++i)
                c_actions.emplace(i, 0);
        return c_actions;
    }

    void ql_agent::train(const size_t &iterations, const double &gamma, const double &alpha, const double &eps_decay) noexcept
    {
        for (size_t it = 0; it < iterations; ++it)
        {
            // the current state..
            const auto c_state = env.get_state();
            // we select an action..
            const float eps_threshold = eps_end + (eps_start - eps_end) * std::exp(-1. * steps_done / eps_decay);
            steps_done++;
            const auto c_action = unif(gen) < eps_threshold ? select_random_action() : select_best_action();
            // we execute the action, getting the resulting state and the reward..
            const auto result = env.execute_action(c_action);
            // this is the current q value for the current state and the selected action..
            double q = q_table.at(c_state).at(c_action);
            // this is the max among the qs for the resulting state and the selected action..
            const auto &c_actions = q_table.at(env.get_state());
            double max_q = std::max_element(c_actions.begin(), c_actions.end())->second;
            // this is the expected q..
            double expected_q = result.reward + max_q * gamma;
            // we update the q table..
            q_table[c_state][c_action] += alpha * (expected_q - q);

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
            for (const auto &c_act : c_actions.second)
                os << "q(" << c_actions.first << ", " << c_act.first << "):" << std::fixed << std::setprecision(2) << ql.q_table.at(c_actions.first).at(c_act.first) << ' ';
            os << std::endl;
        }

        return os;
    }
} // namespace rl
