#include "rl_agent.h"

namespace drl
{
    rl_agent::rl_agent(const size_t &state_dim, const size_t &action_dim, const size_t &c_state) : q_table(state_dim, std::vector<double>(action_dim, 0)), c_state(c_state) {}
    rl_agent::~rl_agent() {}

    size_t rl_agent::select_action() noexcept
    {
        if (rand() < eps)
        { // we randomly select actions (exploration)..
            std::discrete_distribution<size_t> distribution(q_table.at(c_state).begin(), q_table.at(c_state).end());
            return distribution(gen);
        }
        else
        { // we select our currently best actions (exploitation)..
            const auto &c_actions = q_table.at(c_state);
            size_t best_action = 0;
            double q = -std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < c_actions.size(); i++)
                if (c_actions.at(i) > q)
                {
                    q = c_actions.at(i);
                    best_action = i;
                }
            return best_action;
        }
    }

    void rl_agent::train(const size_t &iterations, const double &discount, const double &alpha) noexcept
    {
        for (size_t it = 0; it < iterations; ++it)
        {
            // we select an action..
            const auto c_action = select_action();
            // we execute the action, getting the resulting state and the reward..
            const auto result = execute_action(c_action);
            // this is the current q value for the current state and the selected action..
            double q = q_table.at(c_state).at(c_action);
            // this is the max among the qs for the resulting state and the selected action..
            double max_q = *std::max_element(q_table.at(result.first).begin(), q_table.at(result.first).end());
            // this is the expected q..
            double expected_q = result.second + max_q * discount;
            // we update the q table..
            q_table.at(c_state).assign(c_action, alpha * (expected_q - q));
            // we update the current state..
            c_state = result.first;
        }
    }
} // namespace drl
