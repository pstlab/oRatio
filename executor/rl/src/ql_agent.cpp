#include "ql_agent.h"
#include <iomanip>

namespace rl
{
    ql_agent::ql_agent(const size_t &state_dim, const size_t &action_dim, const size_t &init_state) : state_dim(state_dim), action_dim(action_dim), q_table(state_dim, std::vector<double>(action_dim, 0)), state(init_state) {}
    ql_agent::~ql_agent() {}

    double ql_agent::evaluate(const size_t &init_state, const size_t &eval_episodes) noexcept
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

    size_t ql_agent::select_action() noexcept
    {
        if (unif(gen) < eps) // we randomly select actions (exploration)..
            return std::uniform_int_distribution<size_t>(0, action_dim - 1)(gen);
        else
        { // we select our currently best actions (exploitation)..
            const auto &c_actions = q_table.at(state);
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

    void ql_agent::train(const size_t &iterations, const double &discount, const double &alpha, const double &eps_decay) noexcept
    {
        for (size_t it = 0; it < iterations; ++it)
        {
            // we select an action..
            const auto c_action = select_action();
            // we execute the action, getting the resulting state and the reward..
            const auto result = execute_action(c_action);
            // this is the current q value for the current state and the selected action..
            double q = q_table.at(state).at(c_action);
            // this is the max among the qs for the resulting state and the selected action..
            double max_q = *std::max_element(q_table.at(std::get<0>(result)).begin(), q_table.at(std::get<0>(result)).end());
            // this is the expected q..
            double expected_q = std::get<1>(result) + max_q * discount;
            // we update the q table..
            q_table[state][c_action] += alpha * (expected_q - q);

            if (std::get<2>(result)) // we have done with our task..
                break;
            else // we update the current state..
                state = std::get<0>(result);
        }
        eps *= 1 - eps_decay;
    }

    std::ostream &operator<<(std::ostream &os, const ql_agent &ql)
    {
        for (size_t s = 0; s < ql.state_dim; ++s)
        {
            for (size_t a = 0; a < ql.action_dim; ++a)
                os << "q(" << s << ", " << a << "):" << std::fixed << std::setprecision(2) << ql.q_table.at(s).at(a) << ' ';
            os << std::endl;
        }

        return os;
    }
} // namespace rl
