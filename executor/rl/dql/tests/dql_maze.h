#pragma once

#include "dql_environment.h"
#include <random>

namespace rl
{
    constexpr double act_rew = -0.01;

    class dql_maze : public dql_environment
    {
    public:
        dql_maze(const torch::Tensor &init_state) : dql_environment(2, 4, init_state) {}
        ~dql_maze() {}

        step execute_action(const size_t &action) noexcept override
        {
            const auto c_nondet = unif(gen);
            const auto c_state = get_state();
            const int x = c_state[0].item<int>();
            const int y = c_state[1].item<int>();

            switch (x)
            {
            case 0:
                switch (y)
                {
                case 0:
                    switch (action)
                    {
                    case 0:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({0.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({0.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({0.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                    case 1:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({0.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({1.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({0.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                    case 2:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({0.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({0.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({1.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                    case 3:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({1.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({0.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({0.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                    default:
                        break;
                    }
                    break;
                case 1:
                    switch (action)
                    {
                    case 0:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({0.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({0.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({0.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                    case 1:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({0.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({0.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({0.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                    case 2:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({0.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({0.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({0.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                    case 3:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({0.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({0.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({0.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                    default:
                        break;
                    }
                    break;
                case 2:
                    switch (action)
                    {
                    case 0:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({0.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({0.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({0.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                    case 1:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({0.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({1.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({0.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                    case 2:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({0.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({0.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({1.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                    case 3:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({1.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({0.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({0.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                    default:
                        break;
                    }
                    break;
                default:
                    break;
                }
                break;
            case 1:
                switch (y)
                {
                case 0:
                    switch (action)
                    {
                    case 0:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({1.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({1.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({0.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                    case 1:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({0.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({2.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({1.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                    case 2:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({1.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({1.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({2.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                    case 3:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({2.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({0.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({1.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                    default:
                        break;
                    }
                    break;
                case 2:
                    switch (action)
                    {
                    case 0:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({1.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({1.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({0.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                    case 1:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({0.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({2.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({1.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                    case 2:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({1.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({1.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({2.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                    case 3:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({2.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({0.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({1.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                    default:
                        break;
                    }
                    break;
                default:
                    break;
                }
                break;
            case 2:
                switch (y)
                {
                case 0:
                    switch (action)
                    {
                    case 0:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({2.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({2.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({1.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                    case 1:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({1.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({3.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({2.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                    case 2:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({2.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({2.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({3.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                    case 3:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({3.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({1.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({2.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                    default:
                        break;
                    }
                    break;
                case 1:
                    switch (action)
                    {
                    case 0:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({2.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({2.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({2.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                    case 1:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({2.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({3.0, 1.0}).detach());
                            return {-1, true};
                        }
                        else
                        {
                            set_state(torch::tensor({2.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                    case 2:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({2.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({2.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({3.0, 1.0}).detach());
                            return {-1, true};
                        }
                    case 3:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({3.0, 1.0}).detach());
                            return {-1, true};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({2.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({2.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                    default:
                        break;
                    }
                    break;
                case 2:
                    switch (action)
                    {
                    case 0:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({2.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({2.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({1.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                    case 1:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({1.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({3.0, 2.0}).detach());
                            return {1, true};
                        }
                        else
                        {
                            set_state(torch::tensor({0.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                    case 2:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({2.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({2.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({3.0, 2.0}).detach());
                            return {1, true};
                        }
                    case 3:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({3.0, 2.0}).detach());
                            return {1, true};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({1.0, 2.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({2.0, 1.0}).detach());
                            return {act_rew, false};
                        }
                    default:
                        break;
                    }
                    break;
                default:
                    break;
                }
                break;
            case 3:
                switch (y)
                {
                case 0:
                    switch (action)
                    {
                    case 0:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({3.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({3.0, 1.0}).detach());
                            return {-1, true};
                        }
                        else
                        {
                            set_state(torch::tensor({2.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                    case 1:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({2.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({3.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else
                            set_state(torch::tensor({3.0, 1.0}).detach());
                        return {-1, true};
                    case 2:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({3.0, 1.0}).detach());
                            return {-1, true};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({3.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({3.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                    case 3:
                        if (c_nondet <= 0.1)
                        {
                            set_state(torch::tensor({3.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else if (c_nondet >= 0.9)
                        {
                            set_state(torch::tensor({2.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                        else
                        {
                            set_state(torch::tensor({3.0, 0.0}).detach());
                            return {act_rew, false};
                        }
                    default:
                        break;
                    }
                    break;
                default:
                    break;
                }
                break;
            default:
                break;
            }
            return {act_rew, false};
        }

    private:
        std::default_random_engine gen;
        std::uniform_real_distribution<double> unif = std::uniform_real_distribution<double>(0, 1);
    };
} // namespace rl
