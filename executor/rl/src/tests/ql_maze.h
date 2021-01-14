#pragma once

#include "ql_environment.h"
#include <random>

namespace rl
{
    constexpr double act_rew = -0.01;

    class ql_maze : public ql_environment
    {
    public:
        ql_maze(const size_t &init_state) : ql_environment(11, 4, init_state) {}
        ~ql_maze() {}

        step execute_action(const size_t &action) noexcept override
        {
            const auto c_nondet = unif(gen);
            switch (get_state())
            {
            case 0:
                switch (action)
                {
                case 0:
                    if (c_nondet <= 0.1)
                    {
                        set_state(0);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(1);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(0);
                        return {act_rew, false};
                    }
                case 1:
                    if (c_nondet <= 0.1)
                    {
                        set_state(0);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(3);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(1);
                        return {act_rew, false};
                    }
                case 2:
                    if (c_nondet <= 0.1)
                    {
                        set_state(1);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(0);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(3);
                        return {act_rew, false};
                    }
                case 3:
                    if (c_nondet <= 0.1)
                    {
                        set_state(3);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(0);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(0);
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
                        set_state(0);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(2);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(1);
                        return {act_rew, false};
                    }
                case 1:
                    if (c_nondet <= 0.1)
                    {
                        set_state(1);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(1);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(2);
                        return {act_rew, false};
                    }
                case 2:
                    if (c_nondet <= 0.1)
                    {
                        set_state(2);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(0);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(1);
                        return {act_rew, false};
                    }
                case 3:
                    if (c_nondet <= 0.1)
                    {
                        set_state(1);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(1);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(0);
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
                        set_state(1);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(2);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(2);
                        return {act_rew, false};
                    }
                case 1:
                    if (c_nondet <= 0.1)
                    {
                        set_state(2);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(4);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(2);
                        return {act_rew, false};
                    }
                case 2:
                    if (c_nondet <= 0.1)
                    {
                        set_state(2);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(1);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(4);
                        return {act_rew, false};
                    }
                case 3:
                    if (c_nondet <= 0.1)
                    {
                        set_state(4);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(2);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(1);
                        return {act_rew, false};
                    }
                default:
                    break;
                }
                break;
            case 3:
                switch (action)
                {
                case 0:
                    if (c_nondet <= 0.1)
                    {
                        set_state(3);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(3);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(0);
                        return {act_rew, false};
                    }
                case 1:
                    if (c_nondet <= 0.1)
                    {
                        set_state(0);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(5);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(3);
                        return {act_rew, false};
                    }
                case 2:
                    if (c_nondet <= 0.1)
                    {
                        set_state(3);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(3);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(5);
                        return {act_rew, false};
                    }
                case 3:
                    if (c_nondet <= 0.1)
                    {
                        set_state(5);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(0);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(3);
                        return {act_rew, false};
                    }
                default:
                    break;
                }
                break;
            case 4:
                switch (action)
                {
                case 0:
                    if (c_nondet <= 0.1)
                    {
                        set_state(4);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(4);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(2);
                        return {act_rew, false};
                    }
                case 1:
                    if (c_nondet <= 0.1)
                    {
                        set_state(2);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(7);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(4);
                        return {act_rew, false};
                    }
                case 2:
                    if (c_nondet <= 0.1)
                    {
                        set_state(4);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(4);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(7);
                        return {act_rew, false};
                    }
                case 3:
                    if (c_nondet <= 0.1)
                    {
                        set_state(7);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(2);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(4);
                        return {act_rew, false};
                    }
                default:
                    break;
                }
                break;
            case 5:
                switch (action)
                {
                case 0:
                    if (c_nondet <= 0.1)
                    {
                        set_state(5);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(6);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(3);
                        return {act_rew, false};
                    }
                case 1:
                    if (c_nondet <= 0.1)
                    {
                        set_state(3);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(8);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(6);
                        return {act_rew, false};
                    }
                case 2:
                    if (c_nondet <= 0.1)
                    {
                        set_state(6);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(5);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(8);
                        return {act_rew, false};
                    }
                case 3:
                    if (c_nondet <= 0.1)
                    {
                        set_state(8);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(3);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(5);
                        return {act_rew, false};
                    }
                default:
                    break;
                }
                break;
            case 6:
                switch (action)
                {
                case 0:
                    if (c_nondet <= 0.1)
                    {
                        set_state(5);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(7);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(6);
                        return {act_rew, false};
                    }
                case 1:
                    if (c_nondet <= 0.1)
                    {
                        set_state(6);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(9);
                        return {-1, true};
                    }
                    else
                    {
                        set_state(7);
                        return {act_rew, false};
                    }
                case 2:
                    if (c_nondet <= 0.1)
                    {
                        set_state(7);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(5);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(9);
                        return {-1, true};
                    }
                case 3:
                    if (c_nondet <= 0.1)
                    {
                        set_state(9);
                        return {-1, true};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(6);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(5);
                        return {act_rew, false};
                    }
                default:
                    break;
                }
                break;
            case 7:
                switch (action)
                {
                case 0:
                    if (c_nondet <= 0.1)
                    {
                        set_state(6);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(7);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(4);
                        return {act_rew, false};
                    }
                case 1:
                    if (c_nondet <= 0.1)
                    {
                        set_state(4);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(10);
                        return {1, true};
                    }
                    else
                    {
                        set_state(1);
                        return {act_rew, false};
                    }
                case 2:
                    if (c_nondet <= 0.1)
                    {
                        set_state(7);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(6);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(10);
                        return {1, true};
                    }
                case 3:
                    if (c_nondet <= 0.1)
                    {
                        set_state(10);
                        return {1, true};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(4);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(6);
                        return {act_rew, false};
                    }
                default:
                    break;
                }
                break;
            case 8:
                switch (action)
                {
                case 0:
                    if (c_nondet <= 0.1)
                    {
                        set_state(8);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(9);
                        return {-1, true};
                    }
                    else
                    {
                        set_state(5);
                        return {act_rew, false};
                    }
                case 1:
                    if (c_nondet <= 0.1)
                    {
                        set_state(5);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(8);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(9);
                        return {-1, true};
                    }
                case 2:
                    if (c_nondet <= 0.1)
                    {
                        set_state(9);
                        return {-1, true};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(8);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(9);
                        return {act_rew, false};
                    }
                case 3:
                    if (c_nondet <= 0.1)
                    {
                        set_state(8);
                        return {act_rew, false};
                    }
                    else if (c_nondet >= 0.9)
                    {
                        set_state(5);
                        return {act_rew, false};
                    }
                    else
                    {
                        set_state(8);
                        return {act_rew, false};
                    }
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
