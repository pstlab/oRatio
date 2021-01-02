#include "ql_agent.h"
#include <sstream>
#include <cassert>

constexpr double act_rew = -0.01;

using namespace rl;

class maze_agent : public ql_agent
{
public:
    maze_agent() : ql_agent(11, 4, 0) {}
    ~maze_agent() {}

    std::tuple<size_t, double, bool> execute_action(const size_t &action) noexcept override
    {
        const auto c_nondet = unif(gen);
        switch (get_state())
        {
        case 0:
            switch (action)
            {
            case 0:
                if (c_nondet <= 0.1)
                    return {0, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {1, act_rew, false};
                else
                    return {0, act_rew, false};
            case 1:
                if (c_nondet <= 0.1)
                    return {0, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {3, act_rew, false};
                else
                    return {1, act_rew, false};
            case 2:
                if (c_nondet <= 0.1)
                    return {1, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {0, act_rew, false};
                else
                    return {3, act_rew, false};
            case 3:
                if (c_nondet <= 0.1)
                    return {3, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {0, act_rew, false};
                else
                    return {0, act_rew, false};
            default:
                break;
            }
            break;
        case 1:
            switch (action)
            {
            case 0:
                if (c_nondet <= 0.1)
                    return {0, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {2, act_rew, false};
                else
                    return {1, act_rew, false};
            case 1:
                if (c_nondet <= 0.1)
                    return {1, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {1, act_rew, false};
                else
                    return {2, act_rew, false};
            case 2:
                if (c_nondet <= 0.1)
                    return {2, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {0, act_rew, false};
                else
                    return {1, act_rew, false};
            case 3:
                if (c_nondet <= 0.1)
                    return {1, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {1, act_rew, false};
                else
                    return {0, act_rew, false};
            default:
                break;
            }
            break;
        case 2:
            switch (action)
            {
            case 0:
                if (c_nondet <= 0.1)
                    return {1, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {2, act_rew, false};
                else
                    return {2, act_rew, false};
            case 1:
                if (c_nondet <= 0.1)
                    return {2, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {4, act_rew, false};
                else
                    return {2, act_rew, false};
            case 2:
                if (c_nondet <= 0.1)
                    return {2, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {1, act_rew, false};
                else
                    return {4, act_rew, false};
            case 3:
                if (c_nondet <= 0.1)
                    return {4, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {2, act_rew, false};
                else
                    return {1, act_rew, false};
            default:
                break;
            }
            break;
        case 3:
            switch (action)
            {
            case 0:
                if (c_nondet <= 0.1)
                    return {3, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {3, act_rew, false};
                else
                    return {0, act_rew, false};
            case 1:
                if (c_nondet <= 0.1)
                    return {0, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {5, act_rew, false};
                else
                    return {3, act_rew, false};
            case 2:
                if (c_nondet <= 0.1)
                    return {3, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {3, act_rew, false};
                else
                    return {5, act_rew, false};
            case 3:
                if (c_nondet <= 0.1)
                    return {5, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {0, act_rew, false};
                else
                    return {3, act_rew, false};
            default:
                break;
            }
            break;
        case 4:
            switch (action)
            {
            case 0:
                if (c_nondet <= 0.1)
                    return {4, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {4, act_rew, false};
                else
                    return {2, act_rew, false};
            case 1:
                if (c_nondet <= 0.1)
                    return {2, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {7, act_rew, false};
                else
                    return {4, act_rew, false};
            case 2:
                if (c_nondet <= 0.1)
                    return {4, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {4, act_rew, false};
                else
                    return {7, act_rew, false};
            case 3:
                if (c_nondet <= 0.1)
                    return {7, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {2, act_rew, false};
                else
                    return {4, act_rew, false};
            default:
                break;
            }
            break;
        case 5:
            switch (action)
            {
            case 0:
                if (c_nondet <= 0.1)
                    return {5, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {6, act_rew, false};
                else
                    return {3, act_rew, false};
            case 1:
                if (c_nondet <= 0.1)
                    return {3, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {8, act_rew, false};
                else
                    return {6, act_rew, false};
            case 2:
                if (c_nondet <= 0.1)
                    return {6, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {5, act_rew, false};
                else
                    return {8, act_rew, false};
            case 3:
                if (c_nondet <= 0.1)
                    return {8, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {3, act_rew, false};
                else
                    return {5, act_rew, false};
            default:
                break;
            }
            break;
        case 6:
            switch (action)
            {
            case 0:
                if (c_nondet <= 0.1)
                    return {5, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {7, act_rew, false};
                else
                    return {6, act_rew, false};
            case 1:
                if (c_nondet <= 0.1)
                    return {6, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {9, -1, true};
                else
                    return {7, act_rew, false};
            case 2:
                if (c_nondet <= 0.1)
                    return {7, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {5, act_rew, false};
                else
                    return {9, -1, true};
            case 3:
                if (c_nondet <= 0.1)
                    return {9, -1, true};
                else if (c_nondet >= 0.9)
                    return {6, act_rew, false};
                else
                    return {5, act_rew, false};
            default:
                break;
            }
            break;
        case 7:
            switch (action)
            {
            case 0:
                if (c_nondet <= 0.1)
                    return {6, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {7, act_rew, false};
                else
                    return {4, act_rew, false};
            case 1:
                if (c_nondet <= 0.1)
                    return {4, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {10, 1, true};
                else
                    return {1, act_rew, false};
            case 2:
                if (c_nondet <= 0.1)
                    return {7, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {6, act_rew, false};
                else
                    return {10, 1, true};
            case 3:
                if (c_nondet <= 0.1)
                    return {10, 1, true};
                else if (c_nondet >= 0.9)
                    return {4, act_rew, false};
                else
                    return {6, act_rew, false};
            default:
                break;
            }
            break;
        case 8:
            switch (action)
            {
            case 0:
                if (c_nondet <= 0.1)
                    return {8, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {9, -1, true};
                else
                    return {5, act_rew, false};
            case 1:
                if (c_nondet <= 0.1)
                    return {5, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {8, act_rew, false};
                else
                    return {9, -1, true};
            case 2:
                if (c_nondet <= 0.1)
                    return {9, -1, true};
                else if (c_nondet >= 0.9)
                    return {8, act_rew, false};
                else
                    return {8, act_rew, false};
            case 3:
                if (c_nondet <= 0.1)
                    return {8, act_rew, false};
                else if (c_nondet >= 0.9)
                    return {5, act_rew, false};
                else
                    return {8, act_rew, false};
            default:
                break;
            }
            break;
        default:
            break;
        }
        return {0, act_rew, false};
    }
};

void test_ql_0()
{
    maze_agent a;

    for (size_t i = 0; i < 1000; ++i)
    {
        a.train(100);
        a.set_state(0);
        std::cout << "average reward over the evaluation step " << i << ": " << a.evaluate(0) << std::endl;
    }
    std::cout << a << std::endl;
}

int main(int, char **)
{
    test_ql_0();
}