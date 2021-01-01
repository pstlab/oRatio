#include "ql_agent.h"
#include <sstream>
#include <cassert>

#define DEF_REWARD -0.1

using namespace rl;

class maze_agent : public ql_agent
{
public:
    maze_agent() : ql_agent(11, 4, 0) {}
    ~maze_agent() {}

    std::pair<size_t, double> execute_action(const size_t &action) noexcept override
    {
        const auto c_nondet = unif(gen);
        switch (get_state())
        {
        case 0:
            switch (action)
            {
            case 0:
                if (c_nondet <= 0.1)
                    return {0, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {1, DEF_REWARD};
                else
                    return {0, DEF_REWARD};
            case 1:
                if (c_nondet <= 0.1)
                    return {0, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {3, DEF_REWARD};
                else
                    return {1, DEF_REWARD};
            case 2:
                if (c_nondet <= 0.1)
                    return {1, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {0, DEF_REWARD};
                else
                    return {3, DEF_REWARD};
            case 3:
                if (c_nondet <= 0.1)
                    return {3, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {0, DEF_REWARD};
                else
                    return {0, DEF_REWARD};
            default:
                break;
            }
            break;
        case 1:
            switch (action)
            {
            case 0:
                if (c_nondet <= 0.1)
                    return {0, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {2, DEF_REWARD};
                else
                    return {1, DEF_REWARD};
            case 1:
                if (c_nondet <= 0.1)
                    return {1, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {1, DEF_REWARD};
                else
                    return {2, DEF_REWARD};
            case 2:
                if (c_nondet <= 0.1)
                    return {2, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {0, DEF_REWARD};
                else
                    return {1, DEF_REWARD};
            case 3:
                if (c_nondet <= 0.1)
                    return {1, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {1, DEF_REWARD};
                else
                    return {0, DEF_REWARD};
            default:
                break;
            }
            break;
        case 2:
            switch (action)
            {
            case 0:
                if (c_nondet <= 0.1)
                    return {1, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {2, DEF_REWARD};
                else
                    return {2, DEF_REWARD};
            case 1:
                if (c_nondet <= 0.1)
                    return {2, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {4, DEF_REWARD};
                else
                    return {2, DEF_REWARD};
            case 2:
                if (c_nondet <= 0.1)
                    return {2, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {1, DEF_REWARD};
                else
                    return {4, DEF_REWARD};
            case 3:
                if (c_nondet <= 0.1)
                    return {4, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {2, DEF_REWARD};
                else
                    return {1, DEF_REWARD};
            default:
                break;
            }
            break;
        case 3:
            switch (action)
            {
            case 0:
                if (c_nondet <= 0.1)
                    return {3, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {3, DEF_REWARD};
                else
                    return {0, DEF_REWARD};
            case 1:
                if (c_nondet <= 0.1)
                    return {0, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {5, DEF_REWARD};
                else
                    return {3, DEF_REWARD};
            case 2:
                if (c_nondet <= 0.1)
                    return {3, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {3, DEF_REWARD};
                else
                    return {5, DEF_REWARD};
            case 3:
                if (c_nondet <= 0.1)
                    return {5, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {0, DEF_REWARD};
                else
                    return {3, DEF_REWARD};
            default:
                break;
            }
            break;
        case 4:
            switch (action)
            {
            case 0:
                if (c_nondet <= 0.1)
                    return {4, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {4, DEF_REWARD};
                else
                    return {2, DEF_REWARD};
            case 1:
                if (c_nondet <= 0.1)
                    return {2, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {7, DEF_REWARD};
                else
                    return {4, DEF_REWARD};
            case 2:
                if (c_nondet <= 0.1)
                    return {4, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {4, DEF_REWARD};
                else
                    return {7, DEF_REWARD};
            case 3:
                if (c_nondet <= 0.1)
                    return {7, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {2, DEF_REWARD};
                else
                    return {4, DEF_REWARD};
            default:
                break;
            }
            break;
        case 5:
            switch (action)
            {
            case 0:
                if (c_nondet <= 0.1)
                    return {5, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {6, DEF_REWARD};
                else
                    return {3, DEF_REWARD};
            case 1:
                if (c_nondet <= 0.1)
                    return {3, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {8, DEF_REWARD};
                else
                    return {6, DEF_REWARD};
            case 2:
                if (c_nondet <= 0.1)
                    return {6, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {5, DEF_REWARD};
                else
                    return {8, DEF_REWARD};
            case 3:
                if (c_nondet <= 0.1)
                    return {8, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {3, DEF_REWARD};
                else
                    return {5, DEF_REWARD};
            default:
                break;
            }
            break;
        case 6:
            switch (action)
            {
            case 0:
                if (c_nondet <= 0.1)
                    return {5, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {7, DEF_REWARD};
                else
                    return {6, DEF_REWARD};
            case 1:
                if (c_nondet <= 0.1)
                    return {6, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {9, -1};
                else
                    return {7, DEF_REWARD};
            case 2:
                if (c_nondet <= 0.1)
                    return {7, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {5, DEF_REWARD};
                else
                    return {9, -1};
            case 3:
                if (c_nondet <= 0.1)
                    return {9, -1};
                else if (c_nondet >= 0.9)
                    return {6, DEF_REWARD};
                else
                    return {5, DEF_REWARD};
            default:
                break;
            }
            break;
        case 7:
            switch (action)
            {
            case 0:
                if (c_nondet <= 0.1)
                    return {6, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {7, DEF_REWARD};
                else
                    return {4, DEF_REWARD};
            case 1:
                if (c_nondet <= 0.1)
                    return {4, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {10, 1};
                else
                    return {1, DEF_REWARD};
            case 2:
                if (c_nondet <= 0.1)
                    return {7, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {6, DEF_REWARD};
                else
                    return {10, 1};
            case 3:
                if (c_nondet <= 0.1)
                    return {10, 1};
                else if (c_nondet >= 0.9)
                    return {4, DEF_REWARD};
                else
                    return {6, DEF_REWARD};
            default:
                break;
            }
            break;
        case 8:
            switch (action)
            {
            case 0:
                if (c_nondet <= 0.1)
                    return {8, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {9, -1};
                else
                    return {5, DEF_REWARD};
            case 1:
                if (c_nondet <= 0.1)
                    return {5, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {8, DEF_REWARD};
                else
                    return {9, -1};
            case 2:
                if (c_nondet <= 0.1)
                    return {9, -1};
                else if (c_nondet >= 0.9)
                    return {8, DEF_REWARD};
                else
                    return {8, DEF_REWARD};
            case 3:
                if (c_nondet <= 0.1)
                    return {8, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {5, DEF_REWARD};
                else
                    return {8, DEF_REWARD};
            default:
                break;
            }
            break;
        case 9:
            switch (action)
            {
            case 0:
                if (c_nondet <= 0.1)
                    return {8, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {10, 1};
                else
                    return {6, DEF_REWARD};
            case 1:
                if (c_nondet <= 0.1)
                    return {6, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {9, -1};
                else
                    return {10, 1};
            case 2:
                if (c_nondet <= 0.1)
                    return {10, 1};
                else if (c_nondet >= 0.9)
                    return {8, DEF_REWARD};
                else
                    return {9, -1};
            case 3:
                if (c_nondet <= 0.1)
                    return {9, -1};
                else if (c_nondet >= 0.9)
                    return {6, DEF_REWARD};
                else
                    return {8, DEF_REWARD};
            default:
                break;
            }
            break;
        case 10:
            switch (action)
            {
            case 0:
                if (c_nondet <= 0.1)
                    return {9, -1};
                else if (c_nondet >= 0.9)
                    return {10, 1};
                else
                    return {7, DEF_REWARD};
            case 1:
                if (c_nondet <= 0.1)
                    return {7, DEF_REWARD};
                else if (c_nondet >= 0.9)
                    return {10, 1};
                else
                    return {10, 1};
            case 2:
                if (c_nondet <= 0.1)
                    return {10, 1};
                else if (c_nondet >= 0.9)
                    return {9, -1};
                else
                    return {10, 1};
            case 3:
                if (c_nondet <= 0.1)
                    return {10, 1};
                else if (c_nondet >= 0.9)
                    return {7, DEF_REWARD};
                else
                    return {9, -1};
            default:
                break;
            }
        default:
            break;
        }
        return {0, DEF_REWARD};
    }
};

void test_ql_0()
{
    maze_agent a;

    for (size_t i = 0; i < 1000; ++i)
    {
        a.train(100);
        a.set_state(0);
    }
    std::cout << a << std::endl;
}

int main(int, char **)
{
    test_ql_0();
}