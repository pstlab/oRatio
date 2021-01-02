#include "dql_agent.h"
#include <sstream>
#include <cassert>

constexpr double act_rew = -0.01;

using namespace rl;

class maze_agent : public dql_agent
{
public:
    maze_agent() : dql_agent(2, 4, torch::tensor({0, 0}).detach()) {}
    ~maze_agent() {}

    std::tuple<torch::Tensor, double, bool> execute_action(const size_t &action) noexcept override
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
                        return {torch::tensor({0, 0}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({0, 1}).detach(), act_rew, false};
                    else
                        return {torch::tensor({0, 0}).detach(), act_rew, false};
                case 1:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({0, 0}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({1, 0}).detach(), act_rew, false};
                    else
                        return {torch::tensor({0, 1}).detach(), act_rew, false};
                case 2:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({0, 1}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({0, 0}).detach(), act_rew, false};
                    else
                        return {torch::tensor({1, 0}).detach(), act_rew, false};
                case 3:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({1, 0}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({0, 0}).detach(), act_rew, false};
                    else
                        return {torch::tensor({0, 0}).detach(), act_rew, false};
                default:
                    break;
                }
                break;
            case 1:
                switch (action)
                {
                case 0:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({0, 0}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({0, 2}).detach(), act_rew, false};
                    else
                        return {torch::tensor({0, 1}).detach(), act_rew, false};
                case 1:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({0, 1}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({0, 1}).detach(), act_rew, false};
                    else
                        return {torch::tensor({0, 2}).detach(), act_rew, false};
                case 2:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({0, 2}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({0, 0}).detach(), act_rew, false};
                    else
                        return {torch::tensor({0, 1}).detach(), act_rew, false};
                case 3:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({0, 1}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({0, 1}).detach(), act_rew, false};
                    else
                        return {torch::tensor({0, 0}).detach(), act_rew, false};
                default:
                    break;
                }
                break;
            case 2:
                switch (action)
                {
                case 0:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({0, 1}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({0, 2}).detach(), act_rew, false};
                    else
                        return {torch::tensor({0, 2}).detach(), act_rew, false};
                case 1:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({0, 2}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({1, 2}).detach(), act_rew, false};
                    else
                        return {torch::tensor({0, 2}).detach(), act_rew, false};
                case 2:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({0, 2}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({0, 1}).detach(), act_rew, false};
                    else
                        return {torch::tensor({1, 2}).detach(), act_rew, false};
                case 3:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({1, 2}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({0, 2}).detach(), act_rew, false};
                    else
                        return {torch::tensor({0, 1}).detach(), act_rew, false};
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
                        return {torch::tensor({1, 0}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({1, 0}).detach(), act_rew, false};
                    else
                        return {torch::tensor({0, 0}).detach(), act_rew, false};
                case 1:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({0, 0}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({2, 0}).detach(), act_rew, false};
                    else
                        return {torch::tensor({1, 0}).detach(), act_rew, false};
                case 2:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({1, 0}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({1, 0}).detach(), act_rew, false};
                    else
                        return {torch::tensor({2, 0}).detach(), act_rew, false};
                case 3:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({2, 0}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({0, 0}).detach(), act_rew, false};
                    else
                        return {torch::tensor({1, 0}).detach(), act_rew, false};
                default:
                    break;
                }
                break;
            case 2:
                switch (action)
                {
                case 0:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({1, 2}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({1, 2}).detach(), act_rew, false};
                    else
                        return {torch::tensor({0, 2}).detach(), act_rew, false};
                case 1:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({0, 2}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({2, 2}).detach(), act_rew, false};
                    else
                        return {torch::tensor({1, 2}).detach(), act_rew, false};
                case 2:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({1, 2}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({1, 2}).detach(), act_rew, false};
                    else
                        return {torch::tensor({2, 2}).detach(), act_rew, false};
                case 3:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({2, 2}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({0, 2}).detach(), act_rew, false};
                    else
                        return {torch::tensor({1, 2}).detach(), act_rew, false};
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
                        return {torch::tensor({2, 0}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({2, 1}).detach(), act_rew, false};
                    else
                        return {torch::tensor({1, 0}).detach(), act_rew, false};
                case 1:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({1, 0}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({3, 0}).detach(), act_rew, false};
                    else
                        return {torch::tensor({2, 1}).detach(), act_rew, false};
                case 2:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({2, 1}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({2, 0}).detach(), act_rew, false};
                    else
                        return {torch::tensor({3, 0}).detach(), act_rew, false};
                case 3:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({3, 0}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({1, 0}).detach(), act_rew, false};
                    else
                        return {torch::tensor({2, 0}).detach(), act_rew, false};
                default:
                    break;
                }
                break;
            case 1:
                switch (action)
                {
                case 0:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({2, 0}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({2, 2}).detach(), act_rew, false};
                    else
                        return {torch::tensor({2, 1}).detach(), act_rew, false};
                case 1:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({2, 1}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({3, 1}).detach(), -1, true};
                    else
                        return {torch::tensor({2, 2}).detach(), act_rew, false};
                case 2:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({2, 2}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({2, 0}).detach(), act_rew, false};
                    else
                        return {torch::tensor({3, 1}).detach(), -1, true};
                case 3:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({3, 1}).detach(), -1, true};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({2, 1}).detach(), act_rew, false};
                    else
                        return {torch::tensor({2, 0}).detach(), act_rew, false};
                default:
                    break;
                }
                break;
            case 2:
                switch (action)
                {
                case 0:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({2, 1}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({2, 2}).detach(), act_rew, false};
                    else
                        return {torch::tensor({1, 2}).detach(), act_rew, false};
                case 1:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({1, 2}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({3, 2}).detach(), 1, true};
                    else
                        return {torch::tensor({0, 1}).detach(), act_rew, false};
                case 2:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({2, 2}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({2, 1}).detach(), act_rew, false};
                    else
                        return {torch::tensor({3, 2}).detach(), 1, true};
                case 3:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({3, 2}).detach(), 1, true};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({1, 2}).detach(), act_rew, false};
                    else
                        return {torch::tensor({2, 1}).detach(), act_rew, false};
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
                        return {torch::tensor({3, 0}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({3, 1}).detach(), -1, true};
                    else
                        return {torch::tensor({2, 0}).detach(), act_rew, false};
                case 1:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({2, 0}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({3, 0}).detach(), act_rew, false};
                    else
                        return {torch::tensor({3, 1}).detach(), -1, true};
                case 2:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({3, 1}).detach(), -1, true};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({3, 0}).detach(), act_rew, false};
                    else
                        return {torch::tensor({3, 0}).detach(), act_rew, false};
                case 3:
                    if (c_nondet <= 0.1)
                        return {torch::tensor({3, 0}).detach(), act_rew, false};
                    else if (c_nondet >= 0.9)
                        return {torch::tensor({2, 0}).detach(), act_rew, false};
                    else
                        return {torch::tensor({3, 0}).detach(), act_rew, false};
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
        return {torch::tensor(std::vector<double>(state_dim, 0)), act_rew, false};
    }

private:
    std::default_random_engine gen;
    std::uniform_real_distribution<double> unif = std::uniform_real_distribution<double>(0, 1);
};

void test_ql_0()
{
    maze_agent a;
    for (size_t i = 0; i < a.get_buffer().get_size(); ++i)
    {
        const auto action = a.select_action();
        const auto result = a.execute_action(action);
        a.get_buffer().add(a.get_state(), action, std::get<0>(result), std::get<1>(result), std::get<2>(result));
        if (std::get<2>(result)) // we reset the initial state..
            a.set_state(torch::tensor({0, 0}).detach());
        else
            a.set_state(std::get<0>(result));
    }

    for (size_t i = 0; i < 1000; ++i)
    {
        a.train(100);
        a.set_state(torch::tensor({0, 0}).detach());
    }
}

int main(int, char **)
{
    test_ql_0();
}