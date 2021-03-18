#include "dql_maze.h"
#include "dql_agent.h"
#include <ostream>
#include <cassert>

using namespace rl;

void test_ql_0()
{
    const auto init_state = torch::tensor({0.0, 0.0}).detach();
    dql_maze env(init_state);
    dql_agent a(env);

    for (size_t i = 0; i < 100; ++i)
    {
        // we perform a training step..
        a.train(init_state, 100);
        // we reset the initial state..
        env.set_state(init_state);
        // we evaluate the current policy..
        std::cout << "average reward over the evaluation step " << i << ": " << a.evaluate(init_state, 100) << '\n';
    }
}

int main(int, char **)
{
    test_ql_0();
}