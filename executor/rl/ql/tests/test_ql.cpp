#include "ql_maze.h"
#include "ql_agent.h"
#include <iostream>
#include <cassert>

using namespace rl;

void test_ql_0()
{
    const size_t init_state = 0;
    ql_maze env(0);
    ql_agent a(env);

    for (size_t i = 0; i < 1000; ++i)
    {
        // we perform a training step..
        a.train(100);
        // we reset the initial state..
        env.set_state(init_state);
        // we evaluate the current policy..
        std::cout << "average reward over the evaluation step " << i << ": " << a.evaluate(0, 100) << std::endl;
    }
    std::cout << a << std::endl;
}

int main(int, char **)
{
    test_ql_0();
}