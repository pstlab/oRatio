#include "solver.h"
#include <iostream>

int main(int argc, char *argv[])
{
    using namespace ratio;

    // the problem files..
    std::vector<std::string> prob_names;
    for (int i = 1; i < argc - 1; i++)
        prob_names.push_back(argv[i]);

    // the solution file..
    std::string sol_name = argv[argc - 1];

    std::cout << "starting oRatio";
#ifndef NDEBUG
    std::cout << " in debug mode";
#endif
    std::cout << ".." << std::endl;

    solver s;

    s.init();

    try
    {
        std::cout << "parsing input files.." << std::endl;
        s.read(prob_names);

        std::cout << "solving the problem.." << std::endl;
        s.solve();
        std::cout << "hurray!! we have found a solution.." << std::endl;
    }
    catch (const std::exception &ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
}