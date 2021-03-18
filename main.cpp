#include "solver.h"
#include <iostream>
#include <fstream>

using namespace ratio;

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        std::cerr << "usage: oRatio <input-file> [<input-file> ...] <output-file>\n";
        return -1;
    }

    // the problem files..
    std::vector<std::string> prob_names;
    for (int i = 1; i < argc - 1; i++)
        prob_names.push_back(argv[i]);

    // the solution file..
    std::string sol_name = argv[argc - 1];

#ifdef NDEBUG
    if (std::ifstream(sol_name).good())
    {
        std::cout << "The solution file '" << sol_name << "' already exists! Please, specify a different solution file..";
        return -1;
    }
#endif

    std::cout << "starting oRatio";
#ifndef NDEBUG
    std::cout << " in debug mode";
#endif
    std::cout << "..\n";

    solver s;

    try
    {
        s.init();

        std::cout << "parsing input files..\n";
        s.read(prob_names);

        std::cout << "solving the problem..\n";
        s.solve();
        std::cout << "hurray!! we have found a solution..\n";

        std::ofstream sol_file;
        sol_file.open(sol_name);
        sol_file << s;
        sol_file.close();
    }
    catch (const std::exception &ex)
    {
        std::cout << ex.what() << '\n';
        return 1;
    }
}
