#include "solver.h"
#ifndef NDEBUG
#include "socket_listener.h"
#endif
#include <iostream>
#include <fstream>

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
#ifndef NDEBUG
    socket_listener l(s);
#endif

    s.init();

    try
    {
        std::cout << "parsing input files.." << std::endl;
        s.read(prob_names);

        std::cout << "solving the problem.." << std::endl;
        s.solve();
        std::cout << "hurray!! we have found a solution.." << std::endl;

#ifdef STATISTICS
        std::cout << "Created flaws:" << std::endl;
        std::cout << " - facts:           " << s.nr_created_facts() << std::endl;
        std::cout << " - goals:           " << s.nr_created_goals() << std::endl;
        std::cout << " - disjunctions:    " << s.nr_created_disjs() << std::endl;
        std::cout << " - inconsistencies: " << s.nr_created_incs() << std::endl;
        std::cout << "Solved flaws:" << std::endl;
        std::cout << " - facts:           " << s.nr_solved_facts() << std::endl;
        std::cout << " - goals:           " << s.nr_solved_goals() << std::endl;
        std::cout << " - disjunctions:    " << s.nr_solved_disjs() << std::endl;
        std::cout << " - inconsistencies: " << s.nr_solved_incs() << std::endl;
#endif

        std::ofstream sol_file;
        sol_file.open(sol_name);
        sol_file << s.to_string();
        sol_file.close();
    }
    catch (const std::exception &ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
}