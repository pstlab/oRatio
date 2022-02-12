#include "gui_server.h"

int main(int argc, char const *argv[])
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

    ratio::gui_server srv;
    srv.start();

    try
    {
        std::cout << "parsing input files..\n";
        srv.get_solver().read(prob_names);

        std::cout << "solving the problem..\n";
        srv.get_solver().solve();
        std::cout << "hurray!! we have found a solution..\n";

        std::ofstream sol_file;
        sol_file.open(sol_name);
        sol_file << srv.get_solver();
        sol_file.close();
    }
    catch (const std::exception &ex)
    {
        std::cout << ex.what() << '\n';
    }
    return 0;
}
