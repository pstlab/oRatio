#include "solver.h"
#include "executor_listener.h"
#include "atom.h"
#ifdef BUILD_GUI
#include "solver_socket_listener.h"
#include "executor_socket_listener.h"
#endif

using namespace ratio;

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        std::cerr << "usage: oRatio <input-file> [<input-file> ...] <output-file>" << std::endl;
        return -1;
    }

    // the problem files..
    std::vector<std::string> prob_names;
    for (int i = 1; i < argc - 1; i++)
        prob_names.push_back(argv[i]);

    // the solution file..
    std::string sol_name = argv[argc - 1];

    std::cout << "starting oRatio";
#ifdef BUILD_GUI
    std::cout << " in debug mode";
#endif
    std::cout << ".." << std::endl;

    solver s;
    executor exec(s, 5000);

#ifdef BUILD_GUI
    solver_socket_listener sl(s, HOST, SOLVER_PORT);
    executor_socket_listener el(exec, HOST, EXECUTOR_PORT);
#endif

    s.init();
    try
    {
        std::cout << "parsing input files.." << std::endl;
        s.read(prob_names);

        std::cout << "solving the problem.." << std::endl;
        s.solve();
        std::cout << "hurray!! we have found a solution.." << std::endl;

        std::thread exec_th = exec.start();

        // we wait for the execution to finish..
        exec_th.join();
    }
    catch (const std::exception &ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
}