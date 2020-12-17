#include "solver.h"
#include "executor_listener.h"
#include "atom.h"
#ifdef BUILD_GUI
#include "socket_listener.h"
#endif

using namespace ratio;

class simple_executor : public executor_listener
{
public:
    simple_executor(executor &e) : executor_listener(e) {}
    ~simple_executor() {}

    void starting(const std::set<atom *> &atoms) override
    {
        LOG("starting atoms:\n");
        for (const auto &atm : atoms)
            LOG(atm->to_json() << '\n');
    }
    void ending(const std::set<atom *> &atoms) override
    {
        LOG("ending atoms:\n");
        for (const auto &atm : atoms)
            LOG(atm->to_json() << '\n');
    }
};

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
#ifdef BUILD_GUI
    socket_listener l(s, HOST, PORT);
#endif

    s.init();
    executor exec(s, 5000);
    try
    {
        std::cout << "parsing input files.." << std::endl;
        s.read(prob_names);

        std::cout << "solving the problem.." << std::endl;
        s.solve();
        std::cout << "hurray!! we have found a solution.." << std::endl;

        std::thread exec_th = exec.start();
        exec_th.join();
    }
    catch (const std::exception &ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
}