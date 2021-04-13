#include "solver.h"
#include "predicate.h"
#include "field.h"
#include "riddle_lexer.h"
#include <iostream>
#include <fstream>

using namespace ratio;

void to_ros_msg(const predicate &pred)
{
    std::ofstream msg_file;
    msg_file.open(pred.get_name() + ".msg");
    msg_file << "uint64 id\n\n";
    std::unordered_set<const type *> all_preds;
    std::queue<const type *> q;
    q.push(&pred);
    while (!q.empty())
    {
        const type &c_pred = *q.front();
        q.pop();
        for (const auto &arg : pred.get_args())
        {
            if (arg->get_type().get_name().compare(BOOL_KEYWORD) == 0)
                msg_file << "bool " << arg->get_name() << "\n";
            else if (arg->get_type().get_name().compare(INT_KEYWORD) == 0)
                msg_file << "int64 " << arg->get_name() << "\n";
            else if (arg->get_type().get_name().compare(REAL_KEYWORD) == 0 || arg->get_type().get_name().compare(TP_KEYWORD) == 0)
                msg_file << "float64 " << arg->get_name() << "\n";
            else if (arg->get_type().get_name().compare(STRING_KEYWORD) == 0)
                msg_file << "string " << arg->get_name() << "\n";
            else
                msg_file << "string " << arg->get_name() << "\n";
        }
        for (const auto &t : pred.get_supertypes())
            if (all_preds.insert(t).second)
                q.push(t);
    }

    msg_file.close();
}

int main(int argc, char const *argv[])
{
    // the problem files..
    std::vector<std::string> prob_names;
    for (int i = 1; i < argc - 1; i++)
        prob_names.push_back(argv[i]);

    solver s;

    try
    {
        s.init();

        std::cout << "parsing input files..\n";
        s.read(prob_names);

        for (const auto &p : s.get_predicates())
            to_ros_msg(*p.second);
        std::queue<type *> q;
        for (const auto &t : s.get_types())
            if (!t.second->is_primitive())
                q.push(t.second);
        while (!q.empty())
        {
            const type &t = *q.front();
            q.pop();
            for (const auto &p : t.get_predicates())
                to_ros_msg(*p.second);
            for (const auto &t : t.get_types())
                q.push(t.second);
        }

        return 0;
    }
    catch (const std::exception &ex)
    {
        std::cout << ex.what() << '\n';
        return 1;
    }
}
