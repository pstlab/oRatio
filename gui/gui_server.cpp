#include "gui_server.h"

namespace ratio
{
    gui_server::gui_server() : slv(), exec(slv), core_listener(slv), solver_listener(slv), executor_listener(exec)
    {
    }
    gui_server::~gui_server() {}

    void gui_server::start() {}

    void gui_server::log(const std::string &msg) {}
    void gui_server::read(const std::string &script) {}
    void gui_server::read(const std::vector<std::string> &files) {}

    void gui_server::state_changed() {}

    void gui_server::started_solving() {}
    void gui_server::solution_found() {}
    void gui_server::inconsistent_problem() {}

    void gui_server::flaw_created(const flaw &f) {}
    void gui_server::flaw_state_changed(const flaw &f) {}
    void gui_server::flaw_cost_changed(const flaw &f) {}
    void gui_server::flaw_position_changed(const flaw &f) {}
    void gui_server::current_flaw(const flaw &f) {}

    void gui_server::resolver_created(const resolver &r) {}
    void gui_server::resolver_state_changed(const resolver &r) {}
    void gui_server::current_resolver(const resolver &r) {}

    void gui_server::causal_link_added(const flaw &f, const resolver &r) {}

    void gui_server::tick(const smt::rational &time) {}
    void gui_server::starting(const std::unordered_set<atom *> &atoms) {}
    void gui_server::start(const std::unordered_set<atom *> &atoms) {}
    void gui_server::ending(const std::unordered_set<atom *> &atoms) {}
    void gui_server::end(const std::unordered_set<atom *> &atoms) {}
} // namespace ratio
