#include "py_solver.h"
#include "py_core_listener.h"

namespace ratio
{
    py_solver::py_solver() : slv(new solver(false)), core_listener(*slv), solver_listener(*slv) {}
    py_solver::~py_solver() { delete slv; }

    void py_solver::read_script(const py::str &script) { slv->read(script); }
    void py_solver::read_files(const py::list &files)
    {
        std::vector<std::string> c_files;
        for (const auto &f : files)
            c_files.push_back(py::str(f));
        slv->read(c_files);
    }
    void py_solver::solve() { slv->solve(); }

    void py_solver::add_core_listener(py_core_listener &listener) { core_listeners.push_back(&listener); }
    void py_solver::remove_core_listener(py_core_listener &listener) { core_listeners.erase(std::find(core_listeners.cbegin(), core_listeners.cend(), &listener)); }
    void py_solver::add_solver_listener(py_solver_listener &listener) { solver_listeners.push_back(&listener); }
    void py_solver::remove_solver_listener(py_solver_listener &listener) { solver_listeners.erase(std::find(solver_listeners.cbegin(), solver_listeners.cend(), &listener)); }

    void py_solver::log(const std::string &msg)
    {
        for (const auto &l : core_listeners)
            l->log(msg);
    }
    void py_solver::read(const std::string &script)
    {
        for (const auto &l : core_listeners)
            l->read_script(script);
    }
    void py_solver::read(const std::vector<std::string> &files)
    {
        py::list fs;
        for (const auto &f : files)
            fs.append(f);
        for (const auto &l : core_listeners)
            l->read_files(fs);
    }

    void py_solver::state_changed()
    {
        for (const auto &l : core_listeners)
            l->state_changed();
    }

    void py_solver::started_solving()
    {
        for (const auto &l : core_listeners)
            l->started_solving();
    }
    void py_solver::solution_found()
    {
        for (const auto &l : core_listeners)
            l->solution_found();
    }
    void py_solver::inconsistent_problem()
    {
        for (const auto &l : core_listeners)
            l->inconsistent_problem();
    }

    void py_solver::flaw_created(const flaw &f)
    {
        py::object py_f;
        py_f["id"] = f.get_id();
        py::list py_causes;
        for (const auto &c : f.get_causes())
            py_causes.append(c->get_id());
        py_f["causes"] = py_causes;
        py_f["label"] = f.get_data();
        py_f["state"] = slv->get_sat_core().value(f.get_phi());

        const auto [lb, ub] = slv->get_idl_theory().bounds(f.get_position());
        py::object py_pos;
        py_pos["lb"] = lb;
        py_pos["ub"] = ub;
        py_f["pos"] = py_pos;

        flaws.emplace(&f, py_f);
    }
    void py_solver::flaw_state_changed(const flaw &f)
    {
        py::object &py_f = flaws.find(&f)->second;
        py_f["state"] = slv->get_sat_core().value(f.get_phi());
    }
    void py_solver::flaw_cost_changed(const flaw &f) {}
    void py_solver::flaw_position_changed(const flaw &f) {}
    void py_solver::current_flaw(const flaw &f) {}

    void py_solver::resolver_created(const resolver &r) {}
    void py_solver::resolver_state_changed(const resolver &r) {}
    void py_solver::current_resolver(const resolver &r) {}

    void py_solver::causal_link_added(const flaw &f, const resolver &r) {}
} // namespace ratio
