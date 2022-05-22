#pragma once

#include <pybind11/pybind11.h>
#include "solver.h"
#include "core_listener.h"
#include "solver_listener.h"

namespace py = pybind11;

namespace ratio
{
  class py_core_listener;
  class py_solver_listener;

  class py_solver : public core_listener, public solver_listener
  {
  public:
    py_solver();
    ~py_solver();

    void read_script(const py::str &script);
    void read_files(const py::list &files);
    void solve();

    void add_core_listener(py_core_listener &listener);
    void remove_core_listener(py_core_listener &listener);
    void add_solver_listener(py_solver_listener &listener);
    void remove_solver_listener(py_solver_listener &listener);

  private:
    void log(const std::string &) override;
    void read(const std::string &) override;
    void read(const std::vector<std::string> &) override;

    void state_changed() override;

    void started_solving() override;
    void solution_found() override;
    void inconsistent_problem() override;

    void flaw_created(const flaw &f) override;
    void flaw_state_changed(const flaw &f) override;
    void flaw_cost_changed(const flaw &f) override;
    void flaw_position_changed(const flaw &f) override;
    void current_flaw(const flaw &f) override;

    void resolver_created(const resolver &r) override;
    void resolver_state_changed(const resolver &r) override;
    void current_resolver(const resolver &r) override;

    void causal_link_added(const flaw &f, const resolver &r) override;

  private:
    solver *slv;
    std::vector<py_core_listener *> core_listeners;
    std::vector<py_solver_listener *> solver_listeners;

    std::unordered_map<const type *, py::object> types;
    std::unordered_map<const item *, py::object> items;
    std::unordered_map<const flaw *, py::object> flaws;
    std::unordered_map<const resolver *, py::object> resolvers;
  };
} // namespace ratio
