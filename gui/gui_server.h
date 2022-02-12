#pragma once

#include "core_listener.h"
#include "solver_listener.h"
#include "executor_listener.h"
#include <crow.h>
#include <mutex>

namespace ratio
{

  class gui_server : public core_listener, public solver_listener, public executor_listener
  {
  public:
    gui_server();
    ~gui_server();

    solver &get_solver() { return slv; }
    executor &get_executor() { return exec; }

    void start();

  private:
    void log(const std::string &msg) override;
    void read(const std::string &script) override;
    void read(const std::vector<std::string> &files) override;

    void state_changed() override;

    void started_solving() override;
    void solution_found() override;
    void inconsistent_problem() override;

  private:
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
    void tick(const smt::rational &time) override;
    void starting(const std::unordered_set<atom *> &atoms) override;
    void start(const std::unordered_set<atom *> &atoms) override;
    void ending(const std::unordered_set<atom *> &atoms) override;
    void end(const std::unordered_set<atom *> &atoms) override;

  private:
    solver slv;
    executor exec;
    crow::SimpleApp app;
    std::mutex mtx;
  };
} // namespace ratio