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
    gui_server(executor &exec, const std::string &host = "127.0.0.1", const unsigned short port = 8080);
    ~gui_server();

    solver &get_solver() { return slv; }
    executor &get_executor() { return exec; }

    void start();
    void wait_for_server_start();
    void stop();

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
    static smt::json to_json(const flaw &f);
    static smt::json to_json(const resolver &r);

  private:
    executor &exec;
    std::unordered_set<const flaw *> flaws;
    const flaw *c_flaw = nullptr;
    std::unordered_set<const resolver *> resolvers;
    const resolver *c_resolver = nullptr;
    const std::string host;
    const unsigned short port;
    crow::SimpleApp app;
    std::unordered_set<crow::websocket::connection *> users;
    std::mutex mtx;
  };
} // namespace ratio