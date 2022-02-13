#include "gui_server.h"

namespace ratio
{
    gui_server::gui_server(executor &exec, const std::string &host, const unsigned short port) : exec(exec), host(host), port(port), core_listener(exec.get_solver()), solver_listener(exec.get_solver()), executor_listener(exec)
    {
        CROW_ROUTE(app, "/")
        ([]()
         { 
        crow::mustache::context ctx;
        ctx["name"] = "oRatio";
        return crow::mustache::load("index.html").render(ctx); });

        CROW_ROUTE(app, "/ws")
            .websocket()
            .onopen([&](crow::websocket::connection &conn)
                    { std::lock_guard<std::mutex> _(mtx); })
            .onclose([&](crow::websocket::connection &conn, const std::string &reason)
                     { std::lock_guard<std::mutex> _(mtx); })
            .onmessage([&](crow::websocket::connection &conn, const std::string &data, bool is_binary)
                       { std::lock_guard<std::mutex> _(mtx); });
    }
    gui_server::~gui_server() { std::lock_guard<std::mutex> _(mtx); }

    void gui_server::start() { app.bindaddr(host).port(port).run(); }
    void gui_server::wait_for_server_start() { app.wait_for_server_start(); }
    void gui_server::stop() { app.stop(); }

    void gui_server::log(const std::string &msg) { std::lock_guard<std::mutex> _(mtx); }
    void gui_server::read(const std::string &script) { std::lock_guard<std::mutex> _(mtx); }
    void gui_server::read(const std::vector<std::string> &files) { std::lock_guard<std::mutex> _(mtx); }

    void gui_server::state_changed() { std::lock_guard<std::mutex> _(mtx); }

    void gui_server::started_solving() { std::lock_guard<std::mutex> _(mtx); }
    void gui_server::solution_found() { std::lock_guard<std::mutex> _(mtx); }
    void gui_server::inconsistent_problem() { std::lock_guard<std::mutex> _(mtx); }

    void gui_server::flaw_created(const flaw &f) { std::lock_guard<std::mutex> _(mtx); }
    void gui_server::flaw_state_changed(const flaw &f) { std::lock_guard<std::mutex> _(mtx); }
    void gui_server::flaw_cost_changed(const flaw &f) { std::lock_guard<std::mutex> _(mtx); }
    void gui_server::flaw_position_changed(const flaw &f) { std::lock_guard<std::mutex> _(mtx); }
    void gui_server::current_flaw(const flaw &f) { std::lock_guard<std::mutex> _(mtx); }

    void gui_server::resolver_created(const resolver &r) { std::lock_guard<std::mutex> _(mtx); }
    void gui_server::resolver_state_changed(const resolver &r) { std::lock_guard<std::mutex> _(mtx); }
    void gui_server::current_resolver(const resolver &r) { std::lock_guard<std::mutex> _(mtx); }

    void gui_server::causal_link_added(const flaw &f, const resolver &r) { std::lock_guard<std::mutex> _(mtx); }

    void gui_server::tick(const smt::rational &time) { std::lock_guard<std::mutex> _(mtx); }
    void gui_server::starting(const std::unordered_set<atom *> &atoms) { std::lock_guard<std::mutex> _(mtx); }
    void gui_server::start(const std::unordered_set<atom *> &atoms) { std::lock_guard<std::mutex> _(mtx); }
    void gui_server::ending(const std::unordered_set<atom *> &atoms) { std::lock_guard<std::mutex> _(mtx); }
    void gui_server::end(const std::unordered_set<atom *> &atoms) { std::lock_guard<std::mutex> _(mtx); }
} // namespace ratio
