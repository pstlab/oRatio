#include "gui_server.h"
#include "json.h"

namespace ratio
{
    gui_server::gui_server(executor &exec, const std::string &host, const unsigned short port) : exec(exec), host(host), port(port), core_listener(exec.get_solver()), solver_listener(exec.get_solver()), executor_listener(exec)
    {
        CROW_ROUTE(app, "/")
        ([]()
         {
            crow::mustache::context ctx;
            ctx["title"] = "oRatio";
            return crow::mustache::load("index.html").render(ctx); });

        CROW_ROUTE(app, "/timelines")
        ([&]()
         {
            crow::mustache::context ctx;
            ctx["title"] = "oRatio";
            return crow::mustache::load("timelines.html").render(ctx); });

        CROW_ROUTE(app, "/graph")
        ([&]()
         {
            crow::mustache::context ctx;
            ctx["title"] = "oRatio";
            return crow::mustache::load("graph.html").render(ctx); });

        CROW_ROUTE(app, "/ws-solver")
            .websocket()
            .onopen([&](crow::websocket::connection &conn)
                    { std::lock_guard<std::mutex> _(mtx);
                users.insert(&conn);

                smt::json j_sc;
                j_sc->set("type", new smt::string_val("state_changed"));
                j_sc->set("state", slv.to_json());
                j_sc->set("timelines", slv.extract_timelines());

                std::stringstream ss;
                j_sc.to_json(ss);
                conn.send_text(ss.str());
                ss.clear();

                smt::json j_gr;
                j_gr->set("type", new smt::string_val("graph"));
                std::vector<smt::json> j_flaws;
                for (const auto &f : flaws)
                    j_flaws.push_back(to_json(*f));
                j_gr->set("flaws", new smt::array_val(j_flaws));
                if (c_flaw)
                    j_gr->set("current-flaw", new smt::long_val(reinterpret_cast<std::uintptr_t>(c_flaw)));
                std::vector<smt::json> j_resolvers;
                for (const auto &r : resolvers)
                    j_resolvers.push_back(to_json(*r));
                j_gr->set("resolvers", new smt::array_val(j_resolvers));
                if (c_resolver)
                    j_gr->set("current-resolver", new smt::long_val(reinterpret_cast<std::uintptr_t>(c_resolver)));

                j_gr.to_json(ss);
                conn.send_text(ss.str()); })
            .onclose([&](crow::websocket::connection &conn, const std::string &reason)
                     { std::lock_guard<std::mutex> _(mtx); users.erase(&conn); });

        CROW_ROUTE(app, "/ws-timelines")
            .websocket()
            .onopen([&](crow::websocket::connection &conn)
                    { std::lock_guard<std::mutex> _(mtx);
                timelines_users.insert(&conn);

                smt::json j_sc;
                j_sc->set("type", new smt::string_val("state_changed"));
                j_sc->set("state", slv.to_json());
                j_sc->set("timelines", slv.extract_timelines());

                std::stringstream ss;
                j_sc.to_json(ss);
                conn.send_text(ss.str()); })
            .onclose([&](crow::websocket::connection &conn, const std::string &reason)
                     { std::lock_guard<std::mutex> _(mtx); timelines_users.erase(&conn); });

        CROW_ROUTE(app, "/ws-graph")
            .websocket()
            .onopen([&](crow::websocket::connection &conn)
                    { std::lock_guard<std::mutex> _(mtx);
                graph_users.insert(&conn);
                
                smt::json j_gr;
                j_gr->set("type", new smt::string_val("graph"));
                std::vector<smt::json> j_flaws;
                for (const auto &f : flaws)
                    j_flaws.push_back(to_json(*f));
                j_gr->set("flaws", new smt::array_val(j_flaws));
                if (c_flaw)
                    j_gr->set("current-flaw", new smt::long_val(reinterpret_cast<std::uintptr_t>(c_flaw)));
                std::vector<smt::json> j_resolvers;
                for (const auto &r : resolvers)
                    j_resolvers.push_back(to_json(*r));
                j_gr->set("resolvers", new smt::array_val(j_resolvers));
                if (c_resolver)
                    j_gr->set("current-resolver", new smt::long_val(reinterpret_cast<std::uintptr_t>(c_resolver)));

                std::stringstream ss;
                j_gr.to_json(ss);
                conn.send_text(ss.str()); })
            .onclose([&](crow::websocket::connection &conn, const std::string &reason)
                     { std::lock_guard<std::mutex> _(mtx); graph_users.erase(&conn); });
    }
    gui_server::~gui_server() {}

    void gui_server::start() { app.bindaddr(host).port(port).run(); }
    void gui_server::wait_for_server_start() { app.wait_for_server_start(); }
    void gui_server::stop() { app.stop(); }

    void gui_server::log(const std::string &msg) { std::lock_guard<std::mutex> _(mtx); }
    void gui_server::read(const std::string &script) { std::lock_guard<std::mutex> _(mtx); }
    void gui_server::read(const std::vector<std::string> &files) { std::lock_guard<std::mutex> _(mtx); }

    void gui_server::state_changed()
    {
        std::lock_guard<std::mutex> _(mtx);

        smt::json j_sc;
        j_sc->set("type", new smt::string_val("state_changed"));
        j_sc->set("state", slv.to_json());
        j_sc->set("timelines", slv.extract_timelines());

        std::stringstream ss;
        j_sc.to_json(ss);
        for (const auto &u : users)
            u->send_text(ss.str());
    }

    void gui_server::started_solving()
    {
        std::lock_guard<std::mutex> _(mtx);

        smt::json j_ss;
        j_ss->set("type", new smt::string_val("started_solving"));

        std::stringstream ss;
        j_ss.to_json(ss);
        for (const auto &u : users)
            u->send_text(ss.str());
    }
    void gui_server::solution_found()
    {
        std::lock_guard<std::mutex> _(mtx);

        smt::json j_sf;
        j_sf->set("type", new smt::string_val("solution_found"));
        j_sf->set("state", slv.to_json());
        j_sf->set("timelines", slv.extract_timelines());

        std::stringstream ss;
        j_sf.to_json(ss);
        for (const auto &u : users)
            u->send_text(ss.str());
    }
    void gui_server::inconsistent_problem()
    {
        std::lock_guard<std::mutex> _(mtx);

        smt::json j_ip;
        j_ip->set("type", new smt::string_val("inconsistent_problem"));

        std::stringstream ss;
        j_ip.to_json(ss);
        for (const auto &u : users)
            u->send_text(ss.str());
    }

    void gui_server::flaw_created(const flaw &f)
    {
        std::lock_guard<std::mutex> _(mtx);
        flaws.insert(&f);

        smt::json j_fc = to_json(f);
        j_fc->set("type", new smt::string_val("flaw_created"));

        std::stringstream ss;
        j_fc.to_json(ss);
        for (const auto &u : users)
            u->send_text(ss.str());
    }
    void gui_server::flaw_state_changed(const flaw &f)
    {
        std::lock_guard<std::mutex> _(mtx);

        smt::json j_fsc;
        j_fsc->set("type", new smt::string_val("flaw_state_changed"));
        j_fsc->set("id", new smt::long_val(reinterpret_cast<std::uintptr_t>(&f)));
        j_fsc->set("state", new smt::long_val(slv.get_sat_core().value(f.get_phi())));

        std::stringstream ss;
        j_fsc.to_json(ss);
        for (const auto &u : users)
            u->send_text(ss.str());
    }
    void gui_server::flaw_cost_changed(const flaw &f)
    {
        std::lock_guard<std::mutex> _(mtx);

        smt::json j_cc;
        j_cc->set("type", new smt::string_val("flaw_cost_changed"));
        j_cc->set("id", new smt::long_val(reinterpret_cast<std::uintptr_t>(&f)));
        const auto est_cost = f.get_estimated_cost();
        smt::json j_cost;
        j_cost->set("num", new smt::long_val(est_cost.numerator()));
        j_cost->set("den", new smt::long_val(est_cost.denominator()));
        j_cc->set("cost", j_cost);

        std::stringstream ss;
        j_cc.to_json(ss);
        for (const auto &u : users)
            u->send_text(ss.str());
    }
    void gui_server::flaw_position_changed(const flaw &f)
    {
        std::lock_guard<std::mutex> _(mtx);

        smt::json j_fpc;
        j_fpc->set("type", new smt::string_val("flaw_position_changed"));
        j_fpc->set("id", new smt::long_val(reinterpret_cast<std::uintptr_t>(&f)));
        const auto [lb, ub] = slv.get_idl_theory().bounds(f.get_position());
        smt::json j_pos;
        j_pos->set("lb", new smt::long_val(lb));
        j_pos->set("ub", new smt::long_val(ub));
        j_fpc->set("pos", j_pos);

        std::stringstream ss;
        j_fpc.to_json(ss);
        for (const auto &u : users)
            u->send_text(ss.str());
    }
    void gui_server::current_flaw(const flaw &f)
    {
        std::lock_guard<std::mutex> _(mtx);

        smt::json j_cf;
        j_cf->set("type", new smt::string_val("current_flaw"));
        j_cf->set("id", new smt::long_val(reinterpret_cast<std::uintptr_t>(&f)));

        std::stringstream ss;
        j_cf.to_json(ss);
        for (const auto &u : users)
            u->send_text(ss.str());
    }

    void gui_server::resolver_created(const resolver &r)
    {
        std::lock_guard<std::mutex> _(mtx);
        resolvers.insert(&r);

        smt::json j_rc = to_json(r);
        j_rc->set("type", new smt::string_val("resolver_created"));

        std::stringstream ss;
        j_rc.to_json(ss);
        for (const auto &u : users)
            u->send_text(ss.str());
    }
    void gui_server::resolver_state_changed(const resolver &r)
    {
        std::lock_guard<std::mutex> _(mtx);

        smt::json j_rsc;
        j_rsc->set("type", new smt::string_val("resolver_state_changed"));
        j_rsc->set("id", new smt::long_val(reinterpret_cast<std::uintptr_t>(&r)));
        j_rsc->set("state", new smt::long_val(slv.get_sat_core().value(r.get_rho())));

        std::stringstream ss;
        j_rsc.to_json(ss);
        for (const auto &u : users)
            u->send_text(ss.str());
    }
    void gui_server::current_resolver(const resolver &r)
    {
        std::lock_guard<std::mutex> _(mtx);

        smt::json j_cr;
        j_cr->set("type", new smt::string_val("current_resolver"));
        j_cr->set("id", new smt::long_val(reinterpret_cast<std::uintptr_t>(&r)));

        std::stringstream ss;
        j_cr.to_json(ss);
        for (const auto &u : users)
            u->send_text(ss.str());
    }

    void gui_server::causal_link_added(const flaw &f, const resolver &r)
    {
        std::lock_guard<std::mutex> _(mtx);

        smt::json j_cla;
        j_cla->set("type", new smt::string_val("causal_link_added"));
        j_cla->set("flaw_id", new smt::long_val(reinterpret_cast<std::uintptr_t>(&f)));
        j_cla->set("resolver_id", new smt::long_val(reinterpret_cast<std::uintptr_t>(&r)));

        std::stringstream ss;
        j_cla.to_json(ss);
        for (const auto &u : users)
            u->send_text(ss.str());
    }

    void gui_server::tick(const smt::rational &time)
    {
        std::lock_guard<std::mutex> _(mtx);

        smt::json j_t;
        j_t->set("type", new smt::string_val("tick"));
        smt::json j_time;
        j_time->set("num", new smt::long_val(time.numerator()));
        j_time->set("den", new smt::long_val(time.denominator()));
        j_t->set("time", j_time);

        std::stringstream ss;
        j_t.to_json(ss);
        for (const auto &u : users)
            u->send_text(ss.str());
    }
    void gui_server::starting(const std::unordered_set<atom *> &atoms)
    {
        std::lock_guard<std::mutex> _(mtx);

        smt::json j_st;
        j_st->set("type", new smt::string_val("starting"));
        std::vector<smt::json> causes;
        for (const auto &a : atoms)
            causes.push_back(new smt::long_val(reinterpret_cast<std::uintptr_t>(a)));
        j_st->set("starting", new smt::array_val(causes));

        std::stringstream ss;
        j_st.to_json(ss);
        for (const auto &u : users)
            u->send_text(ss.str());
    }
    void gui_server::start(const std::unordered_set<atom *> &atoms)
    {
        std::lock_guard<std::mutex> _(mtx);

        smt::json j_st;
        j_st->set("type", new smt::string_val("start"));
        std::vector<smt::json> causes;
        for (const auto &a : atoms)
            causes.push_back(new smt::long_val(reinterpret_cast<std::uintptr_t>(a)));
        j_st->set("start", new smt::array_val(causes));

        std::stringstream ss;
        j_st.to_json(ss);
        for (const auto &u : users)
            u->send_text(ss.str());
    }
    void gui_server::ending(const std::unordered_set<atom *> &atoms)
    {
        std::lock_guard<std::mutex> _(mtx);

        smt::json j_st;
        j_st->set("type", new smt::string_val("ending"));
        std::vector<smt::json> causes;
        for (const auto &a : atoms)
            causes.push_back(new smt::long_val(reinterpret_cast<std::uintptr_t>(a)));
        j_st->set("ending", new smt::array_val(causes));

        std::stringstream ss;
        j_st.to_json(ss);
        for (const auto &u : users)
            u->send_text(ss.str());
    }
    void gui_server::end(const std::unordered_set<atom *> &atoms)
    {
        std::lock_guard<std::mutex> _(mtx);

        smt::json j_st;
        j_st->set("type", new smt::string_val("end"));
        std::vector<smt::json> causes;
        for (const auto &a : atoms)
            causes.push_back(new smt::long_val(reinterpret_cast<std::uintptr_t>(a)));
        j_st->set("end", new smt::array_val(causes));

        std::stringstream ss;
        j_st.to_json(ss);
        for (const auto &u : users)
            u->send_text(ss.str());
    }

    smt::json gui_server::to_json(const flaw &f)
    {
        smt::json j_f;
        j_f->set("id", new smt::long_val(reinterpret_cast<std::uintptr_t>(&f)));
        std::vector<smt::json> causes;
        for (const auto &c : f.get_causes())
            causes.push_back(new smt::long_val(reinterpret_cast<std::uintptr_t>(c)));
        j_f->set("causes", new smt::array_val(causes));
        j_f->set("state", new smt::long_val(f.get_solver().get_sat_core().value(f.get_phi())));
        const auto est_cost = f.get_estimated_cost();
        smt::json j_cost;
        j_cost->set("num", new smt::long_val(est_cost.numerator()));
        j_cost->set("den", new smt::long_val(est_cost.denominator()));
        j_f->set("cost", j_cost);
        const auto [lb, ub] = f.get_solver().get_idl_theory().bounds(f.get_position());
        smt::json j_pos;
        j_pos->set("lb", new smt::long_val(lb));
        j_pos->set("ub", new smt::long_val(ub));
        j_f->set("pos", j_pos);
        std::istringstream istr(f.get_data());
        j_f->set("data", smt::json::from_json(istr));
        return j_f;
    }
    smt::json gui_server::to_json(const resolver &r)
    {
        smt::json j_r;
        j_r->set("id", new smt::long_val(reinterpret_cast<std::uintptr_t>(&r)));
        std::vector<smt::json> preconditions;
        for (const auto &p : r.get_preconditions())
            preconditions.push_back(new smt::long_val(reinterpret_cast<std::uintptr_t>(p)));
        j_r->set("preconditions", new smt::array_val(preconditions));
        j_r->set("effect", new smt::long_val(reinterpret_cast<std::uintptr_t>(&r.get_effect())));
        j_r->set("state", new smt::long_val(r.get_solver().get_sat_core().value(r.get_rho())));
        const auto intr_cost = r.get_intrinsic_cost();
        smt::json j_cost;
        j_cost->set("num", new smt::long_val(intr_cost.numerator()));
        j_cost->set("den", new smt::long_val(intr_cost.denominator()));
        j_r->set("intrinsic_cost", j_cost);
        std::istringstream istr(r.get_data());
        j_r->set("data", smt::json::from_json(istr));
        return j_r;
    }
} // namespace ratio
