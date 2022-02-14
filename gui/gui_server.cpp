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
            std::lock_guard<std::mutex> _(mtx);
            std::stringstream ss;
            slv.extract_timelines().to_json(ss);
            return ss.str(); });

        CROW_ROUTE(app, "/graph")
        ([&]()
         {
            std::lock_guard<std::mutex> _(mtx);
            smt::json j_gr;
            std::vector<smt::json> j_flaws;
            for (const auto &f : flaws)
            {
                smt::json j_f;
                j_f->set("id", new smt::long_val(reinterpret_cast<std::uintptr_t>(f)));
                std::vector<smt::json> causes;
                for (const auto &c : f->get_causes())
                    causes.push_back(new smt::long_val(reinterpret_cast<std::uintptr_t>(c)));
                j_f->set("causes", new smt::array_val(causes));
                std::istringstream istr(f->get_label());
                j_f->set("label", smt::json::from_json(istr));
                j_f->set("state", new smt::long_val(slv.get_sat_core().value(f->get_phi())));
                const auto [lb, ub] = slv.get_idl_theory().bounds(f->get_position());
                std::vector<smt::json> pos;
                pos.push_back(new smt::long_val(lb));
                pos.push_back(new smt::long_val(ub));
                j_f->set("pos", new smt::array_val(pos));
                j_flaws.push_back(j_f);
            }
            j_gr->set("flaws", new smt::array_val(j_flaws));
            if (c_flaw)
                j_gr->set("current-flaw", new smt::long_val(reinterpret_cast<std::uintptr_t>(c_flaw)));
            std::vector<smt::json> j_resolvers;
            for (const auto &r : resolvers)
            {
                smt::json j_r;
                j_r->set("id", new smt::long_val(reinterpret_cast<std::uintptr_t>(r)));
                j_r->set("effect", new smt::long_val(reinterpret_cast<std::uintptr_t>(&r->get_effect())));
                std::istringstream istr(r->get_label());
                j_r->set("label", smt::json::from_json(istr));
                const auto intr_cost = r->get_intrinsic_cost();
                smt::json j_cost;
                j_cost->set("num", new smt::long_val(intr_cost.numerator()));
                j_cost->set("den", new smt::long_val(intr_cost.denominator()));
                j_r->set("intrinsic-cost", j_cost);
                j_r->set("state", new smt::long_val(slv.get_sat_core().value(r->get_rho())));
            }
            j_gr->set("resolvers", new smt::array_val(j_resolvers));
            if (c_resolver)
                j_gr->set("current-resolver", new smt::long_val(reinterpret_cast<std::uintptr_t>(c_resolver)));
            std::stringstream ss;
            j_gr.to_json(ss);
            return ss.str(); });

        CROW_ROUTE(app, "/solver")
            .websocket()
            .onopen([&](crow::websocket::connection &conn)
                    { std::lock_guard<std::mutex> _(mtx); users.insert(&conn); })
            .onclose([&](crow::websocket::connection &conn, const std::string &reason)
                     { std::lock_guard<std::mutex> _(mtx); users.erase(&conn); });
    }
    gui_server::~gui_server() {}

    void gui_server::start() { app.bindaddr(host).port(port).run(); }
    void gui_server::wait_for_server_start() { app.wait_for_server_start(); }
    void gui_server::stop() { app.stop(); }

    void gui_server::log(const std::string &msg) { std::lock_guard<std::mutex> _(mtx); }
    void gui_server::read(const std::string &script) { std::lock_guard<std::mutex> _(mtx); }
    void gui_server::read(const std::vector<std::string> &files) { std::lock_guard<std::mutex> _(mtx); }

    void gui_server::state_changed() { std::lock_guard<std::mutex> _(mtx); }

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

        smt::json j_fc;
        j_fc->set("type", new smt::string_val("flaw_created"));
        j_fc->set("id", new smt::long_val(reinterpret_cast<std::uintptr_t>(&f)));
        std::vector<smt::json> causes;
        for (const auto &c : f.get_causes())
            causes.push_back(new smt::long_val(reinterpret_cast<std::uintptr_t>(c)));
        j_fc->set("causes", new smt::array_val(causes));
        std::istringstream istr(f.get_label());
        j_fc->set("label", smt::json::from_json(istr));
        j_fc->set("state", new smt::long_val(slv.get_sat_core().value(f.get_phi())));
        const auto [lb, ub] = slv.get_idl_theory().bounds(f.get_position());
        std::vector<smt::json> pos;
        pos.push_back(new smt::long_val(lb));
        pos.push_back(new smt::long_val(ub));
        j_fc->set("pos", new smt::array_val(pos));

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
        std::vector<smt::json> pos;
        pos.push_back(new smt::long_val(lb));
        pos.push_back(new smt::long_val(ub));
        j_fpc->set("pos", new smt::array_val(pos));

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

        smt::json j_rc;
        j_rc->set("type", new smt::string_val("resolver_created"));
        j_rc->set("id", new smt::long_val(reinterpret_cast<std::uintptr_t>(&r)));
        j_rc->set("effect", new smt::long_val(reinterpret_cast<std::uintptr_t>(&r.get_effect())));
        std::istringstream istr(r.get_label());
        j_rc->set("label", smt::json::from_json(istr));
        const auto intr_cost = r.get_intrinsic_cost();
        smt::json j_cost;
        j_cost->set("num", new smt::long_val(intr_cost.numerator()));
        j_cost->set("den", new smt::long_val(intr_cost.denominator()));
        j_rc->set("intrinsic-cost", j_cost);
        j_rc->set("state", new smt::long_val(slv.get_sat_core().value(r.get_rho())));

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
        j_cla->set("flaw-id", new smt::long_val(reinterpret_cast<std::uintptr_t>(&f)));
        j_cla->set("resolver-id", new smt::long_val(reinterpret_cast<std::uintptr_t>(&r)));

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
} // namespace ratio
