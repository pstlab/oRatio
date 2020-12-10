#include "socket_listener.h"
#include "core.h"
#include "method.h"
#include "type.h"
#include "predicate.h"
#include "field.h"
#include "graph.h"
#include "solver.h"
#include <sstream>
#include <fstream>
#include <iostream>

namespace ratio
{
    std::string replace_all(std::string str, const std::string &from, const std::string &to)
    {
        size_t start_pos = 0;
        while ((start_pos = str.find(from, start_pos)) != std::string::npos)
        {
            str.replace(start_pos, from.length(), to);
            start_pos += to.length();
        }
        return str;
    }

    socket_listener::socket_listener(solver &slv, const std::string &host, const unsigned short &port) : core_listener(slv), solver_listener(slv)
    {
#ifdef _WIN32
        WSADATA wsa_data;
        int err_c = WSAStartup(MAKEWORD(2, 2), &wsa_data);
        if (err_c != 0)
            std::cerr << "WSAStartup failed with error: " << std::to_string(err_c) << std::endl;
#endif

        skt = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

#ifdef _WIN32
        if (skt == INVALID_SOCKET)
#else
        if (skt < 0)
#endif
            std::cerr << "unable to connect to server.." << std::endl;

        struct sockaddr_in sa;
        sa.sin_family = AF_INET;
        sa.sin_port = htons(port);
        inet_pton(AF_INET, host.c_str(), &sa.sin_addr);

        if (connect(skt, (struct sockaddr *)&sa, sizeof(sa)) < 0)
            std::cerr << "unable to connect to server.." << std::endl;
    }
    socket_listener::~socket_listener()
    {
#ifdef _WIN32
        closesocket(skt);
        int err_c = WSACleanup();
#else
        close(skt);
#endif
    }

    void socket_listener::log(const std::string &msg)
    {
        std::stringstream ss;
        ss << "log " << msg << '\n';
        send_message(ss.str());
    }

    void socket_listener::read(const std::string &script)
    {
        smt::json j_msg;
        j_msg->set("message-type", new smt::string_val("read-script"));
        std::string c_script = script;
        replace_all(c_script, "\"", "\\\"");
        j_msg->set("script", new smt::string_val(c_script));

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }

    void socket_listener::read(const std::vector<std::string> &files)
    {
        smt::json j_msg;
        j_msg->set("message-type", new smt::string_val("read-files"));
        std::vector<smt::json> j_files;
        for (const auto &file : files)
        {
            std::string c_file = file;
            replace_all(c_file, "\"", "\\\"");
            j_files.push_back(new smt::string_val(c_file));
        }
        j_msg->set("files", new smt::array_val(j_files));

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }

    void socket_listener::flaw_created(const flaw &f)
    {
        std::string label = f.get_label();
        replace_all(label, "\"", "\\\"");
        std::pair<smt::I, smt::I> bound = slv.get_idl_theory().bounds(f.get_position());

        smt::json j_msg;
        j_msg->set("message-type", new smt::string_val("flaw-created"));
        j_msg->set("id", new smt::string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const flaw *>(&f)))));
        std::vector<smt::json> j_causes;
        for (const auto &cause : f.get_causes())
            j_causes.push_back(new smt::string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const resolver *>(cause)))));
        j_msg->set("causes", new smt::array_val(j_causes));
        j_msg->set("label", new smt::string_val(label));
        j_msg->set("state", new smt::string_val(std::to_string(slv.get_sat_core().value(f.get_phi()))));
        smt::json j_pos;
        j_pos->set("min", new smt::string_val(std::to_string(bound.first)));
        j_pos->set("max", new smt::string_val(std::to_string(bound.second)));
        j_msg->set("pos", j_pos);

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }
    void socket_listener::flaw_state_changed(const flaw &f)
    {
        smt::json j_msg;
        j_msg->set("message-type", new smt::string_val("flaw-state-changed"));
        j_msg->set("id", new smt::string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const flaw *>(&f)))));
        j_msg->set("state", new smt::string_val(std::to_string(slv.get_sat_core().value(f.get_phi()))));

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }
    void socket_listener::flaw_cost_changed(const flaw &f)
    {
        smt::rational est_cost = f.get_estimated_cost();

        smt::json j_msg;
        j_msg->set("message-type", new smt::string_val("flaw-cost-changed"));
        j_msg->set("id", new smt::string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const flaw *>(&f)))));
        smt::json j_cost;
        j_cost->set("num", new smt::string_val(std::to_string(est_cost.numerator())));
        j_cost->set("den", new smt::string_val(std::to_string(est_cost.denominator())));
        j_msg->set("cost", j_cost);

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }
    void socket_listener::flaw_position_changed(const flaw &f)
    {
        std::pair<smt::I, smt::I> bound = slv.get_idl_theory().bounds(f.get_position());

        smt::json j_msg;
        j_msg->set("message-type", new smt::string_val("flaw-position-changed"));
        j_msg->set("id", new smt::string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const flaw *>(&f)))));
        smt::json j_pos;
        j_pos->set("min", new smt::string_val(std::to_string(bound.first)));
        j_pos->set("max", new smt::string_val(std::to_string(bound.second)));
        j_msg->set("pos", j_pos);

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }
    void socket_listener::current_flaw(const flaw &f)
    {
        smt::json j_msg;
        j_msg->set("message-type", new smt::string_val("current-flaw"));
        j_msg->set("id", new smt::string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const flaw *>(&f)))));

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }

    void socket_listener::resolver_created(const resolver &r)
    {
        smt::rational est_cost = r.get_estimated_cost();
        std::string label = r.get_label();
        replace_all(label, "\"", "\\\"");

        smt::json j_msg;
        j_msg->set("message-type", new smt::string_val("resolver-created"));
        j_msg->set("id", new smt::string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const resolver *>(&r)))));
        j_msg->set("effect", new smt::string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const flaw *>(&r.get_effect())))));
        j_msg->set("label", new smt::string_val(label));
        j_msg->set("state", new smt::string_val(std::to_string(slv.get_sat_core().value(r.get_rho()))));
        smt::json j_cost;
        j_cost->set("num", new smt::string_val(std::to_string(est_cost.numerator())));
        j_cost->set("den", new smt::string_val(std::to_string(est_cost.denominator())));
        j_msg->set("cost", j_cost);

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }
    void socket_listener::resolver_state_changed(const resolver &r)
    {
        smt::json j_msg;
        j_msg->set("message-type", new smt::string_val("resolver-state-changed"));
        j_msg->set("id", new smt::string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const resolver *>(&r)))));
        j_msg->set("state", new smt::string_val(std::to_string(slv.get_sat_core().value(r.get_rho()))));

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }
    void socket_listener::current_resolver(const resolver &r)
    {
        smt::json j_msg;
        j_msg->set("message-type", new smt::string_val("current-resolver"));
        j_msg->set("id", new smt::string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const resolver *>(&r)))));

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }

    void socket_listener::causal_link_added(const flaw &f, const resolver &r)
    {
        smt::json j_msg;
        j_msg->set("message-type", new smt::string_val("causal-link"));
        j_msg->set("flaw-id", new smt::string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const flaw *>(&f)))));
        j_msg->set("resolver-id", new smt::string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const resolver *>(&r)))));

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }

    void socket_listener::state_changed()
    {
        smt::json j_msg;
        j_msg->set("message-type", new smt::string_val("state-changed"));
        j_msg->set("state", slv.to_json());

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }

    void socket_listener::send_message(const std::string &msg)
    {
        int total = 0;
        int len = static_cast<int>(msg.size());
        int bytesleft = len;
        int n = -1;
        while (total < len)
        {
            n = send(skt, msg.c_str() + total, bytesleft, 0);
            if (n <= 0)
                throw(n);
            total += n;
            bytesleft -= n;
        }
    }
} // namespace ratio