#include "solver_socket_listener.h"
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

using namespace smt;

namespace ratio
{
    solver_socket_listener::solver_socket_listener(solver &slv, const std::string &host, const unsigned short &port) : core_listener(slv), solver_listener(slv)
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
    solver_socket_listener::~solver_socket_listener()
    {
#ifdef _WIN32
        closesocket(skt);
        int err_c = WSACleanup();
#else
        close(skt);
#endif
    }

    void solver_socket_listener::log(const std::string &msg)
    {
        json j_msg;
        j_msg->set("message_type", new string_val("log"));
        j_msg->set("log", new string_val(msg));

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }

    void solver_socket_listener::read(const std::string &script)
    {
        json j_msg;
        j_msg->set("message_type", new string_val("read_script"));
        std::string c_script = replace_all(replace_all(script, "\"", "\\\""), "\n", "\\n");
        j_msg->set("script", new string_val(c_script));

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }

    void solver_socket_listener::read(const std::vector<std::string> &files)
    {
        json j_msg;
        j_msg->set("message_type", new string_val("read_files"));
        std::vector<json> j_files;
        for (const auto &file : files)
        {
            std::stringstream ss;
            ss << std::ifstream(file).rdbuf();
            j_files.push_back(new string_val(replace_all(replace_all(ss.str(), "\"", "\\\""), "\n", "\\n")));
        }
        j_msg->set("files", new array_val(j_files));

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }

    void solver_socket_listener::flaw_created(const flaw &f)
    {
        std::pair<I, I> bound = slv.get_idl_theory().bounds(f.get_position());

        json j_msg;
        j_msg->set("message_type", new string_val("flaw_created"));
        j_msg->set("id", new string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const flaw *>(&f)))));
        std::vector<json> j_causes;
        for (const auto &cause : f.get_causes())
            j_causes.push_back(new string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const resolver *>(cause)))));
        j_msg->set("causes", new array_val(j_causes));
        j_msg->set("label", new string_val(replace_all(f.get_label(), "\"", "\\\"")));
        j_msg->set("state", new string_val(std::to_string(slv.get_sat_core().value(f.get_phi()))));
        json j_pos;
        j_pos->set("min", new long_val(bound.first));
        j_pos->set("max", new long_val(bound.second));
        j_msg->set("position", j_pos);

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }
    void solver_socket_listener::flaw_state_changed(const flaw &f)
    {
        json j_msg;
        j_msg->set("message_type", new string_val("flaw_state_changed"));
        j_msg->set("id", new string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const flaw *>(&f)))));
        j_msg->set("state", new string_val(std::to_string(slv.get_sat_core().value(f.get_phi()))));

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }
    void solver_socket_listener::flaw_cost_changed(const flaw &f)
    {
        rational est_cost = f.get_estimated_cost();

        json j_msg;
        j_msg->set("message_type", new string_val("flaw_cost_changed"));
        j_msg->set("id", new string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const flaw *>(&f)))));
        json j_cost;
        j_cost->set("num", new long_val(est_cost.numerator()));
        j_cost->set("den", new long_val(est_cost.denominator()));
        j_msg->set("cost", j_cost);

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }
    void solver_socket_listener::flaw_position_changed(const flaw &f)
    {
        std::pair<I, I> bound = slv.get_idl_theory().bounds(f.get_position());

        json j_msg;
        j_msg->set("message_type", new string_val("flaw_position_changed"));
        j_msg->set("id", new string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const flaw *>(&f)))));
        json j_pos;
        j_pos->set("min", new long_val(bound.first));
        j_pos->set("max", new long_val(bound.second));
        j_msg->set("position", j_pos);

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }
    void solver_socket_listener::current_flaw(const flaw &f)
    {
        json j_msg;
        j_msg->set("message_type", new string_val("current_flaw"));
        j_msg->set("id", new string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const flaw *>(&f)))));

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }

    void solver_socket_listener::resolver_created(const resolver &r)
    {
        rational est_cost = r.get_estimated_cost();

        json j_msg;
        j_msg->set("message_type", new string_val("resolver_created"));
        j_msg->set("id", new string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const resolver *>(&r)))));
        j_msg->set("effect", new string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const flaw *>(&r.get_effect())))));
        j_msg->set("label", new string_val(replace_all(r.get_label(), "\"", "\\\"")));
        j_msg->set("state", new string_val(std::to_string(slv.get_sat_core().value(r.get_rho()))));
        json j_cost;
        j_cost->set("num", new long_val(est_cost.numerator()));
        j_cost->set("den", new long_val(est_cost.denominator()));
        j_msg->set("cost", j_cost);

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }
    void solver_socket_listener::resolver_state_changed(const resolver &r)
    {
        json j_msg;
        j_msg->set("message_type", new string_val("resolver_state_changed"));
        j_msg->set("id", new string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const resolver *>(&r)))));
        j_msg->set("state", new string_val(std::to_string(slv.get_sat_core().value(r.get_rho()))));

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }
    void solver_socket_listener::current_resolver(const resolver &r)
    {
        json j_msg;
        j_msg->set("message_type", new string_val("current_resolver"));
        j_msg->set("id", new string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const resolver *>(&r)))));

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }

    void solver_socket_listener::causal_link_added(const flaw &f, const resolver &r)
    {
        json j_msg;
        j_msg->set("message_type", new string_val("causal_link"));
        j_msg->set("flaw", new string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const flaw *>(&f)))));
        j_msg->set("resolver", new string_val(std::to_string(reinterpret_cast<uintptr_t>(static_cast<const resolver *>(&r)))));

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }

    void solver_socket_listener::state_changed()
    {
        json j_msg;
        j_msg->set("message_type", new string_val("state_changed"));
        j_msg->set("state", slv.to_json());

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }

    void solver_socket_listener::send_message(const std::string &msg)
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