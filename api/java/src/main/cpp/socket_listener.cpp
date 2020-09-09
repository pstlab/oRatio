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
        std::stringstream ss;
        ss << "read0" << '\n'
           << script << '\n'
           << "EOS" << '\n';
        send_message(ss.str());
    }

    void socket_listener::read(const std::vector<std::string> &files)
    {
        std::stringstream ss;
        ss << "read1" << '\n';
        for (auto f_it = files.cbegin(); f_it != files.cend(); ++f_it)
        {
            if (f_it != files.begin())
                ss << '\n'
                   << "EOF" << '\n';
            ss << std::ifstream(*f_it).rdbuf() << '\n';
        }
        ss << "EOS" << '\n';
        send_message(ss.str());
    }

    void socket_listener::flaw_created(const flaw &f)
    {
        std::stringstream ss;
        ss << "flaw_created {\"flaw\":\"" << static_cast<const void *>(&f) << "\", \"causes\":[";
        const auto causes = f.get_causes();
        for (auto causes_it = causes.begin(); causes_it != causes.end(); ++causes_it)
        {
            if (causes_it != causes.begin())
                ss << ", ";
            ss << static_cast<const void *>(*causes_it);
        }
        std::string label = f.get_label();
        size_t start_pos = 0;
        while ((start_pos = label.find("\"", start_pos)) != std::string::npos)
        {
            label.replace(start_pos, 1, "\\\"");
            start_pos += 2;
        }
        ss << "], \"label\":\"" << label << "\", \"state\":" << std::to_string(slv.get_sat_core().value(f.get_phi()));
        std::pair<smt::I, smt::I> bound = slv.get_idl_theory().bounds(f.get_position());
        ss << ", \"position\":{"
           << "\"min\":" << std::to_string(bound.first) << ", \"max\":" << std::to_string(bound.second) << "}}\n";
        send_message(ss.str());
    }
    void socket_listener::flaw_state_changed(const flaw &f)
    {
        std::stringstream ss;
        ss << "flaw_state_changed {\"flaw\":\"" << static_cast<const void *>(&f) << "\", \"state\":" << std::to_string(slv.get_sat_core().value(f.get_phi())) << "}\n";
        send_message(ss.str());
    }
    void socket_listener::flaw_cost_changed(const flaw &f)
    {
        smt::rational est_cost = f.get_estimated_cost();
        std::stringstream ss;
        ss << "flaw_cost_changed {\"flaw\":\"" << static_cast<const void *>(&f) << "\", \"cost\":{"
           << "\"num\":" << std::to_string(est_cost.numerator()) << ", \"den\":" << std::to_string(est_cost.denominator()) << "}}\n";
        send_message(ss.str());
    }
    void socket_listener::flaw_position_changed(const flaw &f)
    {
        std::pair<smt::I, smt::I> bound = slv.get_idl_theory().bounds(f.get_position());
        std::stringstream ss;
        ss << "flaw_position_changed {\"flaw\":\"" << static_cast<const void *>(&f) << "\", \"position\":{"
           << "\"min\":" << std::to_string(bound.first) << ", \"max\":" << std::to_string(bound.second) << "}}\n";
        send_message(ss.str());
    }
    void socket_listener::current_flaw(const flaw &f)
    {
        std::stringstream ss;
        ss << "current_flaw {\"flaw\":\"" << static_cast<const void *>(&f) << "\"}\n";
        send_message(ss.str());
    }

    void socket_listener::resolver_created(const resolver &r)
    {
        smt::rational est_cost = r.get_estimated_cost();
        std::stringstream ss;
        std::string label = r.get_label();
        size_t start_pos = 0;
        while ((start_pos = label.find("\"", start_pos)) != std::string::npos)
        {
            label.replace(start_pos, 1, "\\\"");
            start_pos += 2;
        }
        ss << "resolver_created {\"resolver\":\"" << static_cast<const void *>(&r) << "\", \"effect\":\"" << static_cast<const void *>(&r.get_effect()) << "\", \"label\":\"" << label << "\", \"cost\":{"
           << "\"num\":" << std::to_string(est_cost.numerator()) << ", \"den\":" << std::to_string(est_cost.denominator()) << "}"
           << ", \"state\":" << std::to_string(slv.get_sat_core().value(r.get_rho())) << "}\n";
        send_message(ss.str());
    }
    void socket_listener::resolver_state_changed(const resolver &r)
    {
        std::stringstream ss;
        ss << "resolver_state_changed {\"resolver\":\"" << static_cast<const void *>(&r) << "\", \"state\":" << std::to_string(slv.get_sat_core().value(r.get_rho())) << "}\n";
        send_message(ss.str());
    }
    void socket_listener::current_resolver(const resolver &r)
    {
        std::stringstream ss;
        ss << "current_resolver {\"resolver\":\"" << static_cast<const void *>(&r) << "\"}\n";
        send_message(ss.str());
    }

    void socket_listener::causal_link_added(const flaw &f, const resolver &r)
    {
        std::stringstream ss;
        ss << "causal_link_added {\"flaw\":\"" << static_cast<const void *>(&f) << "\", \"resolver\":\"" << static_cast<const void *>(&r) << "\"}\n";
        send_message(ss.str());
    }

    void socket_listener::state_changed()
    {
        std::stringstream ss;
        ss << "state_changed " << to_string(slv) << "\n";
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