#include "socket_listener.h"
#include "core.h"
#include "method.h"
#include "type.h"
#include "predicate.h"
#include "graph.h"
#include "solver.h"
#include <sstream>
#include <iostream>

namespace ratio
{
socket_listener::socket_listener(solver &slv) : core_listener(slv), solver_listener(slv)
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
    sa.sin_port = htons(1100);
    inet_pton(AF_INET, "127.0.0.1", &sa.sin_addr);

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

void socket_listener::method_created(const method &m)
{
    std::stringstream ss;
    ss << "method_created {\"name\":\"" << m.get_name() << "\"";
    if (m.get_return_type())
        ss << "\", \"return_type\":\"" << static_cast<const void *>(m.get_return_type()) << "\"";
    ss << "}\n";
    send_message(ss.str());
}

void socket_listener::method_created(const type &t, const method &m)
{
    std::stringstream ss;
    ss << "method_created {\"type\":\"" << static_cast<const void *>(&t) << "\", \"name\":\"" << m.get_name() << "\"";
    if (m.get_return_type())
        ss << "\", \"return_type\":\"" << static_cast<const void *>(m.get_return_type()) << "\"";
    ss << "}\n";
    send_message(ss.str());
}

void socket_listener::type_created(const type &t)
{
    std::stringstream ss;
    ss << "type_created {\"type\":\"" << static_cast<const void *>(&t) << "\", \"name\":\"" << t.get_name() << "\", \"primitive\":" << std::to_string(t.is_primitive()) << "}\n";
    send_message(ss.str());
}

void socket_listener::type_created(const type &et, const type &t)
{
    std::stringstream ss;
    ss << "type_created {\"enclosing_type\":\"" << static_cast<const void *>(&et) << "\", \"type\":\"" << static_cast<const void *>(&t) << "\", \"name\":\"" << t.get_name() << "\", \"primitive\":" << std::to_string(t.is_primitive()) << "}\n";
    send_message(ss.str());
}

void socket_listener::type_inherited(const type &st, const type &t)
{
    std::stringstream ss;
    ss << "type_inherited {\"supertype\":\"" << static_cast<const void *>(&st) << "\", \"type\":\"" << static_cast<const void *>(&t) << "\"}\n";
    send_message(ss.str());
}

void socket_listener::predicate_created(const predicate &p)
{
    std::stringstream ss;
    ss << "predicate_created {\"predicate\":\"" << static_cast<const void *>(&p) << "\", \"name\":\"" << p.get_name() << "\"}\n";
    send_message(ss.str());
}

void socket_listener::predicate_created(const type &t, const predicate &p)
{
    std::stringstream ss;
    ss << "predicate_created {\"type\":\"" << static_cast<const void *>(&t) << "\", \"predicate\":\"" << static_cast<const void *>(&p) << "\", \"name\":\"" << t.get_name() << "\"}\n";
    send_message(ss.str());
}

void socket_listener::constructor_created(const type &t, const constructor &c)
{
    std::stringstream ss;
    ss << "constructor_created {\"type\":\"" << static_cast<const void *>(&t) << "\", \"constructor\":\"" << static_cast<const void *>(&c) << "}\n";
    send_message(ss.str());
}

void socket_listener::flaw_created(const flaw &f)
{
    std::stringstream ss;
    ss << "flaw_created {\"flaw\":\"" << static_cast<const void *>(&f) << "\", \"causes\":[";
    std::vector<resolver *> causes = f.get_causes();
    for (std::vector<resolver *>::iterator causes_it = causes.begin(); causes_it != causes.end(); ++causes_it)
    {
        if (causes_it != causes.begin())
            ss << ", ";
        ss << static_cast<const void *>(*causes_it);
    }
    ss << "], \"label\":\"" << f.get_label() << "\", \"state\":" << std::to_string(slv.get_sat_core().value(f.get_phi())) << "}\n";
    send_message(ss.str());
}
void socket_listener::flaw_state_changed(const flaw &f)
{
    std::stringstream ss;
    ss << "flaw_state_changed {\"flaw\":\"" << static_cast<const void *>(&f) << "\", \"state\":" << std::to_string(slv.get_sat_core().value(f.get_phi())) << "}\n";
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
    ss << "resolver_created {\"resolver\":\"" << static_cast<const void *>(&r) << "\", \"effect\":\"" << static_cast<const void *>(&r.get_effect()) << "\", \"label\":\"" << r.get_label() << "\", \"cost\":{"
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
void socket_listener::resolver_cost_changed(const resolver &r)
{
    smt::rational est_cost = r.get_estimated_cost();
    std::stringstream ss;
    ss << "resolver_cost_changed {\"resolver\":\"" << static_cast<const void *>(&r) << "\", \"cost\":{"
       << "\"num\":" << std::to_string(est_cost.numerator()) << ", \"den\":" << std::to_string(est_cost.denominator()) << "}}\n";
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

void socket_listener::solution_found()
{
    std::stringstream ss;
    ss << "solution_found " << slv.to_string() << "\n";
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