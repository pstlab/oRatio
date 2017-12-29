#include "socket_listener.h"
#include <iostream>

namespace ratio
{
socket_listener::socket_listener(solver &slv) : solver_listener(slv) {}
socket_listener::~socket_listener() {}

void socket_listener::flaw_created(const flaw &f) {}
void socket_listener::flaw_state_changed(const flaw &f) {}
void socket_listener::current_flaw(const flaw &f) {}

void socket_listener::resolver_created(const resolver &r) {}
void socket_listener::resolver_state_changed(const resolver &r) {}
void socket_listener::resolver_cost_changed(const resolver &r) {}
void socket_listener::current_resolver(const resolver &r) {}

void socket_listener::causal_link_added(const flaw &f, const resolver &r) {}

void socket_listener::skt_init()
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
void socket_listener::skt_send(const std::string &msg)
{
    int total = 0;
    int len = msg.size();
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
void socket_listener::skt_close()
{
#ifdef _WIN32
    closesocket(skt);
    int err_c = WSACleanup();
#else
    close(skt);
#endif
}
}