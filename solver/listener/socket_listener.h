#pragma once

#include "solver_listener.h"
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#endif

namespace ratio
{

class socket_listener : public solver_listener
{
  public:
    socket_listener(solver &slv);
    socket_listener(const socket_listener &orig) = delete;
    virtual ~socket_listener();

  private:
    void flaw_created(const flaw &f) override;
    void flaw_state_changed(const flaw &f) override;
    void current_flaw(const flaw &f) override;

    void resolver_created(const resolver &r) override;
    void resolver_state_changed(const resolver &r) override;
    void resolver_cost_changed(const resolver &r) override;
    void current_resolver(const resolver &r) override;

    void causal_link_added(const flaw &f, const resolver &r) override;

    void skt_init();
    void skt_send(const std::string &msg);
    void skt_close();

#ifdef _WIN32
    SOCKET skt;
#else
    int skt;
#endif
};
}