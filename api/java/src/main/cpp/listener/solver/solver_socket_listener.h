#pragma once

#include "core_listener.h"
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
#include <string>

namespace ratio
{

  class solver_socket_listener : public core_listener, public solver_listener
  {
  public:
    solver_socket_listener(solver &slv, const std::string &host, const unsigned short &port);
    solver_socket_listener(const solver_socket_listener &orig) = delete;
    virtual ~solver_socket_listener();

  private:
    void log(const std::string &msg) override;
    void read(const std::string &script) override;
    void read(const std::vector<std::string> &files) override;

    void flaw_created(const flaw &f) override;
    void flaw_state_changed(const flaw &f) override;
    void flaw_cost_changed(const flaw &f) override;
    void flaw_position_changed(const flaw &f) override;
    void current_flaw(const flaw &f) override;

    void resolver_created(const resolver &r) override;
    void resolver_state_changed(const resolver &r) override;
    void current_resolver(const resolver &r) override;

    void causal_link_added(const flaw &f, const resolver &r) override;

    void state_changed() override;

    void send_message(const std::string &msg);

    static std::string replace_all(std::string str, const std::string &from, const std::string &to)
    {
      size_t start_pos = 0;
      while ((start_pos = str.find(from, start_pos)) != std::string::npos)
      {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length();
      }
      return str;
    }

#ifdef _WIN32
    SOCKET skt;
#else
    int skt;
#endif
  };
} // namespace ratio