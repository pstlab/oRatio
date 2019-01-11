#pragma once

#include "solver_listener.h"
#include "core_listener.h"
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

class socket_listener : public core_listener, public solver_listener
{
public:
  socket_listener(solver &slv);
  socket_listener(const socket_listener &orig) = delete;
  virtual ~socket_listener();

private:
  void method_created(const method &m) override;
  void method_created(const type &t, const method &m) override;

  void type_created(const type &t) override;
  void type_created(const type &et, const type &t) override;
  void type_inherited(const type &st, const type &t) override;

  void predicate_created(const predicate &p) override;
  void predicate_created(const type &t, const predicate &p) override;

  void constructor_created(const type &et, const constructor &c) override;

private:
  void flaw_created(const flaw &f) override;
  void flaw_state_changed(const flaw &f) override;
  void current_flaw(const flaw &f) override;

  void resolver_created(const resolver &r) override;
  void resolver_state_changed(const resolver &r) override;
  void resolver_cost_changed(const resolver &r) override;
  void current_resolver(const resolver &r) override;

  void causal_link_added(const flaw &f, const resolver &r) override;

  void solution_found();

  void send_message(const std::string &msg);

#ifdef _WIN32
  SOCKET skt;
#else
  int skt;
#endif
};
} // namespace ratio