#pragma once

#include "executor_listener.h"
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

  class executor_socket_listener : public executor_listener
  {
  public:
    executor_socket_listener(executor &e, const std::string &host, const unsigned short &port);
    executor_socket_listener(const executor_socket_listener &orig) = delete;
    virtual ~executor_socket_listener();

  private:
    void tick() override;
    void starting(const std::set<atom *> &atoms) override;
    void ending(const std::set<atom *> &atoms) override;

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