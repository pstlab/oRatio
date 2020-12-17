#include "executor_socket_listener.h"
#include "atom.h"
#include <sstream>

using namespace smt;

namespace ratio
{
    executor_socket_listener::executor_socket_listener(executor &e, const std::string &host, const unsigned short &port) : executor_listener(e)
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
    executor_socket_listener::~executor_socket_listener()
    {
#ifdef _WIN32
        closesocket(skt);
        int err_c = WSACleanup();
#else
        close(skt);
#endif
    }

    void executor_socket_listener::tick()
    {
        json j_msg;
        j_msg->set("message_type", new string_val("tick"));
        const auto &current_time = get_executor().get_current_time();
        json j_current_time;
        j_current_time->set("num", new long_val(current_time.numerator()));
        j_current_time->set("den", new long_val(current_time.denominator()));
        j_msg->set("current_time", j_current_time);

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }

    void executor_socket_listener::starting(const std::set<atom *> &atoms)
    {
        json j_msg;
        j_msg->set("message_type", new string_val("starting_atoms"));

        std::vector<json> j_atms;
        for (const auto &atm : atoms)
            j_atms.push_back(atm->to_json());
        j_msg->set("atoms", new array_val(j_atms));

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }

    void executor_socket_listener::ending(const std::set<atom *> &atoms)
    {
        json j_msg;
        j_msg->set("message_type", new string_val("ending_atoms"));

        std::vector<json> j_atms;
        for (const auto &atm : atoms)
            j_atms.push_back(atm->to_json());
        j_msg->set("atoms", new array_val(j_atms));

        std::stringstream ss;
        ss << j_msg << '\n';
        send_message(ss.str());
    }

    void executor_socket_listener::send_message(const std::string &msg)
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