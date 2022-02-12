#include "executor.h"
#include <crow.h>

int main(int argc, char const *argv[])
{
    if (argc < 3)
    {
        std::cerr << "usage: oRatio <input-file> [<input-file> ...] <output-file>\n";
        return -1;
    }

    // the problem files..
    std::vector<std::string> prob_names;
    for (int i = 1; i < argc - 1; i++)
        prob_names.push_back(argv[i]);

    // the solution file..
    std::string sol_name = argv[argc - 1];

    crow::SimpleApp app;
    CROW_ROUTE(app, "/")
    ([]()
     { 
        crow::mustache::context ctx;
        return crow::mustache::load("index.html").render(ctx); });

    CROW_ROUTE(app, "/ws")
        .websocket()
        .onopen([&](crow::websocket::connection &conn) {})
        .onclose([&](crow::websocket::connection &conn, const std::string &reason) {})
        .onmessage([&](crow::websocket::connection &conn, const std::string &data, bool is_binary) {});

    app.port(8080).multithreaded().run();
    return 0;
}
