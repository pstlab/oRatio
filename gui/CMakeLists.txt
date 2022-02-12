add_executable(oRatioServer main.cpp gui_server.cpp)

find_package(Crow)

target_include_directories(oRatioServer PRIVATE .)
target_link_libraries(oRatioServer PRIVATE solver executor Crow::Crow)
install(TARGETS oRatioServer RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})