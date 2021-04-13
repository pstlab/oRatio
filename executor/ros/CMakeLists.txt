add_executable(riddle2ros main.cpp)
target_include_directories(riddle2ros PRIVATE solver core smt smt/lra smt/ov smt/dl)
target_link_libraries(riddle2ros PRIVATE solver)
install(TARGETS riddle2ros RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})