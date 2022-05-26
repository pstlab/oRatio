add_executable(rbs_lib_tests test_rbs.cpp)
target_include_directories(rbs_lib_tests PRIVATE .. ../parser)
target_link_libraries(rbs_lib_tests PRIVATE rbs smt)

