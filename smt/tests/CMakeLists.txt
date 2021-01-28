add_executable(json_lib_tests test_json.cpp)
target_include_directories(json_lib_tests PRIVATE ..)
target_link_libraries(json_lib_tests PRIVATE smt)

add_executable(sat_lib_tests test_sat.cpp)
target_include_directories(sat_lib_tests PRIVATE ..)
target_link_libraries(sat_lib_tests PRIVATE smt)

add_executable(lra_lib_tests test_lra.cpp)
target_include_directories(lra_lib_tests PRIVATE .. ../lra)
target_link_libraries(lra_lib_tests PRIVATE smt)

add_executable(ov_lib_tests test_ov.cpp)
target_include_directories(ov_lib_tests PRIVATE .. ../ov)
target_link_libraries(ov_lib_tests PRIVATE smt)

add_executable(dl_lib_tests test_dl.cpp)
target_include_directories(dl_lib_tests PRIVATE .. ../lra ../dl)
target_link_libraries(dl_lib_tests PRIVATE smt)

add_test(NAME JSON_LibTest COMMAND json_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME SAT_LibTest COMMAND sat_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME LRA_LibTest COMMAND lra_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME OV_LibTest COMMAND ov_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME DL_LibTest COMMAND dl_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})