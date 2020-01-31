file(GLOB SAT_SOURCES test_sat.cpp ${CMAKE_SOURCE_DIR}/smt/*.cpp)
file(GLOB LRA_SOURCES test_lra.cpp ${CMAKE_SOURCE_DIR}/smt/*.cpp ${CMAKE_SOURCE_DIR}/smt/lra/*.cpp)
file(GLOB OV_SOURCES test_ov.cpp ${CMAKE_SOURCE_DIR}/smt/*.cpp ${CMAKE_SOURCE_DIR}/smt/ov/*.cpp)
file(GLOB TIME_SOURCES test_time.cpp ${CMAKE_SOURCE_DIR}/smt/*.cpp ${CMAKE_SOURCE_DIR}/smt/time/*.cpp ${CMAKE_SOURCE_DIR}/smt/lra/*.cpp)

add_executable(sat_lib_tests ${SAT_SOURCES})
add_executable(lra_lib_tests ${LRA_SOURCES})
add_executable(ov_lib_tests ${OV_SOURCES})
add_executable(time_lib_tests ${TIME_SOURCES})

if(OPTIMIZE_FOR_NATIVE_ARCH)
    target_link_libraries(lra_lib_tests PRIVATE Threads::Threads)
endif()

add_test(NAME SAT_LibTest COMMAND sat_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME LRA_LibTest COMMAND lra_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME OV_LibTest COMMAND ov_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
add_test(NAME TIME_LibTest COMMAND time_lib_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})