add_executable(config_tests test_config.cpp)
target_include_directories(config_tests PRIVATE .. ${oRatio_SOURCE_DIR}/smt ${oRatio_SOURCE_DIR}/smt/lra ${oRatio_SOURCE_DIR}/smt/ov ${oRatio_SOURCE_DIR}/smt/dl ${oRatio_SOURCE_DIR}/core ${oRatio_SOURCE_DIR}/solver)
target_link_libraries(config_tests PRIVATE executor)

add_test(NAME EXEC_Config_LibTest COMMAND config_tests WORKING_DIRECTORY ${CMAKE_BINARY_DIR})