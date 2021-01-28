add_library(ql ql_agent.cpp)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()