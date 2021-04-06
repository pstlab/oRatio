add_library(ql SHARED ql_agent.cpp)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()