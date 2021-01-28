find_package(Threads)

add_library(executor executor.cpp)
target_include_directories(executor PRIVATE lra ov dl)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()