find_package(Threads REQUIRED)

file(GLOB CONCURRENT_SOURCES *.cpp)

add_library(concurrent ${CONCURRENT_SOURCES})
target_link_libraries(concurrent PRIVATE Threads::Threads)