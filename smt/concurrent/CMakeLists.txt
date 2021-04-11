find_package(Threads REQUIRED)

file(GLOB CONCURRENT_SOURCES *.cpp)

add_library(concurrent SHARED ${CONCURRENT_SOURCES})
GENERATE_EXPORT_HEADER(concurrent)
target_include_directories(concurrent PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(concurrent PRIVATE Threads::Threads)
install(TARGETS concurrent LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})