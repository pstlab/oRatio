find_package(Threads REQUIRED)

add_library(executor SHARED executor.cpp)
GENERATE_EXPORT_HEADER(executor)
target_include_directories(executor PUBLIC ${CMAKE_CURRENT_BINARY_DIR} .)
target_link_libraries(executor PUBLIC solver)
install(TARGETS executor LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

if(MSVC)
    target_compile_options(executor PRIVATE /W4)
else()
    target_compile_options(executor PRIVATE -Wall -Wextra -pedantic)
endif()