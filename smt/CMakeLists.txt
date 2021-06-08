file(GLOB SMT_SOURCES *.cpp lra/*.cpp ov/*.cpp dl/*.cpp)

add_library(smt SHARED ${SMT_SOURCES})
GENERATE_EXPORT_HEADER(smt)
target_include_directories(smt PUBLIC ${CMAKE_CURRENT_BINARY_DIR} . lra ov dl json)
target_link_libraries(smt PUBLIC json)
install(TARGETS smt LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

if(PARALLELIZE)
    target_include_directories(smt PUBLIC concurrent)
    target_link_libraries(smt PUBLIC concurrent)
endif()

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

# if(MSVC)
#     target_compile_options(smt PRIVATE /W4)
# else()
#     target_compile_options(smt PRIVATE -Wall -Wextra -pedantic)
# endif()