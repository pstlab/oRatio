find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

add_library(dql SHARED dql_agent.cpp)
target_link_libraries(dql "${TORCH_LIBRARIES}")
set_property(TARGET dql PROPERTY CXX_STANDARD 14)

if (MSVC)
  file(GLOB TORCH_DLLS "${TORCH_INSTALL_PREFIX}/lib/*.dll")
  add_custom_command(TARGET dql_tests POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different ${TORCH_DLLS} $<TARGET_FILE_DIR:ql_tests>)
endif (MSVC)

include(CTest)
enable_testing()

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()