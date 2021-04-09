file(GLOB SMT_SOURCES *.cpp lra/*.cpp ov/*.cpp dl/*.cpp)

add_library(smt SHARED ${SMT_SOURCES})
GENERATE_EXPORT_HEADER(smt)
target_include_directories(smt PUBLIC . lra ov dl json ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(smt PUBLIC json)

if(PARALLELIZE)
    target_include_directories(smt PUBLIC concurrent)
    target_link_libraries(smt PUBLIC concurrent)
endif()

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()