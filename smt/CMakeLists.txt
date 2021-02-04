file(GLOB SMT_SOURCES *.cpp lra/*.cpp ov/*.cpp dl/*.cpp)

add_library(smt ${SMT_SOURCES})
target_include_directories(smt PUBLIC . lra ov dl)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()