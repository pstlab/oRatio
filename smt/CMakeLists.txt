file(GLOB SMT_SOURCES *.cpp lra/*.cpp ov/*.cpp dl/*.cpp)

add_library(smt STATIC ${SMT_SOURCES})
target_include_directories(smt PRIVATE . lra ov dl)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()