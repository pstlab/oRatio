file(GLOB CORE_SOURCES *.cpp)

add_library(core ${CORE_SOURCES})
target_include_directories(core PRIVATE ../smt ../smt/lra ../smt/ov ../smt/dl ../riddle)
target_link_libraries(core PRIVATE smt riddle)