file(GLOB CORE_SOURCES *.cpp)

add_library(core STATIC ${CORE_SOURCES})
target_include_directories(core PRIVATE ${oRatio_SOURCE_DIR}/smt ${oRatio_SOURCE_DIR}/smt/lra ${oRatio_SOURCE_DIR}/smt/ov ${oRatio_SOURCE_DIR}/smt/dl ${oRatio_SOURCE_DIR}/riddle)
target_link_libraries(core PRIVATE smt riddle)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()