file(GLOB CORE_SOURCES *.cpp)

add_library(core ${CORE_SOURCES})
target_include_directories(core PUBLIC ${oRatio_SOURCE_DIR}/smt ${oRatio_SOURCE_DIR}/smt/lra ${oRatio_SOURCE_DIR}/smt/ov ${oRatio_SOURCE_DIR}/smt/dl ${oRatio_SOURCE_DIR}/riddle)
target_link_libraries(core PUBLIC smt riddle)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()