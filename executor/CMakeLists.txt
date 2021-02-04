add_compile_definitions(BUILD_SOLVER_LISTENER)

find_package(Threads)

add_library(executor executor.cpp)
target_include_directories(executor PUBLIC ${oRatio_SOURCE_DIR}/smt ${oRatio_SOURCE_DIR}/smt/lra ${oRatio_SOURCE_DIR}/smt/ov ${oRatio_SOURCE_DIR}/smt/dl ${oRatio_SOURCE_DIR}/core ${oRatio_SOURCE_DIR}/solver)
target_link_libraries(executor PUBLIC solver)