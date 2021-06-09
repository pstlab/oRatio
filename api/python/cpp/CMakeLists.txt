find_package(Python COMPONENTS Development REQUIRED)

add_library(solver-python-api SHARED solver.cpp)
target_include_directories(solver-python-api PUBLIC ${Python_INCLUDE_DIRS} ${oRatio_SOURCE_DIR}/smt ${oRatio_SOURCE_DIR}/smt/lra ${oRatio_SOURCE_DIR}/smt/ov ${oRatio_SOURCE_DIR}/smt/dl ${oRatio_SOURCE_DIR}/core ${oRatio_SOURCE_DIR}/solver)
target_link_libraries(solver-python-api PUBLIC solver)
