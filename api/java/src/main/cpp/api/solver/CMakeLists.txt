add_compile_definitions(BUILD_SOLVER_LISTENER BUILD_EXECUTOR_LISTENER)

find_package(JNI)

add_library(solver-api it_cnr_istc_pst_oratio_Solver.cpp java_solver_listener.cpp)
target_include_directories(solver-api PRIVATE ${JNI_INCLUDE_DIRS} ${oRatio_SOURCE_DIR}/smt ${oRatio_SOURCE_DIR}/smt/lra ${oRatio_SOURCE_DIR}/smt/ov ${oRatio_SOURCE_DIR}/smt/dl ${oRatio_SOURCE_DIR}/core ${oRatio_SOURCE_DIR}/solver)