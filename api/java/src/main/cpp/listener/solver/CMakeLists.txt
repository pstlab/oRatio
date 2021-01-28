add_compile_definitions(HOST="${HOST}" SOLVER_PORT=${SOLVER_PORT} BUILD_SOLVER_LISTENER)

add_library(solver-listener solver_socket_listener.cpp)
target_include_directories(solver-listener PRIVATE ${oRatio_SOURCE_DIR}/smt ${oRatio_SOURCE_DIR}/smt/lra ${oRatio_SOURCE_DIR}/smt/ov ${oRatio_SOURCE_DIR}/smt/dl ${oRatio_SOURCE_DIR}/core ${oRatio_SOURCE_DIR}/solver ${oRatio_SOURCE_DIR}/solver)