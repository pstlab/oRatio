add_compile_definitions(EXECUTOR_PORT=${EXECUTOR_PORT} BUILD_EXECUTOR_LISTENER BUILD_SOLVER_LISTENER)

add_library(executor-listener executor_socket_listener.cpp)
target_include_directories(executor-listener PRIVATE ${oRatio_SOURCE_DIR}/smt ${oRatio_SOURCE_DIR}/smt/lra ${oRatio_SOURCE_DIR}/smt/ov ${oRatio_SOURCE_DIR}/smt/dl ${oRatio_SOURCE_DIR}/core ${oRatio_SOURCE_DIR}/solver ${oRatio_SOURCE_DIR}/executor)