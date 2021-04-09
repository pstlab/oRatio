find_package(JNI REQUIRED)

add_library(solver-api SHARED it_cnr_istc_pst_oratio_Solver.cpp scoped_env.cpp java_core_listener.cpp java_solver_listener.cpp)
target_include_directories(solver-api PUBLIC ${JNI_INCLUDE_DIRS} ${oRatio_SOURCE_DIR}/smt ${oRatio_SOURCE_DIR}/smt/lra ${oRatio_SOURCE_DIR}/smt/ov ${oRatio_SOURCE_DIR}/smt/dl ${oRatio_SOURCE_DIR}/core ${oRatio_SOURCE_DIR}/solver)
target_link_libraries(solver-api PUBLIC solver)
install(TARGETS solver-api LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

if(BUILD_EXECUTOR)
    add_library(executor-api SHARED it_cnr_istc_pst_oratio_timelines_TimelinesExecutor.cpp scoped_env.cpp java_executor_listener.cpp)
    target_include_directories(executor-api PUBLIC ${JNI_INCLUDE_DIRS} ${oRatio_SOURCE_DIR}/smt ${oRatio_SOURCE_DIR}/smt/lra ${oRatio_SOURCE_DIR}/smt/ov ${oRatio_SOURCE_DIR}/smt/dl ${oRatio_SOURCE_DIR}/core ${oRatio_SOURCE_DIR}/solver ${oRatio_SOURCE_DIR}/executor)
    target_link_libraries(executor-api PUBLIC solver-api executor)
endif(BUILD_EXECUTOR)
