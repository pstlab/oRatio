find_package(JNI REQUIRED)

add_library(solver-java-api SHARED it_cnr_istc_pst_oratio_Solver.cpp scoped_env.cpp java_core_listener.cpp java_solver_listener.cpp)
target_include_directories(solver-java-api PUBLIC ${JNI_INCLUDE_DIRS} ${oRatio_SOURCE_DIR}/smt ${oRatio_SOURCE_DIR}/smt/lra ${oRatio_SOURCE_DIR}/smt/ov ${oRatio_SOURCE_DIR}/smt/dl ${oRatio_SOURCE_DIR}/core ${oRatio_SOURCE_DIR}/solver)
target_link_libraries(solver-java-api PUBLIC solver)
install(TARGETS solver-java-api LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

if(BUILD_EXECUTOR)
    add_library(executor-java-api SHARED it_cnr_istc_pst_oratio_timelines_TimelinesExecutor.cpp scoped_env.cpp java_executor_listener.cpp)
    target_include_directories(executor-java-api PUBLIC ${JNI_INCLUDE_DIRS} ${oRatio_SOURCE_DIR}/smt ${oRatio_SOURCE_DIR}/smt/lra ${oRatio_SOURCE_DIR}/smt/ov ${oRatio_SOURCE_DIR}/smt/dl ${oRatio_SOURCE_DIR}/core ${oRatio_SOURCE_DIR}/solver ${oRatio_SOURCE_DIR}/executor)
    target_link_libraries(executor-java-api PUBLIC solver-java-api executor)
    install(TARGETS executor-java-api LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif(BUILD_EXECUTOR)
