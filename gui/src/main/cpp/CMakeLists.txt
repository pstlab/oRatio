find_package(JNI REQUIRED)

add_executable(solver_server main.cpp)

target_include_directories(solver_server PRIVATE ${JNI_INCLUDE_DIRS} ${oRatio_SOURCE_DIR}/smt ${oRatio_SOURCE_DIR}/smt/lra ${oRatio_SOURCE_DIR}/smt/ov ${oRatio_SOURCE_DIR}/smt/dl ${oRatio_SOURCE_DIR}/core ${oRatio_SOURCE_DIR}/solver)
target_link_libraries(solver_server PRIVATE ${JAVA_JVM_LIBRARY} solver) # remember to add jdk\bin\server to the path system variable!!