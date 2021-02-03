find_package(JNI)

add_executable(solver-server main.cpp)

target_include_directories(solver-server PRIVATE ${JNI_INCLUDE_DIRS})
target_link_libraries(solver-server PRIVATE ${JAVA_JVM_LIBRARY})