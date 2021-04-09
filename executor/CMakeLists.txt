find_package(Threads REQUIRED)

add_library(executor SHARED executor.cpp)
target_include_directories(executor PUBLIC ${oRatio_SOURCE_DIR}/smt ${oRatio_SOURCE_DIR}/smt/lra ${oRatio_SOURCE_DIR}/smt/ov ${oRatio_SOURCE_DIR}/smt/dl ${oRatio_SOURCE_DIR}/core ${oRatio_SOURCE_DIR}/solver)
target_link_libraries(executor PUBLIC solver)
install(TARGETS executor LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})