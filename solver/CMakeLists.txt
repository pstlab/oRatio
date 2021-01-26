set(TEMPORAL_NETWORK_TYPES DL LA)
set(TEMPORAL_NETWORK_TYPE LA CACHE STRING "Temporal network type")
set_property(CACHE TEMPORAL_NETWORK_TYPE PROPERTY STRINGS ${TEMPORAL_NETWORK_TYPES})

set(HEURISTIC_TYPES h_max h_add)
set(HEURISTIC_TYPE "h_max" CACHE STRING "Heuristic type")
set_property(CACHE HEURISTIC_TYPE PROPERTY STRINGS "h_max" "h_add")

file(GLOB SOLVER_SOURCES *.cpp types/*.cpp)

add_library(solver ${SOLVER_SOURCES})
target_include_directories(solver PRIVATE . ../smt ../smt/lra ../smt/ov ../smt/dl ../core ../riddle types)
target_link_libraries(solver PRIVATE core)

message(STATUS "Temporal network type:    ${TEMPORAL_NETWORK_TYPE}")
if(TEMPORAL_NETWORK_TYPE MATCHES DL)
    add_compile_definitions(DL_TN)
    configure_file(init_dl.rddl ${CMAKE_BINARY_DIR}/init.rddl COPYONLY)
elseif(TEMPORAL_NETWORK_TYPE MATCHES LA)
    add_compile_definitions(LA_TN)
    configure_file(init_la.rddl ${CMAKE_BINARY_DIR}/init.rddl COPYONLY)
else()
    message(FATAL_ERROR "TEMPORAL_NETWORK_TYPE must be one of ${TEMPORAL_NETWORK_TYPES}")
endif()