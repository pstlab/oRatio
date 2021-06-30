find_package(Python COMPONENTS Development REQUIRED)

add_library(solver-python-api SHARED solver.cpp)
target_include_directories(solver-python-api PUBLIC ${Python_INCLUDE_DIRS} .)
target_link_libraries(solver-python-api PUBLIC solver)
