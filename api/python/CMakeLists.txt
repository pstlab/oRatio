find_package(pybind11 CONFIG REQUIRED)

file(GLOB PY_SOURCES *.cpp)

add_library(pyRatio MODULE ${PY_SOURCES})
target_include_directories(pyRatio PUBLIC .)
target_link_libraries(pyRatio PUBLIC pybind11::module solver)