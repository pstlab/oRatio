file(GLOB SMT_SOURCES *.cpp arith/*.cpp arith/lra/*.cpp arith/dl/*.cpp ov/*.cpp)

add_library(smt SHARED ${SMT_SOURCES})
GENERATE_EXPORT_HEADER(smt)
target_include_directories(smt PUBLIC ${CMAKE_CURRENT_BINARY_DIR} . arith arith/lra arith/dl ov)
target_link_libraries(smt PUBLIC json)
install(TARGETS smt LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

if(PARALLELIZE)
    target_include_directories(smt PUBLIC concurrent)
    target_link_libraries(smt PUBLIC concurrent)
endif()

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()