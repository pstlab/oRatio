file(GLOB SMT_SOURCES *.cpp arith/*.cpp arith/lra/*.cpp arith/dl/*.cpp ov/*.cpp)
file(GLOB SMT_HEADERS *.h arith/*.h arith/lra/*.h arith/dl/*.h ov/*.h)

add_library(smt SHARED ${SMT_SOURCES})
GENERATE_EXPORT_HEADER(smt)
target_include_directories(smt PUBLIC ${CMAKE_CURRENT_BINARY_DIR} . arith arith/lra arith/dl ov)
target_link_libraries(smt PUBLIC json)

if(PARALLELIZE)
    target_include_directories(smt PUBLIC concurrent)
    target_link_libraries(smt PUBLIC concurrent)
endif()

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

if(MSVC)
    target_compile_options(smt PRIVATE /W4)
else()
    target_compile_options(smt PRIVATE -Wall -Wextra -pedantic)
endif()

install(TARGETS smt LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${SMT_HEADERS} ${CMAKE_CURRENT_BINARY_DIR}/smt_export.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/smt)