add_executable(oRatioServer main.cpp gui_server.cpp)

find_package(Crow)

file(GLOB GUI_STATIC static/*.css static/*.js)
file(GLOB GUI_TEMPLATES templates/*.html)

target_include_directories(oRatioServer PRIVATE .)
target_link_libraries(oRatioServer PRIVATE solver executor Crow::Crow)
install(TARGETS oRatioServer RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES ${GUI_STATIC} DESTINATION ${CMAKE_INSTALL_BINDIR}/static)
install(FILES ${GUI_TEMPLATES} DESTINATION ${CMAKE_INSTALL_BINDIR}/templates)