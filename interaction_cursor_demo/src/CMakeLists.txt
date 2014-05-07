add_subdirectory(interaction_cursor_demo)

add_executable(interaction_cursor_demo_node
               main.cpp
)
add_dependencies(interaction_cursor_demo_node ${PROJECT_NAME})

target_link_libraries(interaction_cursor_demo_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)



install(TARGETS interaction_cursor_demo_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
