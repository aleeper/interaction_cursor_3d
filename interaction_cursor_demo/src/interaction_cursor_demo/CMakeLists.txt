include_directories(.)

add_library(${PROJECT_NAME}
  abstract_handle.cpp
  tf_scenegraph_object.cpp
  camera_node.cpp
  manipulator_node.cpp
  user_entity.cpp
  abstract_interaction_tool.cpp
  #haptic_interaction_tool.cpp
  hydra_interaction_tool.cpp
)

add_dependencies(${PROJECT_NAME} interaction_cursor_msgs)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
