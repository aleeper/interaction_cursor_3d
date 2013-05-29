add_subdirectory(interaction_cursor_demo)
include_directories(.)

add_executable(user_test
               main.cpp
)
add_dependencies(user_test ${PROJECT_NAME})

target_link_libraries(user_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
