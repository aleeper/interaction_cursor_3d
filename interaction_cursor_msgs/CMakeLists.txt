cmake_minimum_required(VERSION 2.8.3)
project(interaction_cursor_msgs)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(MSG_DEPS
    geometry_msgs 
    std_msgs 
    visualization_msgs)


find_package(catkin REQUIRED COMPONENTS 
              genmsg ${MSG_DEPS}) 

 add_message_files(DIRECTORY msg FILES
   InteractionCursorUpdate.msg
   InteractionCursorFeedback.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(CATKIN_DEPENDS ${MSG_DEPS})
