cmake_minimum_required(VERSION 2.8.3)
project(interaction_cursor_msgs)

set(MSG_DEPS
    geometry_msgs
    std_msgs
    visualization_msgs)


find_package(catkin REQUIRED COMPONENTS
              genmsg ${MSG_DEPS} message_generation)

 add_message_files(DIRECTORY msg FILES
   InteractionCursorUpdate.msg
   InteractionCursorFeedback.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(CATKIN_DEPENDS ${MSG_DEPS} message_runtime)
