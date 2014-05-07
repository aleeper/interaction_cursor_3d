cmake_minimum_required(VERSION 2.8.3)
project(interaction_cursor_rviz)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs interaction_cursor_msgs rviz)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(PkgConfig REQUIRED)

# Ogre
pkg_check_modules(OGRE OGRE)

# Qt Stuff
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
catkin_python_setup()

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES interaction_cursor_rviz
  CATKIN_DEPENDS geometry_msgs interaction_cursor_msgs rviz
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${QT_INCLUDE_DIR}
                    ${OGRE_INCLUDE_DIRS}
                    )

# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set( headers
  include/interaction_cursor_rviz/interaction_cursor.h
)

# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES ${headers})

# Plugin Source
set(SOURCE_FILES
  src/interaction_cursor.cpp
)

add_library(interaction_cursor_rviz ${SOURCE_FILES} ${MOC_SOURCES})
add_dependencies(interaction_cursor_rviz interaction_cursor_msgs_gencpp)
target_link_libraries(interaction_cursor_rviz ${catkin_LIBRARIES} ${OGRE_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES})


#############
## Install ##
#############

install(TARGETS interaction_cursor_rviz LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES  interaction_cursor_rviz_plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


