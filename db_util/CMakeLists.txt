cmake_minimum_required(VERSION 2.8.3)
project(db_util)

# Source dependencies
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  OpenCV
  roscpp
  std_msgs
  message_generation
)

add_service_files(
  FILES
  velocity.srv
  locomotion_cmd.srv
  )

add_message_files(
  FILES
  test_population.msg
  cx_activity.msg
  vmcx_activity.msg
  cue_msg.msg
  cue_list.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

# Describe the catkin_package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES db_util
  CATKIN_DEPENDS cv_bridge roscpp std_msgs
  DEPENDS system_lib
)

# Define the include directories usd by this project
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

add_library(
  db_util
  src/libdbutil.cpp
)

# db_util dependencies
add_dependencies(
  db_util
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Libs for any nodes built here
target_link_libraries(
  db_util
  ${catkin_LIBRARIES}
  ${OpenCV_Libs}
  stdc++fs
 )

# install header files from this package globally
# I plan to make this header only for ease sake
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN "*.svn" EXCLUDE
)

