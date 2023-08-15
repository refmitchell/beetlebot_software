cmake_minimum_required(VERSION 2.8.3)
project(bb_locomotion)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  message_generation
  bb_util
)

# add_service_files(
#   FILES
# #  velocity.srv
# )

# generate_messages(
#   DEPENDENCIES
#   )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES locomotion
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs bb_util
#  DEPENDS system_lib
)

## Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${bb_util_INCLUDE_DIRS}
)

add_executable(cmd_vel_service src/cmd_vel_service.cpp)

add_dependencies(
  cmd_vel_service
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${bb_util_EXPORTED_TARGETS}
  )

target_link_libraries(cmd_vel_service ${catkin_LIBRARIES})

