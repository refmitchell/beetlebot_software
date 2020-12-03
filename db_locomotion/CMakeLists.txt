cmake_minimum_required(VERSION 2.8.3)
project(locomotion)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  message_generation
  db_util
)

add_service_files(
  FILES
  velocity.srv
)

generate_messages(
  DEPENDENCIES
  )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES locomotion
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs db_util
#  DEPENDS system_lib
)

## Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${db_util_INCLUDE_DIRS}
)


add_executable(wakeup src/wakeup.cpp)
add_executable(cmd_vel_service src/cmd_vel_service.cpp)

add_dependencies(wakeup ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(
  cmd_vel_service
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${db_util_EXPORTED_TARGETS}
  )


target_link_libraries(wakeup ${catkin_LIBRARIES})
target_link_libraries(cmd_vel_service ${catkin_LIBRARIES})
