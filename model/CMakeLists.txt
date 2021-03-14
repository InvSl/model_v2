cmake_minimum_required(VERSION 2.8.3)
project(model)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  move_base_msgs
  actionlib
  costmap_2d
  message_generation
  geometry_msgs
  nav_msgs
)


add_service_files(
  FILES
  TakeMap.srv
  TakePose.srv
)

generate_messages(
  DEPENDENCIES
  nav_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES model
  CATKIN_DEPENDS 
    roscpp
    message_runtime
    nav_msgs
    geometry_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



include_directories(include ${catkin_INCLUDE_DIRS})

# TF -----------------------------------------------------
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})


# Services -----------------------------------------------

add_executable(map_service src/map_service.cpp)
target_link_libraries(map_service ${catkin_LIBRARIES})

add_executable(pose_service src/pose_service.cpp)
target_link_libraries(pose_service ${catkin_LIBRARIES})



# Task ---------------------------------------------------
add_library(mapper src/mapper.cpp)
add_library(controller src/controller.cpp)

target_link_libraries(controller mapper)

add_executable(task src/task.cpp)
target_link_libraries(task ${catkin_LIBRARIES})
target_link_libraries(task controller)