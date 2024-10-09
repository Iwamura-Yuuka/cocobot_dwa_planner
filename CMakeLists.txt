cmake_minimum_required(VERSION 3.0.2)
project(cocobot_dwa_planner)

add_compile_options(-std=c++11 -O3 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf
  tf2

  pedestrian_msgs
  ccv_dynamixel_msgs
  dynamixel_workbench_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cocobot_dwa_planner
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/cocobot_dwa_planner.cpp)
add_executable(${PROJECT_NAME}_node src/cocobot_dwa_planner_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})