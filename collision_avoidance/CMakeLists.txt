cmake_minimum_required(VERSION 3.0.2)
project(collision_avoidance)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  moveit_core
  rviz_visual_tools
  moveit_visual_tools
  moveit_ros_planning_interface
  moveit_ros_move_group
  interactive_markers
  tf2_geometry_msgs
  moveit_ros_perception
  moveit_ros_planning
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  actionlib_msgs
  control_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

#find_package(moveit_common REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES collision_avoidance
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    actionlib_msgs
    control_msgs
  DEPENDS
    EIGEN3
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(add_labTable src/add_labTable.cpp)
add_dependencies(add_labTable ${add_labTable_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(add_labTable ${catkin_LIBRARIES})

add_executable(algoritmo_prevenzione_collisioni src/algoritmo_prevenzione_collisioni.cpp)
add_dependencies(algoritmo_prevenzione_collisioni ${algoritmo_prevenzione_collisioni_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(algoritmo_prevenzione_collisioni ${catkin_LIBRARIES})

add_executable(move_obstacle_case2 src/move_obstacle_case2.cpp)
add_dependencies(move_obstacle_case2 ${move_obstacle_case2_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_obstacle_case2 ${catkin_LIBRARIES})

add_executable(move_obstacle_case1 src/move_obstacle_case1.cpp)
add_dependencies(move_obstacle_case1 ${move_obstacle_case1_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_obstacle_case1 ${catkin_LIBRARIES})

add_executable(move_obstacle_case3 src/move_obstacle_case3.cpp)
add_dependencies(move_obstacle_case3 ${move_obstacle_case3_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_obstacle_case3 ${catkin_LIBRARIES})
