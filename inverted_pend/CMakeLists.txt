cmake_minimum_required(VERSION 3.0.2)
project(inverted_pend)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  State_msg.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Unpause.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

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
 INCLUDE_DIRS include
 LIBRARIES inverted_pend
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(inverted_pend_node
  include/plant_dynamics/plant_dynamics.cpp
  include/ros_wrapper/ros_plant_wrapper.cpp
  src/plant_node.cpp
)

target_link_libraries(inverted_pend_node
  ${catkin_LIBRARIES}
)

# controller node
add_executable(inverted_pend_controller
  include/controller/inverted_pend_controller.cpp
  include/controller/pid_controller.cpp
  include/controller/state_feedback_controller.cpp
  include/ros_wrapper/ros_controller.cpp
  src/controller_node.cpp
)

target_link_libraries(inverted_pend_controller
  ${catkin_LIBRARIES}
)