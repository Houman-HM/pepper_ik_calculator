cmake_minimum_required(VERSION 2.8.3)
project(pepper_ik_calculator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    rospy
    std_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    tf2_ros
    tf2_eigen
)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planning_tutorial
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})




add_executable(ik_calculator src/ik_calculator.cpp)
target_link_libraries(ik_calculator ${catkin_LIBRARIES} ${Boost_LIBRARIES})
