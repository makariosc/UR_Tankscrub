cmake_minimum_required(VERSION 3.1.3)
project(tankscrub)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf
    tf2
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    tf2_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

set(THIS_PACKAGE_INCLUDE_DIRS
  doc/interactivity/include
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_msgs
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(move_group_interface_tutorial src/move_group_interface_tutorial.cpp)
target_link_libraries(move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ur5_move_group_interface_demo src/ur5_move_group_test_plan.cpp)
target_link_libraries(ur5_move_group_interface_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ur5_hardcoded_planner src/ur5_hardcoded_planner.cpp)
target_link_libraries(ur5_hardcoded_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ur5_sim_test_plan src/ur5_sim_test_plan.cpp)
target_link_libraries(ur5_sim_test_plan ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(set_gazebo_state_test src/set_gazebo_state_test.cpp)
target_link_libraries(set_gazebo_state_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})