cmake_minimum_required(VERSION 2.8.3)
project(ant_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros 
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs    
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS 
    sensor_msgs
    gazebo_ros 
  DEPENDS 
    gazebo
)
