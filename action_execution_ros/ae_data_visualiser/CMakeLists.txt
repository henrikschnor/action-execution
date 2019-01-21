cmake_minimum_required(VERSION 2.8.3)
project(ae_data_visualiser)

find_package(catkin REQUIRED COMPONENTS
  rospy
  visualization_msgs
  ae_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    visualization_msgs
    ae_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
