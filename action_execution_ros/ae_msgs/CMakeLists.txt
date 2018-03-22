cmake_minimum_required(VERSION 2.8.3)
project(ae_msgs)

find_package(catkin REQUIRED
  message_generation
)
add_message_files(
  FILES
  ActionDataQuery.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package()
include_directories()
