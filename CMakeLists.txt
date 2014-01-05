cmake_minimum_required(VERSION 2.8.3)
project(roomba_sci)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  actionlib
  actionlib_msgs
  tf
  nav_msgs
  geometry_msgs
  cereal_port
  message_generation
  )

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
file(GLOB message_source "msg/*.msg")
list(SORT message_source)
foreach(message_src_file ${message_source})
    get_filename_component(message_base ${message_src_file} NAME)
    message(STATUS "Adding message ${message_base}" )
    add_message_files(DIRECTORY msg FILES ${message_base})
endforeach(message_src_file)

## Generate services in the 'srv' folder
file(GLOB service_source "srv/*.srv")
list(SORT service_source)
foreach(service_src_file ${service_source})
    get_filename_component(service_base ${service_src_file} NAME)
    message(STATUS "Adding service ${service_base}" )
    add_service_files(DIRECTORY srv FILES ${service_base})
endforeach(service_src_file)

## Generate actions in the 'action' folder
file(GLOB action_source "action/*.action")
list(SORT action_source)
foreach(action_src_file ${action_source})
    get_filename_component(action_base ${action_src_file} NAME)
    message(STATUS "Adding action ${action_base}" )
    add_action_files(DIRECTORY action FILES ${action_base})
endforeach(action_src_file)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  nav_msgs
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cereal_port
  CATKIN_DEPENDS
  roscpp
  actionlib
  actionlib_msgs
  tf
  nav_msgs
  geometry_msgs
  cereal_port
  message_runtime

  DEPENDS system_lib  
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_libraries(
  ${catkin_LIBRARIES}
)

add_library(irobot_oi src/iRobot/OpenInterface.cpp)
add_dependencies(irobot_oi ${PROJECT_NAME}_generate_messages_cpp)
add_library(go_dock src/GoDock/GoDock.cpp)
add_dependencies(go_dock ${PROJECT_NAME}_generate_messages_cpp)

add_executable(roomba560_node src/roomba560.cpp src/iRobot/OpenInterface.cpp src/GoDock/GoDock.cpp)
target_link_libraries(roomba560_node cereal_port)
add_dependencies(roomba560_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(roomba500_light_node src/roomba500_lightweight.cpp src/iRobot/OpenInterface.cpp)
target_link_libraries(roomba500_light_node cereal_port)
add_dependencies(roomba500_light_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(godock_server src/godock_server.cpp src/GoDock/GoDock.cpp)
add_dependencies(godock_server ${PROJECT_NAME}_generate_messages_cpp)
add_executable(godock_client src/godock_client.cpp)
add_dependencies(godock_client ${PROJECT_NAME}_generate_messages_cpp)

