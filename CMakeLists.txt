cmake_minimum_required(VERSION 2.8.3)
project(dvrk-gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  )

find_package(gazebo REQUIRED)


include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES cisst_ros_bridge
  # CATKIN_DEPENDS message_runtime
  # DEPENDS system_lib
)

add_definitions(-std=c++11)

add_executable(gazebo_move
  scripts/gazebo_move.cpp)

add_executable(auto_move
  scripts/auto_move.cpp)

add_executable(program_move
  scripts/program_move.cpp)

add_executable(labirintus
  scripts/labirintus.cpp)

add_executable(connection
  scripts/connection.cpp)

target_link_libraries (auto_move ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (gazebo_move ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (program_move ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (labirintus ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (connection ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
