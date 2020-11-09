cmake_minimum_required(VERSION 3.0.2)
project(land_yacht_gazebo)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(
  DIRECTORY models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
