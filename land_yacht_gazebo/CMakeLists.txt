cmake_minimum_required(VERSION 3.0.2)
project(land_yacht_gazebo)

find_package(catkin REQUIRED COMPONENTS
    rospy
)

catkin_package()

install(
  DIRECTORY launch models scripts worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
