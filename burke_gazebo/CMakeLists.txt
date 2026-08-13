cmake_minimum_required(VERSION 3.22)
project(burke_gazebo)

find_package(ament_cmake REQUIRED)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
