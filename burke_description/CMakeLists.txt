cmake_minimum_required(VERSION 3.22)
project(burke_description)

find_package(ament_cmake REQUIRED)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY cad config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
