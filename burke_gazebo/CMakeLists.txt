cmake_minimum_required(VERSION 3.22)
project(burke_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_add_pytest_test(test_arm_topics test/test_arm_topics.py
  TIMEOUT 120
)

ament_add_pytest_test(test_lidar_topics test/test_lidar_topics.py
  TIMEOUT 120
)

ament_package()
