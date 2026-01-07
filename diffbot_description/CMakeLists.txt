cmake_minimum_required(VERSION 3.16)
project(diffbot_description)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

