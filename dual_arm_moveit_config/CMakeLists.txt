cmake_minimum_required(VERSION 3.5)
project(dual_arm_moveit_config)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# 安装 config 和 launch 文件夹
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()