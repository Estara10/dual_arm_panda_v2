cmake_minimum_required(VERSION 3.5)
project(dual_arm_description)

# 寻找依赖
find_package(ament_cmake REQUIRED)

# 安装 URDF 文件夹到共享目录 (非常重要，否则 Gazebo/MoveIt 找不到模型)
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()