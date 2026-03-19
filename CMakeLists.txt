cmake_minimum_required(VERSION 3.8)
project(g1_slam)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  g1_slam/restamp_cloud.py
  g1_slam/restamp_odom.py
  g1_slam/odom_to_tf.py
  g1_slam/pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()