cmake_minimum_required(VERSION 3.8)
project(aruco_detect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# for C++ node
install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

# for python node
ament_python_install_package(${PROJECT_NAME}) # for importing the package in "aruco_detect/aruco_detect"

install(PROGRAMS
  scripts/camera_node.py
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
