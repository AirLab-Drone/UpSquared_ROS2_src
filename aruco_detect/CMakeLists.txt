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

# Install Python modules
ament_python_install_package(${PROJECT_NAME}_py)
# Install Python executables
file(GLOB SCRIPTS "scripts/*.py")

install(PROGRAMS
  ${SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   src/service_name.srv
#   DEPENDENCIES std_msgs
# )

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
