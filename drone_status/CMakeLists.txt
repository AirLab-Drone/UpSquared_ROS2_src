cmake_minimum_required(VERSION 3.8)
project(drone_status)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)



ament_python_install_package(${PROJECT_NAME}_py)
# Install Python executables
file(GLOB SCRIPTS "scripts/*.py")
install(PROGRAMS
  ${SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)


rosidl_generate_interfaces(${PROJECT_NAME}
  srv/CheckUSBDevices.srv
)



install(DIRECTORY
  # launch
  # lib
  # config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
