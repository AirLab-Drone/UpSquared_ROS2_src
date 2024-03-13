cmake_minimum_required(VERSION 3.8)
project(small_thermal)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++17
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()


include_directories(include/small_thermal)

link_directories(lib)


# ---------------------------------- 找套件 --------------------------------- #
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)





add_executable(test_small_thermal_node src/test_small_thermal.cpp)
target_link_libraries(test_small_thermal_node
  GuideUSBCamera
)
ament_target_dependencies(test_small_thermal_node rclcpp)


# install(TARGETS
#   test_small_thermal_node
#   DESTINATION lib/${PROJECT_NAME}
# )




# install(
#   DIRECTORY src/${PROJECT_NAME}
#   DESTINATION include
# )

# install(
#   TARGETS ${PROJECT_NAME}
#   EXPORT ${PROJECT_NAME}Targets
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )




ament_package()
