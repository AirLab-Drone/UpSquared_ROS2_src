cmake_minimum_required(VERSION 3.8)
project(coin417rg2_thermal)


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# ------------------------------ 根據系統架構選擇不同的編譯配置 ----------------------------- #
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  message(STATUS "Target system is x86_64")
  # 在這裡添加 x64 特定的編譯配置
  link_directories(lib/x86_64)
  include_directories(include/coin417rg2_thermal/x86_64)


elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "Target system is ARM or AARCH64")
  # 在這裡添加 ARM 或 AARCH64 特定的編譯配置
  link_directories(lib/arm)
  include_directories(include/coin417rg2_thermal/arm)

  file(GLOB ARM_LIBRARIES "lib/arm/*.so")
  foreach(lib ${ARM_LIBRARIES})
    message(STATUS "Found library: ${lib}")
  endforeach()



else()
  message(STATUS "Unknown processor architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()



# ---------------------------------- 找套件 --------------------------------- #
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)






# target_link_libraries
# 用於指定目標（例如可執行文件或庫）依賴的庫文件。
# 通常用於將目標與其他庫文件連接，以解析符號引用並生成最終的可執行文件或庫。
# 這個命令是 CMake 的內置命令，用於指定目標的連接器選項。


# ament_target_dependencies
# 用於指定目標依賴的 ROS 套件
# 這個命令是由 ament_cmake package 提供的功能，用於指定 ROS 2 packages 之間的依賴關係。
# 它會確保目標在編譯時能夠找到所需的 ROS 2 套件，並將其包含在最終的目標中。 

# /* ----------------------------------- 測試用 ---------------------------------- */
add_executable(test_node src/test.cpp)
target_link_libraries(test_node 
    ${ARM_LIBRARIES}
    GuideUSBCamera -ldl
    GuideMT -ldl
    pthread
    m
    udev
    
)
ament_target_dependencies(test_node 
    rclcpp 
    std_msgs 
    sensor_msgs 
    cv_bridge 
    image_transport 
    OpenCV)
 

# /* -------------------------------- ros2 node ------------------------------- */
add_executable(COIN417RG2_ros2_node src/COIN417RG2_ros2_node.cpp)
target_link_libraries(COIN417RG2_ros2_node 
    ${ARM_LIBRARIES}
    GuideUSBCamera -ldl
    GuideMT -ldl
    pthread
    m
    udev
)
ament_target_dependencies(COIN417RG2_ros2_node 
    rclcpp 
    std_msgs 
    sensor_msgs 
    cv_bridge 
    image_transport 
    OpenCV)

#/* ----------------------- thermal fram to drone frame ---------------------- */
add_executable(thermal_frame_to_drone_frame src/thermal_frame_to_drone_frame.cpp)
ament_target_dependencies(thermal_frame_to_drone_frame 
    rclcpp 
    std_msgs 
    sensor_msgs 
    yaml-cpp
)



install(TARGETS
  test_node
  COIN417RG2_ros2_node
  thermal_frame_to_drone_frame
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  lib
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
