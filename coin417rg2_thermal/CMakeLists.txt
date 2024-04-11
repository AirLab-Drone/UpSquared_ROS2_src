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


# ---------------------------------- 找套件 --------------------------------- #
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)



link_directories(lib)
include_directories(include/coin417rg2_thermal)



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


install(TARGETS
  test_node
  COIN417RG2_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
