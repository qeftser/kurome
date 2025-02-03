cmake_minimum_required(VERSION 3.8)
project(kurome)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -g)
endif()

# install dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(Ceres REQUIRED)

# brain node
add_executable(brain src/brain.cpp)
ament_target_dependencies(brain rclcpp sensor_msgs geometry_msgs visualization_msgs nav_msgs tf2 tf2_ros)

# wall filter node
add_executable(wall_filter src/wall_filter.cpp)
ament_target_dependencies(wall_filter rclcpp sensor_msgs geometry_msgs visualization_msgs
                          nav_msgs tf2 tf2_ros tf2_sensor_msgs)

# pino node
add_executable(pino src/pino.cpp)
target_link_libraries(pino Ceres::ceres)
ament_target_dependencies(pino rclcpp sensor_msgs geometry_msgs visualization_msgs nav_msgs tf2 tf2_ros)

# yoriko node
add_executable(yoriko src/yoriko.cpp)
ament_target_dependencies(yoriko rclcpp sensor_msgs geometry_msgs visualization_msgs nav_msgs tf2 tf2_ros)

install(TARGETS brain wall_filter pino yoriko
        DESTINATION lib/${PROJECT_NAME})

ament_package()
