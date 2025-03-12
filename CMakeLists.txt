cmake_minimum_required(VERSION 3.8)
project(kurome C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-gdwarf-4 -Wall -Wextra -Wpedantic -std=c++17)
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
find_package(tf2_geometry_msgs REQUIRED)

find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Ceres REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

find_package(SFML COMPONENTS window graphics system)

# generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
   "msg/Rectangle.msg"
   "msg/CompetitionArea.msg"
   DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(kurome_base STATIC src/kurome.cpp)
target_link_libraries(kurome_base Eigen3::Eigen)
ament_target_dependencies(kurome_base rclcpp sensor_msgs geometry_msgs visualization_msgs nav_msgs 
                          tf2 tf2_ros tf2_geometry_msgs)

# brain node
add_executable(brain src/brain.cpp)
target_link_libraries(brain kurome_base "${cpp_typesupport_target}")
ament_target_dependencies(brain rclcpp sensor_msgs geometry_msgs visualization_msgs nav_msgs tf2 tf2_ros)

# wall filter node
add_executable(wall_filter src/wall_filter.cpp)
ament_target_dependencies(wall_filter rclcpp sensor_msgs geometry_msgs visualization_msgs
                          nav_msgs tf2 tf2_ros tf2_sensor_msgs)

# pino node
add_executable(pino src/pino.cpp)
target_link_libraries(pino graph_slam_backend Ceres::ceres Eigen3::Eigen kurome_base)
ament_target_dependencies(pino rclcpp sensor_msgs geometry_msgs visualization_msgs nav_msgs
                          tf2 tf2_ros tf2_geometry_msgs tf2_sensor_msgs)

# yoriko node
add_executable(yoriko src/yoriko.cpp)
target_link_libraries(yoriko sfml-window sfml-graphics sfml-system kurome_base)
ament_target_dependencies(yoriko rclcpp sensor_msgs geometry_msgs visualization_msgs nav_msgs 
                          tf2 tf2_ros tf2_geometry_msgs)

# misao node
add_executable(misao src/misao.cpp)
target_link_libraries(misao sfml-window sfml-graphics sfml-system kurome_base)
ament_target_dependencies(misao rclcpp sensor_msgs geometry_msgs visualization_msgs nav_msgs 
                          tf2 tf2_ros tf2_geometry_msgs)

# the iron rod node 
add_executable(the_iron_rod src/the_iron_rod.cpp)
target_link_libraries(the_iron_rod kurome_base)
ament_target_dependencies(the_iron_rod rclcpp geometry_msgs nav_msgs sensor_msgs visualization_msgs)

# the hardcoded path publisher 
add_executable(hardcoded_path_publisher src/hardcoded_path_publisher.cpp)
ament_target_dependencies(hardcoded_path_publisher rclcpp nav_msgs geometry_msgs)

# the matcher test
add_executable(matcher_test src/matcher_test.cpp)
target_link_libraries(matcher_test kurome_base)
ament_target_dependencies(matcher_test rclcpp sensor_msgs)

# the pose publisher
add_executable(pose_publisher src/pose_publisher.cpp)
target_link_libraries(pose_publisher kurome_base)
ament_target_dependencies(pose_publisher rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# the local mask
add_executable(local_mask src/local_mask.cpp)
target_link_libraries(local_mask kurome_base)
ament_target_dependencies(local_mask rclcpp sensor_msgs tf2 tf2_ros tf2_geometry_msgs)



install(TARGETS brain wall_filter pino yoriko misao
                the_iron_rod hardcoded_path_publisher matcher_test pose_publisher local_mask
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})

ament_package()
