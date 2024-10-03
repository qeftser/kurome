cmake_minimum_required(VERSION 3.15)

project(kurome VERSION 0.3 LANGUAGES CXX C)

# set(SFML_STATIC_LIBRARIES TRUE)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(SFML COMPONENTS window graphics system)

add_library(k_lib STATIC src/Grid.cpp src/Iterators.cpp src/Frame.cpp src/Agent.cpp src/Reporter.cpp
                         src/gsocket.cpp src/handlers.cpp src/kcmd.cpp)

add_executable(serv src/sserv.cpp)
add_executable(viewer src/gui/viewer/viewer.cpp)

# target_compile_definitions(viewer PRIVATE SFML_STATIC)

target_include_directories(serv PUBLIC src/headers src/mappers)
target_include_directories(viewer PUBLIC src/headers src/waiters)

target_link_libraries(serv PUBLIC k_lib Eigen3::Eigen)
target_link_libraries(viewer PUBLIC k_lib Eigen3::Eigen sfml-window sfml-graphics sfml-system)
