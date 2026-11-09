cmake_minimum_required(VERSION 3.8)
project(lidar_intensity_assist)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(pcd_correction_node src/main.cpp src/lidar_intensity_match.cpp)
target_include_directories(pcd_correction_node PRIVATE include ${PCL_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(pcd_correction_node rclcpp sensor_msgs nav_msgs std_msgs tf2)
target_link_libraries(pcd_correction_node ${CERES_LIBRARIES} ${PCL_LIBRARIES} OpenMP::OpenMP_CXX ${OpenCV_LIBS})

install(TARGETS pcd_correction_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_package()
