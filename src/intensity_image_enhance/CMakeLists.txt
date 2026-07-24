cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(intensity_image_enhance)

find_package(Ceres REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL 1.10 REQUIRED COMPONENTS common io)
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)

# 指定正确的 include 目录
catkin_package(
  # INCLUDE_DIRS include  # 现在这个目录存在了
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions
  DEPENDS PCL
)

include_directories(
  include              # 添加这一行
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(WORKSPACE_DEVEL_LIB_DIR "${CMAKE_SOURCE_DIR}/../devel/lib/${PROJECT_NAME}")
file(MAKE_DIRECTORY "${WORKSPACE_DEVEL_LIB_DIR}")

add_executable(pcd_correction_node src/computeT_deskew_current_sample_scaled.cpp)
set_target_properties(pcd_correction_node PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${WORKSPACE_DEVEL_LIB_DIR}"
)
target_include_directories(pcd_correction_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(pcd_correction_node
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
  ${OpenCV_LIBS}
)

install(TARGETS pcd_correction_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

# 安装头文件（如果需要）
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
