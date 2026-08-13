cmake_minimum_required(VERSION 3.8)
project(pcl_demo_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

# 查找 PCL 并指定所需组件
# 注意：range_image 用于 NARF 关键点，features 用于法线计算（SUSAN 需要），keypoints 必不可少
find_package(PCL REQUIRED COMPONENTS 
  common 
  io 
  filters 
  sample_consensus 
  surface 
  keypoints 
  features 
  range_image
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_demo_1_node src/pcl_demo1_node.cpp)
ament_target_dependencies(pcl_demo_1_node 
  rclcpp 
  std_msgs 
  sensor_msgs 
  pcl_ros 
  pcl_conversions
)

target_link_libraries(pcl_demo_1_node ${PCL_LIBRARIES})

install(TARGETS
  pcl_demo_1_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()