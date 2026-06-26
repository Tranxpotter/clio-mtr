cmake_minimum_required(VERSION 3.8)
project(cloud_aggregator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(cloud_aggregator_node src/cloud_aggregator_node.cpp)
ament_target_dependencies(cloud_aggregator_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(cloud_aggregator_node ${PCL_LIBRARIES})

install(TARGETS
  cloud_aggregator_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
