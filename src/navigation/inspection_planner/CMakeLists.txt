cmake_minimum_required(VERSION 3.8)
project(inspection_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(inspection_planner_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# TSP Solver executable
add_executable(tsp_solver src/tsp_solver.cpp)
target_include_directories(tsp_solver PRIVATE include)
ament_target_dependencies(tsp_solver
  rclcpp
  inspection_planner_interfaces
  geometry_msgs
  std_msgs
)

# Install executables
install(TARGETS
  tsp_solver
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

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_message_definitions test/test_message_definitions.py)
  ament_add_pytest_test(test_tsp_integration test/test_tsp_integration.py)
  ament_add_pytest_test(test_tsp_distance_matrix test/test_tsp_distance_matrix.py)
endif()

ament_package()
