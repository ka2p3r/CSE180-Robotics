cmake_minimum_required(VERSION 3.5)
project(CSE180_FinalProject)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(navigator src/navigation.cpp)
add_executable(ournav src/ournav.cpp src/navigation.cpp)
add_executable(testpackage src/testpackage.cpp src/navigation.cpp)

target_include_directories(navigator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_include_directories(ournav
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_include_directories(testpackage
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(navigator rclcpp geometry_msgs lifecycle_msgs rclcpp_action nav2_msgs)
ament_export_targets(navigatorlib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs lifecycle_msgs rclcpp_action nav2_msgs)
ament_target_dependencies(ournav rclcpp geometry_msgs lifecycle_msgs rclcpp_action nav2_msgs tf2_ros tf2_geometry_msgs sensor_msgs) # Add sensor_msgs here
ament_target_dependencies(testpackage rclcpp geometry_msgs lifecycle_msgs rclcpp_action nav2_msgs)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS navigator
  EXPORT navigatorlib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )

install(TARGETS ournav DESTINATION lib/${PROJECT_NAME})
install(TARGETS testpackage DESTINATION lib/${PROJECT_NAME})

ament_package()
