cmake_minimum_required(VERSION 3.8)
project(planners)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)

# Needed to find the .so file of px4_ros_com package
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

########### Add include directory for Eigen ###################
include_directories(
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>)

add_executable(contact_based_reference_finder src/contact_based_reference_finder.cpp)
ament_target_dependencies(contact_based_reference_finder rclcpp geometry_msgs sensor_msgs custom_interfaces std_srvs px4_msgs px4_ros_com Eigen3)
target_include_directories(contact_based_reference_finder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/planners>
  $<INSTALL_INTERFACE:include>)
target_compile_features(contact_based_reference_finder PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(fake_wrench_publisher src/fake_wrench_publisher.cpp)
ament_target_dependencies(fake_wrench_publisher rclcpp geometry_msgs)

add_executable(ee_position_test src/ee_position_test.cpp)
ament_target_dependencies(ee_position_test rclcpp sensor_msgs geometry_msgs std_srvs Eigen3 px4_msgs px4_ros_com custom_interfaces)

add_executable(base_reference_position_publisher src/base_reference_position_publisher.cpp)
ament_target_dependencies(base_reference_position_publisher rclcpp geometry_msgs px4_msgs px4_ros_com Eigen3)
target_include_directories(base_reference_position_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/planners>
  $<INSTALL_INTERFACE:include>)

install(TARGETS contact_based_reference_finder base_reference_position_publisher fake_wrench_publisher ee_position_test
  DESTINATION lib/${PROJECT_NAME})

########### Add Launch Directory ########
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

########## Add Config Directory #########
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
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
