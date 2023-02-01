cmake_minimum_required(VERSION 3.8)
project(sensors_and_observers)

set (CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

########### Add include directory for Eigen ###################
include_directories(
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>)

########### Executable for encoder node #######################
add_executable(encoder_node src/encoder_node.cpp
                            src/cui-devices-amt-10x-driver/cui_devices_amt10x.cpp)
ament_target_dependencies(encoder_node rclcpp sensor_msgs)
target_link_libraries(encoder_node ${catkin_LIBRARIES} -lwiringPi)
target_include_directories(encoder_node PUBLIC include/sensors_and_observers src/cui-devices-amt-10x-driver)

########### Executable for force estimator ######################
add_executable(force_estimator_node src/force_estimator_node.cpp
                                    src/cui-devices-amt-10x-driver/cui_devices_amt10x.cpp)
ament_target_dependencies(force_estimator_node rclcpp sensor_msgs Eigen3)
target_link_libraries(force_estimator_node ${catkin_LIBRARIES} -lwiringPi)
target_include_directories(force_estimator_node PUBLIC include/sensors_and_observers src/cui-devices-amt-10x-driver)

########### Add Targets #################
install(TARGETS
  encoder_node
  force_estimator_node
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

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
