cmake_minimum_required(VERSION 3.8)
project(eolab_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(px4_odom_tf src/px4_odom_tf.cpp)
ament_target_dependencies(px4_odom_tf rclcpp std_msgs geometry_msgs nav_msgs px4_msgs tf2 tf2_ros)

add_executable(odom_viz src/odom_viz.cpp)
ament_target_dependencies(odom_viz rclcpp nav_msgs geometry_msgs)

add_executable(px4_to_imu_msg src/px4_to_imu_msg.cpp)
ament_target_dependencies(px4_to_imu_msg rclcpp sensor_msgs px4_msgs)

install(TARGETS
    px4_odom_tf
    odom_viz
    px4_to_imu_msg
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
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
