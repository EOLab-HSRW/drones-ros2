cmake_minimum_required(VERSION 3.8)
project(eolab_bringup_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)


add_executable(wait_gz_ready src/wait_gz_ready.cpp)
ament_target_dependencies(wait_gz_ready rclcpp ros_gz_interfaces)

add_executable(wait_gz_entity src/wait_gz_entity.cpp)
ament_target_dependencies(wait_gz_entity rclcpp)

add_executable(wait_gz_gui_ready src/wait_gz_gui_ready.cpp)
ament_target_dependencies(wait_gz_gui_ready rclcpp ros_gz_interfaces)

target_link_libraries(wait_gz_entity
  gz-transport13::core
  gz-msgs10::core
)

target_link_libraries(wait_gz_gui_ready
  gz-transport13::core
)

install(TARGETS
  wait_gz_ready
  wait_gz_entity
  wait_gz_gui_ready
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

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
