cmake_minimum_required(VERSION 3.8)
project(eolab_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

execute_process(COMMAND ${Python_EXECUTABLE} -m pip install "git+https://github.com/EOLab-HSRW/drones-fw.git@main")

include(ExternalProject)
include(FetchContent)

find_program(MICRO_XRCE_AGENT_EXECUTABLE MicroXRCEAgent)

if(NOT MICRO_XRCE_AGENT_EXECUTABLE)
    message(STATUS "MicroXRCEAgent not found, proceeding with installation...")

    ExternalProject_Add(
        Micro-XRCE-DDS-Agent
        GIT_REPOSITORY https://github.com/harleylara/Micro-XRCE-DDS-Agent.git
        GIT_TAG v2.4.2
        PREFIX ${CMAKE_BINARY_DIR}/Micro-XRCE-DDS-Agent
        CMAKE_ARGS 
            -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
            -DUAGENT_USE_SYSTEM_FASTDDS=ON
            -DUAGENT_USE_SYSTEM_FASTCDR=ON
            -DCMAKE_BUILD_TYPE=Release
            -DCMAKE_SUPPRESS_DEVELOPER_WARNINGS=1
    )

else()
    message(STATUS "MicroXRCEAgent is already installed: ${MICRO_XRCE_AGENT_EXECUTABLE}")
endif()

# set(PX4_MSGS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../px4_msgs)
#
# ExternalProject_Add(
#     px4_msgs
#     GIT_REPOSITORY https://github.com/PX4/px4_msgs.git
#     GIT_TAG main
#     PREFIX ${CMAKE_BINARY_DIR}/px4_msgs
#     SOURCE_DIR ${PX4_MSGS_DIR}
#     PATCH_COMMAND eolab_drones build --type sitl --drone protoflyer --msgs-output ${PX4_MSGS_DIR}
#     CMAKE_ARGS 
#         -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
#         -DCMAKE_SUPPRESS_DEVELOPER_WARNINGS=1
# )

add_executable(wait_gz_ready ./src/wait_gz_ready.cpp)
ament_target_dependencies(wait_gz_ready rclcpp ros_gz_interfaces)

add_executable(wait_gz_entity src/wait_gz_entity.cpp)
ament_target_dependencies(wait_gz_entity rclcpp)
target_link_libraries(wait_gz_entity
  gz-transport13::core
  gz-msgs10::core
)

install(TARGETS
  wait_gz_ready
  wait_gz_entity
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz
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
