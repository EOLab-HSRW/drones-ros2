cmake_minimum_required(VERSION 3.8)
project(eolab_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake REQUIRED)

include(ExternalProject)
include(FetchContent)

find_program(MICRO_XRCE_AGENT_EXECUTABLE MicroXRCEAgent)

if(NOT MICRO_XRCE_AGENT_EXECUTABLE)
    message(STATUS "MicroXRCEAgent not found, proceeding with installation...")

    ExternalProject_Add(
        Micro-XRCE-DDS-Agent
        GIT_REPOSITORY https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
        GIT_TAG v2.4.3
        PREFIX ${CMAKE_BINARY_DIR}/Micro-XRCE-DDS-Agent
        CMAKE_ARGS 
            -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
            -DCMAKE_CXX_FLAGS="-Wno-dev" # harley: to silent some dev warnings
            -DCMAKE_C_FLAGS="-Wno-dev"
    )

else()
    message(STATUS "MicroXRCEAgent is already installed: ${MICRO_XRCE_AGENT_EXECUTABLE}")
endif()

FetchContent_Declare(
    px4_msgs
    GIT_REPOSITORY https://github.com/PX4/px4_msgs.git
    GIT_TAG main
)

# harley: integrated way to fetch the px4_msgs package
FetchContent_MakeAvailable(px4_msgs)

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
