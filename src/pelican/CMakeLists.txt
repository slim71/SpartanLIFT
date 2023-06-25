cmake_minimum_required(VERSION 3.8)
# TODO: add VERSION 1.0
# DESCRIPTION "Very nice project"
# LANGUAGES CXX)
project(pelican) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add pugixml folder to CMAKE_PREFIX_PATH
# Create a list by replacing colon with semicolon
# string(REPLACE ":" ";" CMAKE_PREFIX_PATH "$ENV{CMAKE_PREFIX_PATH}")
# # Append to the newly created list
# list(APPEND CMAKE_PREFIX_PATH "/usr/local/lib/")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(pugixml REQUIRED)

# Find pugixml
find_library(PUGIXML_DIR pugixml)

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

install(
  DIRECTORY config models launch
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

# One for each node: create executable
add_executable(pelican_listener src/pelican.cpp)
ament_target_dependencies(pelican_listener rclcpp px4_msgs)
target_link_libraries(pelican_listener "${PUGIXML_DIR}")

# Once: install executables
install(TARGETS
    pelican_listener
    DESTINATION lib/${PROJECT_NAME} # lib folder inside ROS2 workspace
)

ament_package()
