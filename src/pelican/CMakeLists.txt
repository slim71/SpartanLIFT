cmake_minimum_required(VERSION 3.8)
project(pelican
        VERSION 0.1
        DESCRIPTION "SpartanLIFT"
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14) # TODO: check if later is good

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic) # -Werror
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(pugixml REQUIRED)
find_package(comms REQUIRED)

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

# To allow separation of headers and sources
add_library(Pelican_lib src/pelican.cpp)
ament_target_dependencies(Pelican_lib rclcpp px4_msgs comms)
target_include_directories(Pelican_lib PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}")
target_link_libraries(Pelican_lib "${PUGIXML_DIR}")

find_package(fmt)
target_link_libraries(Pelican_lib fmt::fmt)

# One for each node: create executable
add_executable(pelican_listener 
                src/pelican_node.cpp 
                src/pelican_roles.cpp 
                src/pelican_heartbeat.cpp 
                src/pelican_election.cpp
                src/pelican_checkers.cpp
                src/pelican_getters.cpp
                src/pelican_setters.cpp)
target_include_directories(pelican_listener PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}")
ament_target_dependencies(pelican_listener rclcpp px4_msgs comms)
target_link_libraries(pelican_listener Pelican_lib)


# Once: install executables
install(TARGETS
    pelican_listener
    DESTINATION lib/${PROJECT_NAME} # lib folder inside ROS2 workspace
)

ament_package()
