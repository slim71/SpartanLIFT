cmake_minimum_required(VERSION 3.8)
project(pelican
        VERSION 0.1
        DESCRIPTION "Package containing the node related to the flying agents"
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14) # TODO: check if later is good

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic) # -Werror
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(pugixml REQUIRED)
find_package(comms REQUIRED)
find_package(fmt)
find_library(PUGIXML_DIR pugixml)

# Create a library to allow separation of headers and sources
add_library(Pelican_class src/pelican.cpp)
ament_target_dependencies(Pelican_class rclcpp px4_msgs comms)
target_include_directories(Pelican_class PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}")
target_link_libraries(Pelican_class "${PUGIXML_DIR}" fmt::fmt)

set(MAIN 
    src/pelican_main.cpp)

set(SOURCES
    src/pelican_roles.cpp 
    src/pelican_heartbeat.cpp 
    src/pelican_election.cpp
    src/pelican_checkers.cpp
    src/pelican_getters.cpp
    src/pelican_setters.cpp
)

set(TEST_SOURCES
    test/fixtures.cpp
    test/getters_test.cpp
    test/smoke.cpp
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  # Find GTests
  find_package(ament_cmake_gtest REQUIRED)
  # Add all source files related to tests
  ament_add_gtest(${PROJECT_NAME}_test 
                  ${TEST_SOURCES}
                  ${SOURCES})
  # Add needed include directories
  target_include_directories(${PROJECT_NAME}_test 
                            PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
                            $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  # Add dependencies if needed
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp px4_msgs comms)
  # Link to local library
  target_link_libraries(${PROJECT_NAME}_test Pelican_class)

  # Install testing targets
  install(TARGETS ${PROJECT_NAME}_test
          DESTINATION lib/${PROJECT_NAME}
  )
endif()

# One for each node: create executable
add_executable(pelican
              ${MAIN}
              ${SOURCES}
)
target_include_directories(pelican PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}")
ament_target_dependencies(pelican rclcpp px4_msgs comms)
target_link_libraries(pelican Pelican_class)
# TODO: delete executables "pelican_listener"
# Once: install executables
install(
  TARGETS pelican
  DESTINATION lib/${PROJECT_NAME} # lib folder inside ROS2 workspace
)
install(
  DIRECTORY config models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
