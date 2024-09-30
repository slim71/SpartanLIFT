cmake_minimum_required(VERSION 3.8)
project(cargo
        VERSION 0.4.2
        DESCRIPTION "Package containing the cargo node development"
        LANGUAGES CXX)

#To generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 14) # Minimum

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic) # -Werror
endif()

if(DEBUG_MODE)
  message("Compiling with debug symbols!")
  set(CMAKE_BUILD_TYPE Debug)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(comms REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(fmt)

# Create a library to allow separation of headers and sources
add_library(Cargo_class src/cargo.cpp)
target_include_directories(Cargo_class PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}" ${ros_gz_interfaces_INCLUDE_DIRS})
ament_target_dependencies(Cargo_class rclcpp comms nav_msgs geometry_msgs ros_gz_interfaces)
target_link_libraries(Cargo_class fmt::fmt)

set(MAIN
   src/main.cpp
)

set(SOURCES
    src/types.cpp
)

set(LOGGER_SOURCES
    src/logger.cpp
)

set(CARGO_SOURCES
    src/cargo.cpp
    src/setters.cpp
)

if(BUILD_TESTING)
  # Find GTests
  find_package(ament_cmake_gtest REQUIRED)
  # Add all source files related to tests
  ament_add_gtest(${PROJECT_NAME}_test
                  ${SOURCES}
                  ${LOGGER_SOURCES}
                  ${CARGO_SOURCES}
                  TIMEOUT 100
  )
  # Add needed include directories
  target_include_directories(${PROJECT_NAME}_test
                            PUBLIC
                            "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>;${CMAKE_CURRENT_SOURCE_DIR}/test/include/"
                            $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  # Add dependencies if needed
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp comms nav_msgs geometry_msgs ros_gz_interfaces)
  # Link to local library
  target_link_libraries(${PROJECT_NAME}_test Cargo_class)

  # Install testing targets
  install(TARGETS ${PROJECT_NAME}_test
          DESTINATION lib/${PROJECT_NAME}
  )
endif()

# One for each node: create executable
add_executable(cargo
              ${MAIN}
              ${SOURCES}
              ${LOGGER_SOURCES}
              ${CARGO_SOURCES}
)
target_include_directories(cargo PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}" ${ros_gz_interfaces_INCLUDE_DIRS})
ament_target_dependencies(cargo rclcpp comms nav_msgs geometry_msgs ros_gz_interfaces)
target_link_libraries(cargo Cargo_class)

# Once: install executables
install(
  TARGETS cargo
  DESTINATION lib/${PROJECT_NAME} # lib folder inside ROS2 workspace
)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
