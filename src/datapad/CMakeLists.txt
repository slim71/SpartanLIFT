cmake_minimum_required(VERSION 3.8)
project(datapad
        VERSION 0.3.2
        DESCRIPTION "Package simulating a tablet through which a human agent would interact with the fleet"
        LANGUAGES CXX)

#To generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17) # Minimum

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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(comms REQUIRED)
find_package(fmt)

# Create a library to allow separation of headers and sources
add_library(Datapad_class src/datapad.cpp)
target_include_directories(Datapad_class PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}")
target_compile_definitions(Datapad_class PRIVATE "TELEOPDATA__BUILDING_DLL")
ament_target_dependencies(Datapad_class rclcpp rclcpp_action rclcpp_components comms nav_msgs geometry_msgs)
target_link_libraries(Datapad_class fmt::fmt)
rclcpp_components_register_node(Datapad_class PLUGIN "pelican::TeleopDataActionServer" EXECUTABLE teleopdata_action_server)

set(MAIN
  src/main.cpp
)

set(SOURCES
  src/types.cpp
)

set(LOGGER_SOURCES
    src/logger.cpp
)

set(DATAPAD_SOURCES
  src/datapad.cpp
  src/getters.cpp
  src/setters.cpp
  src/functionalities.cpp
)

set(TEST_SOURCES
  test/src/fixtures.cpp
  test/src/datapad_tests.cpp
)

if(BUILD_TESTING)
  # Find GTests
  find_package(ament_cmake_gtest REQUIRED)
  # Add all source files related to tests
  ament_add_gtest(${PROJECT_NAME}_test
                  ${TEST_SOURCES}
                  ${SOURCES}
                  ${LOGGER_SOURCES}
                  ${DATAPAD_SOURCES}
                  TIMEOUT 100
  )
  # Add needed include directories
  target_include_directories(${PROJECT_NAME}_test
                            PUBLIC
                            "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>;${CMAKE_CURRENT_SOURCE_DIR}/test/include/"
                            $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  # Add dependencies if needed
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp comms nav_msgs geometry_msgs)
  # Link to local library
  target_link_libraries(${PROJECT_NAME}_test Datapad_class)

  # Install testing targets
  install(TARGETS ${PROJECT_NAME}_test
          DESTINATION lib/${PROJECT_NAME}
  )
endif()

# One for each node: create executable
add_executable(datapad
              ${MAIN}
              ${SOURCES}
              ${LOGGER_SOURCES}
              ${DATAPAD_SOURCES}
)
target_include_directories(datapad PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}")
ament_target_dependencies(datapad rclcpp comms nav_msgs geometry_msgs)
target_link_libraries(datapad Datapad_class)
# Once: install executables
install(
  TARGETS datapad
  DESTINATION lib/${PROJECT_NAME} # lib folder inside ROS2 workspace
)

ament_package()
