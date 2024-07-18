cmake_minimum_required(VERSION 3.8)
project(pelican
        VERSION 0.10.0
        DESCRIPTION "Package containing the node related to the flying agents"
        LANGUAGES CXX)

#To generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17) # Minimum

message("Compiler: ${CMAKE_CXX_COMPILER_ID}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(PEDANTIC_MODE)
  message("I'll be pedantic!")
  add_compile_options(-Werror)
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
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pugixml REQUIRED)
find_package(comms REQUIRED)
find_package(tf2 REQUIRED)
find_package(fmt)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_library(PUGIXML_DIR pugixml)

# Create a library to allow separation of headers and sources
add_library(Pelican_class src/PelicanModule/pelican.cpp)
target_include_directories(Pelican_class PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}")
target_compile_definitions(Pelican_class PRIVATE "TELEOPDATA__BUILDING_DLL")
ament_target_dependencies(Pelican_class rclcpp rclcpp_action rclcpp_components px4_msgs comms nav_msgs geometry_msgs)
target_link_libraries(Pelican_class "${PUGIXML_DIR}" fmt::fmt Eigen3::Eigen)
rclcpp_components_register_node(Pelican_class PLUGIN "pelican::TeleopDataActionServer" EXECUTABLE teleopdata_action_server)

set(MAIN
    src/main.cpp
)

set(SOURCES
    src/types.cpp
    src/utilities.cpp
)

set(PELICAN_SOURCES
    src/PelicanModule/pelican.cpp
    src/PelicanModule/getters.cpp
    src/PelicanModule/setters.cpp
    src/PelicanModule/roles.cpp
    src/PelicanModule/commands.cpp
    src/PelicanModule/module_comms.cpp
)

set(ELECTION_SOURCES
    src/ElectionModule/election.cpp
    src/ElectionModule/setters.cpp
    src/ElectionModule/getters.cpp
    src/ElectionModule/checkers.cpp
    src/ElectionModule/external_comms.cpp
)

set(HEARTBEAT_SOURCES
    src/HeartbeatModule/heartbeat.cpp
    src/HeartbeatModule/getters.cpp
    src/HeartbeatModule/external_comms.cpp
)

set(TACMAP_SOURCES
    src/TacMapModule/tacmap.cpp
    src/TacMapModule/external_comms.cpp
    src/TacMapModule/receiver.cpp
    src/TacMapModule/transmitter.cpp
    src/TacMapModule/getters.cpp
)

set(UNSC_SOURCES
    src/UNSCModule/unsc.cpp
    src/UNSCModule/getters.cpp
    src/UNSCModule/setters.cpp
    src/UNSCModule/external_comms.cpp
    src/UNSCModule/control.cpp
    src/UNSCModule/algorithms.cpp
)

set(LOGGER_SOURCES
    src/LoggerModule/logger.cpp
)

set(TEST_SOURCES
    test/src/smoke.cpp
    test/src/fixtures.cpp
    test/src/pelican_tests.cpp
    test/src/behavior_tests.cpp
    test/src/heartbeat_tests.cpp
    test/src/election_tests.cpp
    test/src/tacmap_tests.cpp
    test/src/unsc_tests.cpp
    test/src/logger_tests.cpp
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  # Find GTests
  find_package(ament_cmake_gtest REQUIRED)
  # Add all source files related to tests
  ament_add_gtest(${PROJECT_NAME}_test
                  ${TEST_SOURCES}
                  ${SOURCES}
                  ${PELICAN_SOURCES}
                  ${ELECTION_SOURCES}
                  ${HEARTBEAT_SOURCES}
                  ${TACMAP_SOURCES}
                  ${UNSC_SOURCES}
                  ${LOGGER_SOURCES}
                  TIMEOUT 100
  )
  # Add needed include directories
  target_include_directories(${PROJECT_NAME}_test
                            PUBLIC
                            "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>;${CMAKE_CURRENT_SOURCE_DIR}/test/include/"
                            $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  # Add dependencies if needed
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp rclcpp_action rclcpp_components px4_msgs comms geometry_msgs)
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
              ${PELICAN_SOURCES}
              ${ELECTION_SOURCES}
              ${HEARTBEAT_SOURCES}
              ${TACMAP_SOURCES}
              ${UNSC_SOURCES}
              ${LOGGER_SOURCES}
)
target_include_directories(pelican PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}")
ament_target_dependencies(pelican rclcpp px4_msgs comms nav_msgs geometry_msgs)
target_link_libraries(pelican Pelican_class)
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
