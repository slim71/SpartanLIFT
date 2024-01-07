cmake_minimum_required(VERSION 3.8)
project(datapad
        VERSION 0.1.0
        DESCRIPTION "Package simulating a tablet through which a human agent would interact with the fleet"
        LANGUAGES CXX)

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
find_package(comms REQUIRED)
find_package(fmt)

# Create a library to allow separation of headers and sources
add_library(Datapad_class src/datapad.cpp)
ament_target_dependencies(Datapad_class rclcpp comms)
target_include_directories(Datapad_class PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}")
target_link_libraries(Datapad_class fmt::fmt)

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
    src/functionalities.cpp
)

set(TEST_SOURCES
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
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp comms)
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
ament_target_dependencies(datapad rclcpp comms)
target_link_libraries(datapad Datapad_class)
# Once: install executables
install(
  TARGETS datapad
  DESTINATION lib/${PROJECT_NAME} # lib folder inside ROS2 workspace
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
