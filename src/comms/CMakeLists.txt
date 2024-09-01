cmake_minimum_required(VERSION 3.8)
project(comms
        VERSION 0.3.7
        DESCRIPTION "Package for custom ROS2 messages and services")

#To generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

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

set(MESSAGE_FILES
    msg/Command.msg
    msg/Heartbeat.msg
    msg/Proposal.msg
    msg/RequestVoteRPC.msg
    msg/NetworkVertex.msg
    msg/FormationDesired.msg
    msg/FleetSync.msg
)

set(SERVICE_FILES
    srv/FleetInfo.srv
    srv/CargoPoint.srv
    srv/CargoLinkage.srv
    srv/FormationReached.srv
)

set(ACTION_FILES
    action/TeleopData.action
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MESSAGE_FILES}
  ${SERVICE_FILES}
  ${ACTION_FILES}
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_package()
