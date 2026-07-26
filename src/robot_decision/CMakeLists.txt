cmake_minimum_required(VERSION 3.16)
project(robot_decision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS
  src/*.cpp
)

find_package(ament_cmake_auto REQUIRED)
find_package(yaml-cpp REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(robot_decision
  ${SRC_FILES}
)

target_link_libraries(robot_decision
  yaml-cpp
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  behavior_trees
  config
)
