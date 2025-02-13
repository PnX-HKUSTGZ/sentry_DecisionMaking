# cmake_minimum_required(VERSION 3.5)
# project(robot_decision)

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# ## Export compile commands for clangd
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# # Find dependencies
# find_package (ament_cmake_auto REQUIRED) 
# ament_auto_find_build_dependencies()

# # Create the executable
# ament_auto_add_executable(${PROJECT_NAME}_node
#   src/decision_node.cpp
# )


# ament_auto_package (
#   INSTALL_TO_SHARE
#   launch
# )

cmake_minimum_required(VERSION 3.10)
project(robot_decision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 使用 ament_cmake_auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 头文件目录
include_directories(include)

# 添加可执行文件
ament_auto_add_executable(robot_decision
  src/main.cpp
  src/NavigateToPose.cpp
)

# 导出 package
ament_auto_package()
