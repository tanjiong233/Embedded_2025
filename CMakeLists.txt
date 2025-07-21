cmake_minimum_required(VERSION 3.8)

include(cmake/arm_toolchain.cmake)      # ARM交叉编译工具链
include(cmake/build_helper.cmake)

project(Embedded_2025)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)    # 生成编译命令数据库

add_subdirectory(boards)
add_subdirectory(shared)
add_subdirectory(robot)
add_subdirectory(examples)
