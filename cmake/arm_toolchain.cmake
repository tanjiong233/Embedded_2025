# 设置为交叉编译环境
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 告诉CMake不要测试编译可执行文件，只测试静态库
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# 设置C/C++标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# 默认构建类型为Debug
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# 配置ARM GCC工具链
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

# 通用编译选项
set(COMMON_FLAGS "-Wall -Werror -Wextra -fdiagnostics-color=always")

# 不同构建类型的编译选项
set(CMAKE_C_FLAGS_DEBUG "-Og -g -DDEBUG")      # Debug: 优化调试体验
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG")      # Release: 最大优化
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O3 -g -DDEBUG")  # 带调试信息的优化版本