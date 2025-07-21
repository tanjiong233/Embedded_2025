cmake_minimum_required(VERSION 3.22)

# 获取当前目录名作为项目名
get_filename_component(CURRENT_DIR_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${CURRENT_DIR_NAME} C ASM)

# STM32F407特定配置
set(MCU_LINE STM32F407xx)
set(MCU_ARCH cortex-m4)
set(MCU_FLOAT_ABI hard)
set(MCU_FPU fpv4-sp-d16)
set(MCU_LINKER_SCRIPT STM32F407XX_FLASH.ld)
set(MCU_OPTIONS -mcpu=${MCU_ARCH} -mthumb -mthumb-interwork
        -mfpu=${MCU_FPU} -mfloat-abi=${MCU_FLOAT_ABI})

# 先创建一个临时的应用程序目标，供STM32CubeMX的CMakeLists.txt使用
add_executable(${CMAKE_PROJECT_NAME} "")

# 在包含STM32CubeMX的CMakeLists.txt之前，重新定义路径变量
# 将CMAKE_SOURCE_DIR替换为CMAKE_CURRENT_SOURCE_DIR以匹配实际文件位置
set(ORIGINAL_CMAKE_SOURCE_DIR ${CMAKE_SOURCE_DIR})
set(CMAKE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# 包含STM32CubeMX的CMakeLists.txt，这样就可以使用其中定义的库和变量
# 注意：这会自动将MX_Application_Src添加到${CMAKE_PROJECT_NAME}目标中
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/stm32cubemx/CMakeLists.txt)

# 恢复原始的CMAKE_SOURCE_DIR
set(CMAKE_SOURCE_DIR ${ORIGINAL_CMAKE_SOURCE_DIR})

# 创建板子接口库（Interface Library）
add_library(${PROJECT_NAME}_interface INTERFACE)
target_compile_definitions(${PROJECT_NAME}_interface INTERFACE
        ${MCU_LINE}                                    # MCU型号定义
        ARM_MATH_CM4                                   # ARM数学库支持
        BOARD_HAS_UART_DMA_DOUBLE_BUFFER)             # 板子特性定义

target_compile_options(${PROJECT_NAME}_interface INTERFACE ${MCU_OPTIONS})
target_link_options(${PROJECT_NAME}_interface INTERFACE
        ${MCU_OPTIONS} -T${CMAKE_CURRENT_SOURCE_DIR}/${MCU_LINKER_SCRIPT})

# 继承通用板子接口和STM32CubeMX接口
target_link_libraries(${PROJECT_NAME}_interface INTERFACE
        board_interface
        stm32cubemx)

# 创建板子驱动库
add_library(${PROJECT_NAME} OBJECT
        ${MX_Application_Src}
        ${STM32_Drivers_Src}
        ${USB_Device_Library_Src}
        ${FreeRTOS_Src})


# 将DSP库和板子接口链接到板子库
target_link_libraries(${PROJECT_NAME} PUBLIC
        ${PROJECT_NAME}_interface)
