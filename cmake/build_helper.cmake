## fcs_add_arm_executable(<name>
#                         TARGET <board_specific_driver_library>
#                         SOURCES <src1>.c [<src2>.cc <src3>.s ...]
#                         [DEPENDS <dep1> ...]
#                         [INCLUDES <inc1> ...])
#
#   用于生成ARM可执行文件的辅助函数，生成<name>.elf、<name>.bin、<name>.hex文件
#   将创建快捷命令`flash-<name>`（在debug构建中还有`debug-<name>`）
#   用于通过stlink-utils进行烧录和启动gdb调试
#
#   例如：fcs_add_arm_executable(hero TARGET DJI_Board_TypeA ...)创建名为
#   `flash-hero`和`debug-hero`的快捷目标。在命令行中运行这些目标
#   就像运行以下命令一样简单：
#
#   `make flash-hero` 和/或 `make debug-hero`
#
function(fcs_add_arm_executable name)
    # 解析函数参数
    cmake_parse_arguments(ARG "" "TARGET" "SOURCES;INCLUDES;DEPENDS" ${ARGN})

    # 设置输出文件路径
    set(HEX_FILE ${CMAKE_CURRENT_BINARY_DIR}/${name}.hex)    # HEX格式文件
    set(BIN_FILE ${CMAKE_CURRENT_BINARY_DIR}/${name}.bin)    # 二进制文件
    set(MAP_FILE ${CMAKE_CURRENT_BINARY_DIR}/${name}.map)    # 内存映射文件

    # 创建可执行文件目标
    add_executable(${name}.elf ${ARG_SOURCES})

    # 链接库文件
    target_link_libraries(${name}.elf
            PRIVATE ${ARG_DEPENDS} ${ARG_TARGET} ${ARG_TARGET}_lib)

    # 添加头文件目录
    target_include_directories(${name}.elf PRIVATE ${ARG_INCLUDES})

    # 设置链接选项，生成内存映射文件
    target_link_options(${name}.elf PRIVATE -Wl,-Map=${MAP_FILE})

    # 查找ARM工具链程序
    find_program(ARM_SIZE arm-none-eabi-size REQUIRED)        # 查看程序大小的工具
    find_program(ARM_OBJCOPY arm-none-eabi-objcopy REQUIRED)  # 对象文件转换工具

    # 添加构建后的自定义命令
    add_custom_command(TARGET ${name}.elf POST_BUILD
            COMMAND ${ARM_SIZE} ${name}.elf                                      # 显示程序大小信息
            COMMAND ${ARM_OBJCOPY} -Oihex $<TARGET_FILE:${name}.elf> ${HEX_FILE} # 生成HEX文件
            COMMAND ${ARM_OBJCOPY} -Obinary $<TARGET_FILE:${name}.elf> ${BIN_FILE} # 生成二进制文件
            COMMENT "Building ${HEX_FILE}\nBuilding ${BIN_FILE}")

#    # 创建通过ST-Link烧录的目标
#    add_custom_target(flash-${name}
#            COMMAND st-flash --reset write ${BIN_FILE} 0x8000000  # 烧录到Flash地址0x8000000
#            DEPENDS ${name}.elf)
#
#    # 创建通过OpenOCD烧录的目标
#    add_custom_target(rflash-${name}
#            COMMENT "Flashing ${BIN_FILE}"
#            COMMAND openocd -f ${CMAKE_SOURCE_DIR}/debug/OpenOCD/cmsis-dap.cfg -c "program ${BIN_FILE} reset exit 0x08000000"
#            DEPENDS ${name}.elf)

#    # 如果不是Release构建，则创建调试目标
#    if (NOT CMAKE_BUILD_TYPE STREQUAL "Release")
#        find_program(ARM_GDB arm-none-eabi-gdb REQUIRED)  # 查找ARM GDB调试器
#        add_custom_target(debug-${name}
#                COMMAND ${ARM_GDB} $<TARGET_FILE:${name}.elf>  # 启动GDB调试
#                DEPENDS ${name}.elf)
#    endif()
endfunction(fcs_add_arm_executable)

## fcs_add_board_specific_library(<name>
#                                 TARGET <board_specific_driver_library>
#                                 SOURCES <src1>.c [<src2>.cc <src3>.s ...]
#                                 [INCLUDES <inc1> ...]
#                                 [DEPENDS <dep1> ...])
#
#   用于生成板级特定静态库的辅助函数
#   使用示例请参考shared/CMakeLists.txt
function(fcs_add_board_specific_library name)
    # 解析函数参数
    cmake_parse_arguments(ARG "" "TARGET" "SOURCES;INCLUDES;DEPENDS" ${ARGN})

    # 创建对象库
    add_library(${name} OBJECT ${ARG_SOURCES})

    # 链接库文件
    target_link_libraries(${name}
            PUBLIC ${ARG_TARGET}_interface   # 公共接口库
            PRIVATE ${ARG_DEPENDS})          # 私有依赖库

    # 添加头文件目录
    target_include_directories(${name} PUBLIC ${ARG_INCLUDES})
endfunction(fcs_add_board_specific_library)