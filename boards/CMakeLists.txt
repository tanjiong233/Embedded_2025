# 创建通用板子接口库
add_library(board_interface INTERFACE)

# 定义所有板子共用的编译宏
target_compile_definitions(board_interface INTERFACE USE_HAL_DRIVER)

# 设置通用编译选项
target_compile_options(board_interface INTERFACE
        $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp>  # 汇编文件用C预处理器
        -ffunction-sections -fdata-sections               # 函数和数据分段，便于链接器优化
        -fmessage-length=0)                              # 错误信息长度不限制

# 设置通用链接选项
target_link_options(board_interface INTERFACE
        -specs=nano.specs -specs=nosys.specs             # 使用newlib-nano库
        -Wl,--gc-sections                                # 链接时删除未使用段
        -Wl,--wrap=malloc -Wl,--wrap=free               # 包装内存分配函数
        -Wl,--wrap=calloc -Wl,--wrap=realloc)

# 链接标准C库和数学库
target_link_libraries(board_interface INTERFACE c m)

# 添加具体板子的子目录
add_subdirectory(DJI_Board_TypeC)