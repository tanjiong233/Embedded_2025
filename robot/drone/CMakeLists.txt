project(Hero ASM C CXX)

fcs_add_arm_executable(drone_
        TARGET DJI_Board_TypeC
        SOURCES main_.cc)

fcs_add_arm_executable(drone__
        TARGET DJI_Board_TypeC
        SOURCES main__.cc)

# 查找所有控制器源文件
file(GLOB_RECURSE CONTROLLER_SOURCES
        "controllers/*.cc"
)

# 查找公共文件源文件 (如果有.cc/.cpp文件的话)
file(GLOB_RECURSE COMMON_SOURCES
        "common/*.cc"
)

# 设置包含目录
set(INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/controllers
        ${CMAKE_CURRENT_SOURCE_DIR}/common
)

# 主执行文件 - drone
fcs_add_arm_executable(drone
        TARGET DJI_Board_TypeC
        SOURCES
        main.cc
        ${CONTROLLER_SOURCES}
        ${COMMON_SOURCES}
        INCLUDES ${INCLUDE_DIRS}
)
