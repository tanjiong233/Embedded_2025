# Eigen interface library
add_library(eigen3 INTERFACE)
target_include_directories(eigen3 SYSTEM INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen-3.3.9)

file(GLOB_RECURSE BSP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/bsp/*.cc)
file(GLOB_RECURSE LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/libraries/*.cc)
file(GLOB_RECURSE THIRD_PARTY_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/third_party/*.c)

set(fcs_SOURCES ${BSP_SOURCES} ${LIB_SOURCES} ${THIRD_PARTY_SOURCES})
set(fcs_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/bsp
        ${CMAKE_CURRENT_SOURCE_DIR}/libraries
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

fcs_add_board_specific_library(DJI_Board_TypeC_lib
        TARGET DJI_Board_TypeC
        SOURCES ${fcs_SOURCES}
        INCLUDES ${fcs_INCLUDES}
        DEPENDS eigen3)

