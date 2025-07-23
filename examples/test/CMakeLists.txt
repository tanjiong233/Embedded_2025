project(example_test ASM C CXX)

fcs_add_arm_executable(${PROJECT_NAME}
        TARGET DJI_Board_TypeC
        SOURCES drone_test.cc)