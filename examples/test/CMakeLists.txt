project(example_drone ASM C CXX)

fcs_add_arm_executable(${PROJECT_NAME}_2006_test
        TARGET DJI_Board_TypeC
        SOURCES drone_2006.cc)

fcs_add_arm_executable(${PROJECT_NAME}_actuator_test
        TARGET DJI_Board_TypeC
        SOURCES drone_actuator.cc)

fcs_add_arm_executable(${PROJECT_NAME}_3508_test
        TARGET DJI_Board_TypeC
        SOURCES drone_3508.cc)
