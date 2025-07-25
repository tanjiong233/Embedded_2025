#include "bsp_imu.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "main.h"
#include "spi.h"
#include "sbus.h"
#include "oled.h"
#include "bsp_buzzer.h"
#include "motor.h"
#include "rgb.h"
#include "bsp_usb.h"
#include "minipc_protocol.h"
#include "bsp_pwm.h"
#include "bsp_gpio.h"

#include <memory>
#include <functional>
#include <array>

// Include controller headers
#include "system_state_manager.h"
#include "vehicle_controller.h"
#include "transform_controller.h"
#include "suction_controller.h"
#include "imu_controller.h"
#include "led_controller.h"
#include "self_test_controller.h"
#include "communication_controller.h"

#include "control_modes.h"
#include "task_config.h"

//==================================================================================================
// Global Instances
//==================================================================================================

static remote::SBUS* g_sbus = nullptr;
static bsp::CAN* g_can1 = nullptr;

// Controller instances
static std::unique_ptr<SystemStateManager> g_state_manager;
static std::unique_ptr<VehicleController> g_vehicle_controller;
static std::unique_ptr<TransformController> g_transform_controller;
static std::unique_ptr<SuctionController> g_suction_controller;
static std::unique_ptr<IMUController> g_imu_controller;
static std::unique_ptr<LEDController> g_led_controller;
static std::unique_ptr<SelfTestController> g_self_test_controller;
static std::unique_ptr<CommunicationController> g_comm_controller;

//==================================================================================================
// Task Functions
//==================================================================================================

extern "C" void vehicleTask(void* arg) {
    UNUSED(arg);
    while (true) {
        if (g_vehicle_controller) {
            g_vehicle_controller->Execute();
        }
    }
}

extern "C" void transformTask(void* arg) {
    UNUSED(arg);
    osDelay(1000); // Wait for system initialization
    while (true) {
        if (g_transform_controller) {
            g_transform_controller->Execute();
        }
    }
}

extern "C" void suctionTask(void* arg) {
    UNUSED(arg);
    while (true) {
        if (g_suction_controller) {
            g_suction_controller->Execute();
        }
    }
}

extern "C" void imuTask(void* arg) {
    UNUSED(arg);
    while (true) {
        if (g_imu_controller) {
            g_imu_controller->Execute();
        }
    }
}

extern "C" void selfTestTask(void* arg) {
    UNUSED(arg);
    while (true) {
        if (g_self_test_controller) {
            g_self_test_controller->Execute();
        }
    }
}

extern "C" void communicationTask(void* arg) {
    UNUSED(arg);
    while (true) {
        if (g_comm_controller) {
            g_comm_controller->Execute();
        }
    }
}

extern "C" void ledTask(void* arg) {
    UNUSED(arg);
    while (true) {
        if (g_led_controller) {
            g_led_controller->Execute();
        }
    }
}

//==================================================================================================
// Task Attributes
//==================================================================================================

const osThreadAttr_t vehicleTaskAttribute = {
        .name = "vehicleTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = TASK_STACK_SIZE,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};

const osThreadAttr_t transformTaskAttribute = {
        .name = "transformTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = TASK_STACK_SIZE,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};

const osThreadAttr_t suctionTaskAttribute = {
        .name = "suctionTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = TASK_STACK_SIZE,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};

const osThreadAttr_t imuTaskAttribute = {
        .name = "imuTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = TASK_STACK_SIZE,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};

const osThreadAttr_t selfTestTaskAttribute = {
        .name = "selfTestTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = TASK_STACK_SIZE,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};

const osThreadAttr_t communicationTaskAttribute = {
        .name = "communicationTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = TASK_STACK_SIZE,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};

const osThreadAttr_t ledTaskAttribute = {
        .name = "ledTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = TASK_STACK_SIZE,
        .priority = (osPriority_t)osPriorityLow,
        .tz_module = 0,
        .reserved = 0
};

// Task handles
osThreadId_t vehicleTaskHandle;
osThreadId_t transformTaskHandle;
osThreadId_t suctionTaskHandle;
osThreadId_t imuTaskHandle;
osThreadId_t selfTestTaskHandle;
osThreadId_t communicationTaskHandle;
osThreadId_t ledTaskHandle;

//==================================================================================================
// RTOS Initialization
//==================================================================================================

extern "C" void RTOS_Init() {
    // Initialize serial print
    print_use_uart(&huart1);

    // Initialize global hardware
    g_sbus = new remote::SBUS(&huart3);
    g_can1 = new bsp::CAN(&hcan1, true);

    // Initialize controllers
    g_state_manager = std::make_unique<SystemStateManager>();
    g_state_manager->Init(g_sbus);

    g_vehicle_controller = std::make_unique<VehicleController>();
    g_vehicle_controller->Init(g_can1, g_state_manager.get(), g_sbus);

    g_transform_controller = std::make_unique<TransformController>();
    g_transform_controller->Init(g_can1, g_state_manager.get());

    g_suction_controller = std::make_unique<SuctionController>();
    g_suction_controller->Init(&htim1, g_state_manager.get(), g_sbus);

    // Initialize IMU
    bsp::IST8310_init_t IST8310_init;
    IST8310_init.hi2c = &hi2c3;
    IST8310_init.int_pin = DRDY_IST8310_Pin;
    IST8310_init.rst_group = GPIOG;
    IST8310_init.rst_pin = GPIO_PIN_6;

    bsp::BMI088_init_t BMI088_init;
    BMI088_init.hspi = &hspi1;
    BMI088_init.CS_ACCEL_Port = CS1_ACCEL_GPIO_Port;
    BMI088_init.CS_ACCEL_Pin = CS1_ACCEL_Pin;
    BMI088_init.CS_GYRO_Port = CS1_GYRO_GPIO_Port;
    BMI088_init.CS_GYRO_Pin = CS1_GYRO_Pin;

    bsp::heater_init_t heater_init;
    heater_init.htim = &htim10;
    heater_init.channel = 1;
    heater_init.clock_freq = 1000000;
    heater_init.temp = 45;

    bsp::IMU_typeC_init_t imu_init;
    imu_init.IST8310 = IST8310_init;
    imu_init.BMI088 = BMI088_init;
    imu_init.heater = heater_init;
    imu_init.hspi = &hspi1;
    imu_init.hdma_spi_rx = &hdma_spi1_rx;
    imu_init.hdma_spi_tx = &hdma_spi1_tx;
    imu_init.Accel_INT_pin_ = INT1_ACCEL_Pin;
    imu_init.Gyro_INT_pin_ = INT1_GYRO_Pin;

    g_imu_controller = std::make_unique<IMUController>();
    g_imu_controller->Init(imu_init);

    g_led_controller = std::make_unique<LEDController>();
    g_led_controller->Init(&htim5);

    g_self_test_controller = std::make_unique<SelfTestController>();
    g_self_test_controller->Init(&hi2c2, &htim4,
                                 g_vehicle_controller.get(),
                                 g_transform_controller.get(),
                                 g_sbus,
                                 g_state_manager.get(),
                                 g_imu_controller.get());

    g_comm_controller = std::make_unique<CommunicationController>();
    g_comm_controller->Init(g_state_manager.get(),
                            g_vehicle_controller.get(),
                            g_imu_controller.get(),
                            g_self_test_controller.get());
}

//==================================================================================================
// RTOS Task Initialization
//==================================================================================================

extern "C" void RTOS_Threads_Init(void) {
    imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
    vehicleTaskHandle = osThreadNew(vehicleTask, nullptr, &vehicleTaskAttribute);
    transformTaskHandle = osThreadNew(transformTask, nullptr, &transformTaskAttribute);
    suctionTaskHandle = osThreadNew(suctionTask, nullptr, &suctionTaskAttribute);
    communicationTaskHandle = osThreadNew(communicationTask, nullptr, &communicationTaskAttribute);
    selfTestTaskHandle = osThreadNew(selfTestTask, nullptr, &selfTestTaskAttribute);
    ledTaskHandle = osThreadNew(ledTask, nullptr, &ledTaskAttribute);

    // Set task handles for callbacks
    if (g_imu_controller) {
        g_imu_controller->SetTaskHandle(imuTaskHandle);
    }

    if (g_comm_controller) {
        g_comm_controller->SetTaskHandle(communicationTaskHandle);
    }
}

//==================================================================================================
// RTOS Default Task
//==================================================================================================

extern "C" void RTOS_Default_Task(const void* args) {
    UNUSED(args);

    while (true) {
        if (g_state_manager) {
            g_state_manager->UpdateControlModes();

            if (g_state_manager->IsInEmergencyStop() && g_vehicle_controller) {
                g_vehicle_controller->EmergencyStop();
            }
        }

        osDelay(DEFAULT_TASK_DELAY_MS);
    }
}