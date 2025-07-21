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

// Global Variables
static remote::SBUS* sbus;
static bsp::CAN* can1 = nullptr;

// Delays
static const int VEHICLE_TASK_DELAY = 2;
static const int TRANSFORM_TASK_DELAY = 10;
static const int SUCTION_TASK_DELAY = 20;
static const int IMU_TASK_DELAY = 1;
static const int SELFTEST_TASK_DELAY = 100;
static const int LED_TASK_DELAY = 1;
static const int DEFAULT_TASK_DELAY = 100;

//==================================================================================================
// Vehicle
//==================================================================================================

// Params Initialization
static control::MotorCANBase* vl_motor = nullptr;
static control::MotorCANBase* vr_motor = nullptr;

const osThreadAttr_t vehicleTaskAttribute = {
        .name = "vehicleTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};
osThreadId_t vehicleTaskHandle;

extern "C" void vehicleTask(void* arg) {
    UNUSED(arg);
    while (true){

        osDelay(VEHICLE_TASK_DELAY);
    }
}

//==================================================================================================
// Transform
//==================================================================================================

// Params Initialization
static control::MotorCANBase* tf_motor = nullptr;

const osThreadAttr_t transformTaskAttribute = {
        .name = "transformTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityAboveNormal,
        .tz_module = 0,
        .reserved = 0
};
osThreadId_t transformTaskHandle;

extern "C" void transformTask(void* arg) {
    UNUSED(arg);
    while(true){

        osDelay(TRANSFORM_TASK_DELAY);
    }
}

//==================================================================================================
// Suction
//==================================================================================================

const osThreadAttr_t suctionTaskAttribute = {
        .name = "suctionTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityLow,
        .tz_module = 0,
        .reserved = 0
};
osThreadId_t suctionTaskHandle;

extern "C" void suctionTask(void* arg) {
    UNUSED(arg);
    while (true) {

        osDelay(SUCTION_TASK_DELAY);
    }
}


//==================================================================================================
// Communication
//==================================================================================================

#define USB_RX_SIGNAL (1 << 0)

const osThreadAttr_t communicationTaskAttribute = {
        .name = "communicationTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityBelowNormal,
        .tz_module = 0,
        .reserved = 0
};
osThreadId_t communicationTaskHandle;

static bsp::VirtualUSB* usb;

class CustomUSBCallback : public bsp::VirtualUSB {
protected:
    void RxCompleteCallback() override final { osThreadFlagsSet(communicationTaskHandle, USB_RX_SIGNAL); }
};

extern "C" void communicationTask(void* arg) {
    UNUSED(arg);
    while(true){
        /* wait until rx data is available */
        uint32_t flags = osThreadFlagsWait(USB_RX_SIGNAL, osFlagsWaitAll, osWaitForever);

    }
}

//==================================================================================================
// IMU
//==================================================================================================

#define IMU_RX_SIGNAL (1 << 1)

const osThreadAttr_t imuTaskAttribute = {.name = "imuTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0};
osThreadId_t imuTaskHandle;

class IMU : public bsp::IMU_typeC {
public:
    using bsp::IMU_typeC::IMU_typeC;

protected:
    void RxCompleteCallback() final { osThreadFlagsSet(imuTaskHandle, IMU_RX_SIGNAL); }
};

static IMU* imu = nullptr;

extern "C" void imuTask(void* arg) {
    UNUSED(arg);
    imu->Calibrate();
    while (true) {
        uint32_t flags = osThreadFlagsWait(IMU_RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & IMU_RX_SIGNAL) {  // unnecessary check
            imu->Update();
        }
        osDelay(IMU_TASK_DELAY);
    }
}

//==================================================================================================
// SelfTest
//==================================================================================================

const osThreadAttr_t selfTestTaskAttribute = {
        .name = "selfTestTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityBelowNormal,
        .tz_module = 0,
        .reserved = 0
};
osThreadId_t selfTestTaskHandle;

// Params Initialization
static bsp::Buzzer* buzzer = nullptr;
static display::OLED* OLED = nullptr;
using Note = bsp::BuzzerNote;
static bsp::BuzzerNoteDelayed Mario[] = {
        {Note::Mi3M, 80}, {Note::Silent, 80}, {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::Do1M, 80}, {Note::Silent, 80}, {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::So5M, 80}, {Note::Silent, 560}, {Note::So5L, 80}, {Note::Silent, 0}, {Note::Finish, 0}};

// Vehicle motor flags
static bool vl_motor_flag = false;
static bool vr_motor_flag = false;
// Transform motor flag
static bool tf_motor_flag = false;
// Sbus flag
static bool sbus_flag = false;
// Computer flag
static bool computer_flag = false;

extern "C" void selfTestTask(void* arg) {
    UNUSED(arg);
    osDelay(SELFTEST_TASK_DELAY);
    // Enter screen
    OLED->ShowIlliniRMLOGO();
    buzzer->SingSong(Mario, [](uint32_t milli) { osDelay(milli);});
    OLED->OperateGram(display::PEN_CLEAR);
    OLED->RefreshGram();
    // Showing Name for self-check
    // Vehicle motors
    OLED->ShowString(0, 0, (uint8_t*)"VL");
    OLED->ShowString(1, 0, (uint8_t*)"VR");
    // Transform motor
    OLED->ShowString(2, 0, (uint8_t*)"Tf");
    // Sbus
    OLED->ShowString(3, 0, (uint8_t*)"Sbs");
    // Computer
    OLED->ShowString(4, 0, (uint8_t*)"Cp");
    OLED->RefreshGram();
    while (true) {
        // Vehicle motors
        vl_motor->connection_flag_ = false;
        vr_motor->connection_flag_ = false;
        // Transform motor
        tf_motor->connection_flag_ = false;
        // Sbus
        sbus->connection_flag_ = false;
        osDelay(SELFTEST_TASK_DELAY);
        // Vehicle motors
        vl_motor_flag = vl_motor->connection_flag_;
        vr_motor_flag = vr_motor->connection_flag_;
        // Transform motor
        tf_motor_flag = tf_motor->connection_flag_;
        // Sbus
        sbus_flag = sbus->connection_flag_;

        //Showing the result for self check
        // Vehicle motors
        OLED->ShowBlock(0, 4, vl_motor_flag);
        OLED->ShowBlock(1, 4, vr_motor_flag);
        // Transform motor
        OLED->ShowBlock(2, 4, tf_motor_flag);
        // Sbus
        OLED->ShowBlock(3, 4, sbus_flag);
        // Computer (if you have computer connection check)
        OLED->ShowBlock(4, 4, computer_flag);
        OLED->RefreshGram();
        osDelay(100);
    }
}
//==================================================================================================
// LED-RGB
//==================================================================================================

static display::RGB* led = nullptr;

const osThreadAttr_t ledTaskAttribute = {
        .name = "ledTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityLow,
        .tz_module = 0,
        .reserved = 0
};
osThreadId_t ledTaskHandle;

extern "C" void ledTask(void* arg) {
    UNUSED(arg);

    // RGB流水灯参数
    int RGB_FLOW_COLOR_CHANGE_TIME = 300;
    uint32_t RGB_flow_color[3] = {0xFFFF0000, 0xFF00FF00, 0xFF0000FF}; // 红、绿、蓝

    float delta_alpha, delta_red, delta_green, delta_blue;
    float alpha, red, green, blue;
    uint32_t aRGB;

    int i = 0;
    while (true) {
        // 获取当前颜色分量
        alpha = (RGB_flow_color[i] & 0xFF000000) >> 24;
        red = ((RGB_flow_color[i] & 0x00FF0000) >> 16);
        green = ((RGB_flow_color[i] & 0x0000FF00) >> 8);
        blue = ((RGB_flow_color[i] & 0x000000FF) >> 0);

        // 计算到下一个颜色的增量
        delta_alpha = (float)((RGB_flow_color[(i + 1) % 3] & 0xFF000000) >> 24) -
                      (float)((RGB_flow_color[i] & 0xFF000000) >> 24);
        delta_red = (float)((RGB_flow_color[(i + 1) % 3] & 0x00FF0000) >> 16) -
                    (float)((RGB_flow_color[i] & 0x00FF0000) >> 16);
        delta_green = (float)((RGB_flow_color[(i + 1) % 3] & 0x0000FF00) >> 8) -
                      (float)((RGB_flow_color[i] & 0x0000FF00) >> 8);
        delta_blue = (float)((RGB_flow_color[(i + 1) % 3] & 0x000000FF) >> 0) -
                     (float)((RGB_flow_color[i] & 0x000000FF) >> 0);

        delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;

        // 渐变过程
        for (int j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; ++j) {
            alpha += delta_alpha;
            red += delta_red;
            green += delta_green;
            blue += delta_blue;

            aRGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 |
                   ((uint32_t)(green)) << 8 | ((uint32_t)(blue)) << 0;

            if (led != nullptr) {
                led->Display(aRGB);
            }
            osDelay(LED_TASK_DELAY);
        }
        ++i;
        i = i % 3;
    }
}

//==================================================================================================
// RTOS Init
//==================================================================================================

extern "C" void RTOS_Init() {
    // print
    print_use_uart(&huart1);
    // Imu initialization
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
    imu = new IMU(imu_init, false);
    // Sbus
    sbus = new remote::SBUS(&huart3);
    // Buzzer
    buzzer = new bsp::Buzzer(&htim4, 3, 1000000);
    // OLED
    OLED = new display::OLED(&hi2c2, 0x3C);
    // LED-RPG
    led = new display::RGB(&htim5, 3, 2, 1, 1000000);
    // CAN
    can1 = new bsp::CAN(&hcan1, true);
    // Vehicle initialization
    vl_motor = new control::Motor2006(can1, 0x201);  // 左轮电机，CAN ID 1
    vr_motor = new control::Motor2006(can1, 0x202);  // 右轮电机，CAN ID 2
    // Transform initialization
    tf_motor = new control::Motor3508(can1, 0x203);  // 变形电机，CAN ID 3
    // USB
    usb = new CustomUSBCallback();
    usb->SetupTx(2048);
    usb->SetupRx(2048);

}

//==================================================================================================
// RTOS Threads Init
//==================================================================================================

extern "C" void RTOS_Threads_Init(void) {
    imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
    vehicleTaskHandle = osThreadNew(vehicleTask, nullptr, &vehicleTaskAttribute);
    transformTaskHandle = osThreadNew(transformTask, nullptr, &transformTaskAttribute);
    suctionTaskHandle = osThreadNew(suctionTask, nullptr, &suctionTaskAttribute);
    communicationTaskHandle = osThreadNew(communicationTask, nullptr, &communicationTaskAttribute);
    selfTestTaskHandle = osThreadNew(selfTestTask, nullptr, &selfTestTaskAttribute);
    ledTaskHandle = osThreadNew(ledTask, nullptr, &ledTaskAttribute);
}


//==================================================================================================
// RTOS Default Task
//==================================================================================================

extern "C" void RTOS_Default_Task(const void* args) {
    UNUSED(args);

    while (true) {

        osDelay(DEFAULT_TASK_DELAY);
    }
}
