/****************************************************************************
 *                                                                          *
 *  简化版双Motor2006按键控制测试代码                                          *
 *  功能：按一下按键让两个电机转动，再按一下停止                                 *
 *  按键：PA0，低电平为按下                                                   *
 *  显示：0.96寸OLED (128x64, I2C)                                           *
 *                                                                          *
 ****************************************************************************/

#include "bsp_gpio.h"
#include "bsp_can.h"
#include "motor.h"
#include "utils.h"
#include "oled.h"
#include "main.h"
#include "cmsis_os.h"

// 按键配置 - PA0，低电平为按下
#define KEY_GPIO_GROUP GPIOA
#define KEY_GPIO_PIN GPIO_PIN_0

// 电机运行参数
#define MOTOR_OUTPUT 3000       // 直接输出值 (范围: -30000 到 30000)
#define LEFT_MOTOR_CAN_ID 0x201 // 左电机CAN ID
#define RIGHT_MOTOR_CAN_ID 0x202// 右电机CAN ID

// OLED配置 - 参考main.cc使用hi2c2
#define OLED_I2C_ADDR 0x3C

// 外部I2C句柄声明 - 参考main.cc使用hi2c2
extern I2C_HandleTypeDef hi2c2;
extern CAN_HandleTypeDef hcan1;

// 任务属性定义
const osThreadAttr_t motorTestTaskAttribute = {
        .name = "motorTestTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};
osThreadId_t motorTestTaskHandle;

// 全局变量
static bsp::CAN* can1 = nullptr;                        // CAN总线对象
static control::Motor2006* left_motor = nullptr;        // 左电机对象
static control::Motor2006* right_motor = nullptr;       // 右电机对象
static bsp::GPIO* key = nullptr;                        // 按键GPIO对象
static display::OLED* oled = nullptr;                   // OLED显示对象

// 电机控制状态
static bool motors_running = false;                     // 电机运行状态
static bool key_pressed_last = false;                   // 上次按键状态

/**
 * @brief 检测按键按下（简单去抖动）
 */
bool IsKeyPressed() {
    static uint32_t last_press_time = 0;
    bool key_current = !key->Read(); // PA0低电平为按下

    if (key_current && !key_pressed_last) {
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_press_time > 200) { // 200ms去抖动
            last_press_time = current_time;
            key_pressed_last = key_current;
            return true;
        }
    }
    key_pressed_last = key_current;
    return false;
}

/**
 * @brief 系统初始化 - 参考main.cc的RTOS_Init
 */
extern "C" void RTOS_Init() {
    // OLED初始化 - 参考main.cc使用hi2c2
    oled = new display::OLED(&hi2c2, OLED_I2C_ADDR);

    // CAN总线初始化 - 参考main.cc
    can1 = new bsp::CAN(&hcan1, true);

    // 按键GPIO初始化 - PA0
    key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);

    // 两个2006电机初始化 - 参考main.cc的电机初始化方式
    left_motor = new control::Motor2006(can1, LEFT_MOTOR_CAN_ID);
    right_motor = new control::Motor2006(can1, RIGHT_MOTOR_CAN_ID);

    // 等待系统稳定
    osDelay(100);

    // 显示初始化信息
    if (oled) {
        oled->OperateGram(display::PEN_CLEAR);
        oled->Printf(0, 0, "Motor Test Ready");
        oled->Printf(1, 0, "Key: PA0");
        oled->Printf(2, 0, "Output: %d", MOTOR_OUTPUT);
        oled->Printf(3, 0, "L:0x%03X R:0x%03X", LEFT_MOTOR_CAN_ID, RIGHT_MOTOR_CAN_ID);
        oled->Printf(4, 0, "Press to start");
        oled->RefreshGram();
    }
}

/**
 * @brief 电机测试任务
 */
extern "C" void motorTestTask(void* arg) {
    UNUSED(arg);

    // 等待系统稳定
    osDelay(1000);

    uint32_t loop_count = 0;

    while (true) {
        // 检测按键按下
        if (IsKeyPressed()) {
            motors_running = !motors_running; // 切换电机状态

            // 显示状态变化
            if (oled) {
                oled->OperateGram(display::PEN_CLEAR);
                if (motors_running) {
                    oled->Printf(0, 0, "MOTORS RUNNING");
                    oled->Printf(1, 0, "Output: %d", MOTOR_OUTPUT);
                } else {
                    oled->Printf(0, 0, "MOTORS STOPPED");
                    oled->Printf(1, 0, "Output: 0");
                }
                oled->Printf(2, 0, "L:%s R:%s",
                             left_motor->connection_flag_ ? "OK" : "X",
                             right_motor->connection_flag_ ? "OK" : "X");
                oled->Printf(3, 0, "Press to toggle");
                oled->RefreshGram();
            }
        }

        // 电机控制 - 直接设置输出值，不使用PID
        if (motors_running) {
            left_motor->SetOutput(MOTOR_OUTPUT);
            right_motor->SetOutput(MOTOR_OUTPUT);
        } else {
            left_motor->SetOutput(0);
            right_motor->SetOutput(0);
        }

        // 发送CAN控制指令 - 参考main.cc的发送方式
        if (left_motor->connection_flag_ || right_motor->connection_flag_) {
            control::MotorCANBase* motors[2] = {left_motor, right_motor};
            control::MotorCANBase::TransmitOutput(motors, 2);
        }

        // 每500次循环更新一次显示（约1秒）
        if (oled && (loop_count % 500 == 0)) {
            oled->OperateGram(display::PEN_CLEAR);
            oled->Printf(0, 0, "%s", motors_running ? "RUNNING" : "STOPPED");
            oled->Printf(1, 0, "L:%s %.1f rpm",
                         left_motor->connection_flag_ ? "OK" : "X",
                         left_motor->GetOmega() * 60.0f / (2 * 3.14159f));
            oled->Printf(2, 0, "R:%s %.1f rpm",
                         right_motor->connection_flag_ ? "OK" : "X",
                         right_motor->GetOmega() * 60.0f / (2 * 3.14159f));
            oled->Printf(3, 0, "Key: %s", !key->Read() ? "Press" : "Free");
            oled->Printf(4, 0, "Count: %lu", loop_count / 500);
            oled->RefreshGram();
        }

        loop_count++;

        // 任务延时 2ms - 参考main.cc的任务延时
        osDelay(2);
    }
}

/**
 * @brief 创建RTOS任务
 */
extern "C" void RTOS_Threads_Init(void) {
    motorTestTaskHandle = osThreadNew(motorTestTask, nullptr, &motorTestTaskAttribute);
}

/**
 * @brief 默认任务 - 参考main.cc的默认任务
 */
extern "C" void RTOS_Default_Task(const void* args) {
    UNUSED(args);

    while (true) {
        // 简单的系统监控
        osDelay(1000);
    }
}