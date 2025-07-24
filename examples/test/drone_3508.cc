/****************************************************************************
 *                                                                          *
 *  3508电机按键控制测试代码                                                    *
 *  功能：按键控制3508电机状态循环切换                                          *
 *  状态循环：正转→停止→反转→停止→正转                                         *
 *  按键：PA0，低电平为按下                                                   *
 *  电机：M3508，CAN ID 3 (0x203)                                            *
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
#define MOTOR_FORWARD_OUTPUT 300   // 正转输出值
#define MOTOR_REVERSE_OUTPUT -300  // 反转输出值
#define MOTOR_CAN_ID 0x203         // 3508电机CAN ID

// OLED配置
#define OLED_I2C_ADDR 0x3C

// 外部句柄声明 - 参考main.cc
extern I2C_HandleTypeDef hi2c2;
extern CAN_HandleTypeDef hcan1;

// 任务属性定义
const osThreadAttr_t motor3508TestTaskAttribute = {
        .name = "motor3508TestTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};
osThreadId_t motor3508TestTaskHandle;

// 电机状态枚举
typedef enum {
    MOTOR_STOPPED_IDLE = 0,        // 停止状态（初始）
    MOTOR_RUNNING_FORWARD,         // 正转状态
    MOTOR_STOPPED_FORWARD,         // 停止状态（从正转停止）
    MOTOR_RUNNING_REVERSE          // 反转状态
} motor_state_t;

// 全局变量 - 注意：使用与main.cc相同的命名风格
static bsp::CAN* can1 = nullptr;                        // CAN总线对象
static control::Motor3508* motor_3508 = nullptr;        // 3508电机对象
static bsp::GPIO* key = nullptr;                        // 按键GPIO对象
static display::OLED* OLED = nullptr;                   // OLED显示对象 - 使用大写，与main.cc保持一致

// 电机控制状态
static motor_state_t current_motor_state = MOTOR_STOPPED_IDLE;
static bool key_pressed_last = false;                   // 上次按键状态
static int state_change_count = 0;                      // 状态切换计数

/**
 * @brief 获取状态名称字符串
 */
const char* GetMotorStateString(motor_state_t state) {
    switch (state) {
        case MOTOR_STOPPED_IDLE:
            return "STOP(IDLE)";
        case MOTOR_RUNNING_FORWARD:
            return "FORWARD";
        case MOTOR_STOPPED_FORWARD:
            return "STOP(FWD)";
        case MOTOR_RUNNING_REVERSE:
            return "REVERSE";
        default:
            return "UNKNOWN";
    }
}

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
 * @brief 切换电机状态
 */
void SwitchMotorState() {
    // 状态循环切换：正转→停止→反转→停止→正转
    switch (current_motor_state) {
        case MOTOR_STOPPED_IDLE:
            current_motor_state = MOTOR_RUNNING_FORWARD;
            break;
        case MOTOR_RUNNING_FORWARD:
            current_motor_state = MOTOR_STOPPED_FORWARD;
            break;
        case MOTOR_STOPPED_FORWARD:
            current_motor_state = MOTOR_RUNNING_REVERSE;
            break;
        case MOTOR_RUNNING_REVERSE:
            current_motor_state = MOTOR_STOPPED_IDLE;
            break;
    }
    state_change_count++;
}

/**
 * @brief 根据当前状态设置电机输出
 */
void SetMotorOutput() {
    if (!motor_3508) return;

    switch (current_motor_state) {
        case MOTOR_STOPPED_IDLE:
        case MOTOR_STOPPED_FORWARD:
            motor_3508->SetOutput(0);
            break;
        case MOTOR_RUNNING_FORWARD:
            motor_3508->SetOutput(MOTOR_FORWARD_OUTPUT);
            break;
        case MOTOR_RUNNING_REVERSE:
            motor_3508->SetOutput(MOTOR_REVERSE_OUTPUT);
            break;
    }
}

/**
 * @brief 系统初始化 - 完全模仿main.cc的RTOS_Init风格
 */
extern "C" void RTOS_Init() {

//    HAL_Delay(500);
    // OLED初始化 - 仅创建对象，完全不显示任何内容
    OLED = new display::OLED(&hi2c2, OLED_I2C_ADDR);

    // CAN总线初始化
    can1 = new bsp::CAN(&hcan1, true);

    // 按键GPIO初始化 - PA0
    key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);

    // 3508电机初始化
    motor_3508 = new control::Motor3508(can1, MOTOR_CAN_ID);

    // 等待系统稳定 - 与main.cc保持一致的短延时
    osDelay(100);

    // 关键：完全不在初始化中显示任何OLED内容
}

/**
 * @brief 3508电机测试任务 - 模仿main.cc的selfTestTask风格
 */
extern "C" void motor3508TestTask(void* arg) {
    UNUSED(arg);

    // 等待延时 - 与main.cc的selfTestTask保持一致
    osDelay(100);

    if (OLED) {
        // 关键步骤：模仿main.cc，首先显示启动画面来初始化OLED
        OLED->ShowIlliniRMLOGO();
        osDelay(2000);  // 显示logo 2秒

        // 清屏并准备显示应用内容
        OLED->OperateGram(display::PEN_CLEAR);
        OLED->RefreshGram();

        // 显示初始化信息
        OLED->Printf(0, 0, "M3508 Test Ready");
        OLED->Printf(1, 0, "Key: PA0");
        OLED->Printf(2, 0, "CAN ID: 0x%03X", MOTOR_CAN_ID);
        OLED->Printf(3, 0, "State: %s", GetMotorStateString(current_motor_state));
        OLED->Printf(4, 0, "Press to start");
        OLED->RefreshGram();
    }

    uint32_t loop_count = 0;
    uint32_t last_display_update = HAL_GetTick();

    while (true) {
        uint32_t current_time = HAL_GetTick();

        // 检测按键按下
        if (IsKeyPressed()) {
            // 状态切换
            SwitchMotorState();

            // 立即更新显示
            if (OLED) {
                OLED->OperateGram(display::PEN_CLEAR);
                OLED->Printf(0, 0, "State Changed!");
                OLED->Printf(1, 0, "New: %s", GetMotorStateString(current_motor_state));
                OLED->Printf(2, 0, "Count: %d", state_change_count);
                OLED->Printf(3, 0, "Output: %d",
                             (current_motor_state == MOTOR_RUNNING_FORWARD) ? MOTOR_FORWARD_OUTPUT :
                             (current_motor_state == MOTOR_RUNNING_REVERSE) ? MOTOR_REVERSE_OUTPUT : 0);
                OLED->Printf(4, 0, "Press again...");
                OLED->RefreshGram();
            }
            last_display_update = current_time;
        }

        // 设置电机输出
        SetMotorOutput();

        // 发送CAN控制指令
        if (motor_3508 && motor_3508->connection_flag_) {
            control::MotorCANBase* motors[1] = {motor_3508};
            control::MotorCANBase::TransmitOutput(motors, 1);
        }

        // 每500ms更新一次显示（如果最近没有按键更新）
        if (OLED && (current_time - last_display_update > 500)) {
            OLED->OperateGram(display::PEN_CLEAR);
            OLED->Printf(0, 0, "M3508 Test");
            OLED->Printf(1, 0, "State: %s", GetMotorStateString(current_motor_state));
            OLED->Printf(2, 0, "Speed: %.1f rpm",
                         motor_3508 ? motor_3508->GetOmega() * 60.0f / (2 * 3.14159f) : 0.0f);
            OLED->Printf(3, 0, "Conn: %s Temp: %d",
                         (motor_3508 && motor_3508->connection_flag_) ? "OK" : "X",
                         motor_3508 ? motor_3508->GetTemp() : 0);
            OLED->Printf(4, 0, "K:%s Out:%d",
                         !key->Read() ? "P" : "R",  // Key Pressed/Released
                         (current_motor_state == MOTOR_RUNNING_FORWARD) ? MOTOR_FORWARD_OUTPUT :
                         (current_motor_state == MOTOR_RUNNING_REVERSE) ? MOTOR_REVERSE_OUTPUT : 0);
            OLED->RefreshGram();
            last_display_update = current_time;
        }

        loop_count++;

        // 任务延时 10ms
        osDelay(10);
    }
}

/**
 * @brief 创建RTOS任务
 */
extern "C" void RTOS_Threads_Init(void) {
    motor3508TestTaskHandle = osThreadNew(motor3508TestTask, nullptr, &motor3508TestTaskAttribute);
}

/**
 * @brief 默认任务
 */
extern "C" void RTOS_Default_Task(const void* args) {
    UNUSED(args);

    while (true) {
        // 简单的系统监控
        osDelay(1000);
    }
}