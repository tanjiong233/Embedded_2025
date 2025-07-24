/****************************************************************************
 *                                                                          *
 *  电推杆单独调试测试代码                                                      *
 *  功能：按键控制电推杆状态切换                                                *
 *  按键：PA0，低电平为按下                                                   *
 *  电推杆：PI7 GPIO控制，低有效脉冲                                           *
 *  状态循环：正推→停止→反拉→停止→正推                                         *
 *  显示：0.96寸OLED (128x64, I2C)                                           *
 *                                                                          *
 ****************************************************************************/

#include "bsp_gpio.h"
#include "utils.h"
#include "oled.h"
#include "main.h"
#include "cmsis_os.h"

// 按键配置 - PA0，低电平为按下
#define KEY_GPIO_GROUP GPIOA
#define KEY_GPIO_PIN GPIO_PIN_0

// 电推杆配置 - PI7
#define ACTUATOR_GPIO_GROUP GPIOI
#define ACTUATOR_GPIO_PIN GPIO_PIN_7

// OLED配置
#define OLED_I2C_ADDR 0x3C

// 电推杆控制参数
#define ACTUATOR_PULSE_WIDTH 100        // 脉冲宽度100ms

// 外部I2C句柄声明
extern I2C_HandleTypeDef hi2c2;

// 任务属性定义
const osThreadAttr_t actuatorTestTaskAttribute = {
        .name = "actuatorTestTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};
osThreadId_t actuatorTestTaskHandle;

// 电推杆状态枚举
typedef enum {
    ACTUATOR_STOPPED_RETRACTED = 0,    // 停止状态（收缩位置）
    ACTUATOR_EXTENDING,                // 正推状态
    ACTUATOR_STOPPED_EXTENDED,         // 停止状态（伸出位置）
    ACTUATOR_RETRACTING               // 反拉状态
} actuator_state_t;

// 全局变量
static bsp::GPIO* key = nullptr;                        // 按键GPIO对象
static bsp::GPIO* actuator_gpio = nullptr;              // 电推杆GPIO对象
static display::OLED* oled = nullptr;                   // OLED显示对象

// 电推杆控制状态
static actuator_state_t current_actuator_state = ACTUATOR_STOPPED_RETRACTED;
static bool key_pressed_last = false;                   // 上次按键状态
static bool actuator_pulsing = false;                   // 是否正在发送脉冲
static uint32_t last_actuator_pulse_time = 0;           // 上次脉冲时间
static int actuator_pulse_count = 0;                    // 电推杆脉冲计数器

/**
 * @brief 获取状态名称字符串
 */
const char* GetStateString(actuator_state_t state) {
    switch (state) {
        case ACTUATOR_STOPPED_RETRACTED:
            return "STOP(IN)";
        case ACTUATOR_EXTENDING:
            return "EXTENDING";
        case ACTUATOR_STOPPED_EXTENDED:
            return "STOP(OUT)";
        case ACTUATOR_RETRACTING:
            return "RETRACTING";
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
 * @brief 发送电推杆脉冲信号
 * @note 电推杆逻辑：低有效脉冲，低电平持续时间至少100ms
 *       每给一次信号，状态改变：正推-停止-反拉-停止-正推 循环
 */
void SendActuatorPulse() {
    if (actuator_pulsing) return;

    actuator_pulsing = true;
    last_actuator_pulse_time = HAL_GetTick();
    actuator_gpio->Low();  // 低有效脉冲开始
    actuator_pulse_count++;

    // 更新电推杆状态（4个状态循环：正推-停止-反拉-停止）
    switch (actuator_pulse_count % 4) {
        case 1:
            current_actuator_state = ACTUATOR_EXTENDING;
            break;
        case 2:
            current_actuator_state = ACTUATOR_STOPPED_EXTENDED;
            break;
        case 3:
            current_actuator_state = ACTUATOR_RETRACTING;
            break;
        case 0:
            current_actuator_state = ACTUATOR_STOPPED_RETRACTED;
            break;
    }
}

/**
 * @brief 更新电推杆脉冲状态
 */
void UpdateActuatorPulse() {
    if (!actuator_pulsing) return;

    uint32_t current_time = HAL_GetTick();
    if (current_time - last_actuator_pulse_time >= ACTUATOR_PULSE_WIDTH) {
        actuator_gpio->High();  // 脉冲结束
        actuator_pulsing = false;
    }
}

/**
 * @brief 系统初始化
 */
extern "C" void RTOS_Init() {
    // OLED初始化
    oled = new display::OLED(&hi2c2, OLED_I2C_ADDR);

    // 按键GPIO初始化 - PA0
    key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);

    // 电推杆GPIO初始化 - PI7
    actuator_gpio = new bsp::GPIO(ACTUATOR_GPIO_GROUP, ACTUATOR_GPIO_PIN);
    actuator_gpio->High();  // 初始化为高电平（无效状态）

    // 等待系统稳定
    osDelay(100);

    // 显示初始化信息
    if (oled) {
        oled->OperateGram(display::PEN_CLEAR);
        oled->Printf(0, 0, "Actuator Test");
        oled->Printf(1, 0, "Key: PA0");
        oled->Printf(2, 0, "GPIO: PI7");
        oled->Printf(3, 0, "State: %s", GetStateString(current_actuator_state));
        oled->Printf(4, 0, "Press to change");
        oled->RefreshGram();
    }
}

/**
 * @brief 电推杆测试任务
 */
extern "C" void actuatorTestTask(void* arg) {
    UNUSED(arg);

    // 等待系统稳定
    osDelay(1000);

    uint32_t loop_count = 0;
    uint32_t last_display_update = 0;

    while (true) {
        uint32_t current_time = HAL_GetTick();

        // 更新电推杆脉冲状态
        UpdateActuatorPulse();

        // 检测按键按下
        if (IsKeyPressed()) {
            // 发送脉冲信号切换状态
            SendActuatorPulse();

            // 立即更新显示
            if (oled) {
                oled->OperateGram(display::PEN_CLEAR);
                oled->Printf(0, 0, "State Changed!");
                oled->Printf(1, 0, "New: %s", GetStateString(current_actuator_state));
                oled->Printf(2, 0, "Count: %d", actuator_pulse_count);
                oled->Printf(3, 0, "Pulse: %s", actuator_pulsing ? "ON" : "OFF");
                oled->Printf(4, 0, "Press again...");
                oled->RefreshGram();
            }
            last_display_update = current_time;
        }

        // 每500ms更新一次显示（如果最近没有按键更新）
        if (oled && (current_time - last_display_update > 500)) {
            oled->OperateGram(display::PEN_CLEAR);
            oled->Printf(0, 0, "Actuator Test");
            oled->Printf(1, 0, "State: %s", GetStateString(current_actuator_state));
            oled->Printf(2, 0, "Count: %d", actuator_pulse_count);
            oled->Printf(3, 0, "Pulse: %s", actuator_pulsing ? "ON" : "OFF");

            // 显示按键状态和GPIO状态
            oled->Printf(4, 0, "K:%s G:%s",
                         !key->Read() ? "P" : "R",  // Key Pressed/Released
                         actuator_gpio->Read() ? "H" : "L"); // GPIO High/Low
            oled->RefreshGram();
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
    actuatorTestTaskHandle = osThreadNew(actuatorTestTask, nullptr, &actuatorTestTaskAttribute);
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
