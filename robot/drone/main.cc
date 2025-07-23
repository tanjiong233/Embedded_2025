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

//==================================================================================================
// 全局变量定义
//==================================================================================================

static remote::SBUS* sbus;        // SBUS遥控器对象
static bsp::CAN* can1 = nullptr;  // CAN总线对象

// 电机控制互斥锁
static osMutexId_t motor_control_mutex;

// 任务延迟时间定义（毫秒）- 根据README要求调整
static const int VEHICLE_TASK_DELAY = 2;        // 车辆控制任务延时
static const int TRANSFORM_TASK_DELAY = 10;     // 变形控制任务延时
static const int SUCTION_TASK_DELAY = 20;       // 吸盘控制任务延时
static const int IMU_TASK_DELAY = 1;            // IMU任务延时
static const int SELFTEST_TASK_DELAY = 100;     // 自检任务延时
static const int LED_TASK_DELAY = 1;            // LED任务延时
static const int COMMUNICATION_TASK_DELAY = 50; // 通信任务延时
static const int DEFAULT_TASK_DELAY = 100;      // 默认任务延时

//==================================================================================================
// 控制模式定义
//==================================================================================================

typedef enum {
    EMERGENCY_STOP_MODE = 0,    // 急停模式（通道6下位置）
    REMOTE_CONTROL_MODE = 1,    // 遥控控制模式（通道6中位置）
    COMPUTER_CONTROL_MODE = 2   // 上位机控制模式（通道6上位置）
} control_mode_t;

typedef enum {
    VEHICLE_MODE = 0,           // 车载模式（通道5下位置）
    FLIGHT_MODE = 1             // 飞行模式（通道5上位置）
} transform_mode_t;

// 系统状态变量
static control_mode_t current_control_mode = EMERGENCY_STOP_MODE;
static control_mode_t previous_control_mode = EMERGENCY_STOP_MODE;
static transform_mode_t current_transform_mode = VEHICLE_MODE;
static transform_mode_t previous_transform_mode = VEHICLE_MODE;

// 控制模式阈值定义
static const int16_t MODE_UPPER_THRESHOLD = 400;   // 上位置模式阈值
static const int16_t MODE_LOWER_THRESHOLD = -400;  // 下位置模式阈值
static const uint32_t SBUS_TIMEOUT_MS = 500;       // SBUS超时时间

//==================================================================================================
// 车辆控制模块
//==================================================================================================

// 车辆控制任务属性定义
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

// 车辆电机对象（左轮：CAN ID 1，右轮：CAN ID 2）
static control::MotorCANBase* left_wheel_motor = nullptr;   // 左轮电机
static control::MotorCANBase* right_wheel_motor = nullptr;  // 右轮电机

// 车辆控制参数
static const float MAX_VEHICLE_SPEED = 5.0f;  // 最大车辆速度 (rad/s)
static const float TURN_SENSITIVITY = 0.5f;   // 转向灵敏度系数
static const int SBUS_MAX_VALUE = 660;        // SBUS最大通道值
static const int SBUS_DEADZONE = 50;          // SBUS死区范围

// 电机PID控制参数
static float left_motor_pid_params[3] = {8.0f, 0.1f, 0.0f};   // 左轮电机PID参数
static float right_motor_pid_params[3] = {8.0f, 0.1f, 0.0f};  // 右轮电机PID参数

// PID控制器对象
static control::ConstrainedPID* left_motor_pid = nullptr;   // 左轮电机PID控制器
static control::ConstrainedPID* right_motor_pid = nullptr;  // 右轮电机PID控制器

// 车辆运动状态变量
static float target_linear_speed = 0.0f;    // 目标线速度
static float target_angular_speed = 0.0f;   // 目标角速度
static float left_wheel_speed = 0.0f;       // 左轮目标转速
static float right_wheel_speed = 0.0f;      // 右轮目标转速

/**
 * @brief 对SBUS输入应用死区处理
 */
int16_t ApplyDeadzone(int16_t input, int16_t deadzone) {
    return (abs(input) < deadzone) ? 0 : input;
}

/**
 * @brief 将SBUS输入标准化到 [-1, 1] 范围
 */
float NormalizeSBUS(int16_t input) {
    return (float)input / SBUS_MAX_VALUE;
}

/**
 * @brief 计算差分驱动的左右轮速度
 */
void CalculateDifferentialSpeeds(float linear_speed, float angular_speed,
                                 float* left_speed, float* right_speed) {
    // 差分驱动运动学：简化版本
    *left_speed = linear_speed - angular_speed;
    *right_speed = linear_speed + angular_speed;

    // 限制速度到最大值范围内
    *left_speed = clip<float>(*left_speed, -MAX_VEHICLE_SPEED, MAX_VEHICLE_SPEED);
    *right_speed = clip<float>(*right_speed, -MAX_VEHICLE_SPEED, MAX_VEHICLE_SPEED);
}

/**
 * @brief 车辆控制任务
 */
extern "C" void vehicleTask(void* arg) {
    UNUSED(arg);

    while (true) {
        // 只在车载模式且遥控模式下执行车辆控制
        if (current_transform_mode != VEHICLE_MODE ||
            current_control_mode != REMOTE_CONTROL_MODE) {
            // 非遥控模式时，不发送任何电机命令，避免与其他控制模式冲突
            osDelay(VEHICLE_TASK_DELAY);
            continue;
        }

        // 检查SBUS连接状态
        if (!sbus || !sbus->connection_flag_) {
            // SBUS未连接，停止电机
            if (left_wheel_motor) left_wheel_motor->SetOutput(0);
            if (right_wheel_motor) right_wheel_motor->SetOutput(0);

            if (left_wheel_motor && right_wheel_motor &&
                (left_wheel_motor->connection_flag_ || right_wheel_motor->connection_flag_)) {
                control::MotorCANBase* motors[2] = {left_wheel_motor, right_wheel_motor};
                control::MotorCANBase::TransmitOutput(motors, 2);
            }
            osDelay(VEHICLE_TASK_DELAY);
            continue;
        }

        // 读取SBUS通道并应用死区处理
        // 根据README：通道3为油门，通道4为转向
        int16_t throttle_raw = ApplyDeadzone(sbus->ch[2], SBUS_DEADZONE);    // 通道3（ch[2]）
        int16_t steering_raw = ApplyDeadzone(sbus->ch[3], SBUS_DEADZONE);    // 通道4（ch[3]）

        // 将输入标准化到 [-1, 1] 范围
        float throttle = NormalizeSBUS(throttle_raw);
        float steering = NormalizeSBUS(steering_raw);

        // 计算目标速度
        target_linear_speed = throttle * MAX_VEHICLE_SPEED;
        target_angular_speed = steering * MAX_VEHICLE_SPEED * TURN_SENSITIVITY;

        // 计算差分驱动的左右轮速度
        CalculateDifferentialSpeeds(target_linear_speed, target_angular_speed,
                                    &left_wheel_speed, &right_wheel_speed);

        // 计算PID输出
        if (left_wheel_motor && right_wheel_motor && left_motor_pid && right_motor_pid) {
            float left_error = left_wheel_speed - left_wheel_motor->GetOmega();
            float right_error = right_wheel_speed - right_wheel_motor->GetOmega();

            int16_t left_output = left_motor_pid->ComputeConstrainedOutput(left_error);
            int16_t right_output = right_motor_pid->ComputeConstrainedOutput(right_error);

            // 使用互斥锁保护电机控制
            if (osMutexAcquire(motor_control_mutex, 10) == osOK) {  // 10ms超时
                // 设置电机输出
                left_wheel_motor->SetOutput(left_output);
                right_wheel_motor->SetOutput(right_output);

                // 发送CAN消息
                if (left_wheel_motor->connection_flag_ || right_wheel_motor->connection_flag_) {
                    control::MotorCANBase* motors[2] = {left_wheel_motor, right_wheel_motor};
                    control::MotorCANBase::TransmitOutput(motors, 2);
                }

                osMutexRelease(motor_control_mutex);
            }
        }

        osDelay(VEHICLE_TASK_DELAY);
    }
}

//==================================================================================================
// 变形控制模块
//==================================================================================================

// 变形电机对象（3508：CAN ID 3）
static control::MotorCANBase* transform_motor = nullptr;  // 变形电机

// 电推杆GPIO控制
static bsp::GPIO* actuator_gpio = nullptr;  // 电推杆脉冲信号GPIO (PI7)

// 变形控制任务属性定义
const osThreadAttr_t transformTaskAttribute = {
        .name = "transformTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};
osThreadId_t transformTaskHandle;

// 电推杆控制参数
static const uint32_t ACTUATOR_PULSE_WIDTH = 100;  // 脉冲宽度100ms
static uint32_t last_actuator_pulse_time = 0;      // 上次脉冲时间
static bool actuator_pulsing = false;              // 是否正在发送脉冲

/**
 * @brief 发送电推杆脉冲信号
 * 电推杆逻辑：低有效脉冲，低电平持续时间至少100ms
 * 每给一次信号，状态改变：正推-停止-反拉-停止-正推 循环
 */
void SendActuatorPulse() {
    if (!actuator_gpio || actuator_pulsing) return;

    actuator_pulsing = true;
    last_actuator_pulse_time = HAL_GetTick();
    actuator_gpio->Low();  // 低有效脉冲开始
}

/**
 * @brief 更新电推杆脉冲状态
 */
void UpdateActuatorPulse() {
    if (!actuator_pulsing || !actuator_gpio) return;

    uint32_t current_time = HAL_GetTick();
    if (current_time - last_actuator_pulse_time >= ACTUATOR_PULSE_WIDTH) {
        actuator_gpio->High();  // 脉冲结束
        actuator_pulsing = false;
    }
}

/**
 * @brief 变形控制任务
 */
extern "C" void transformTask(void* arg) {
    UNUSED(arg);

    while (true) {
        // 更新电推杆脉冲状态
        UpdateActuatorPulse();
        osDelay(TRANSFORM_TASK_DELAY);
    }
}

//==================================================================================================
// 吸盘控制模块
//==================================================================================================

// 吸盘风扇对象
static bsp::PWM* suction_fan = nullptr;  // 吸盘风扇PWM控制器

// 吸盘控制参数
static const float MIN_FAN_PULSE_WIDTH = 1000.0f;   // 最小脉宽（微秒）- 风扇关闭
static const float MAX_FAN_PULSE_WIDTH = 2000.0f;   // 最大脉宽（微秒）- 最大速度
static const int SBUS_SUCTION_DEADZONE = 30;        // 吸盘通道死区
static const uint32_t FAN_PWM_FREQUENCY = 1500;     // 风扇PWM频率 (Hz)
static const uint32_t FAN_TIMER_CLOCK = 1000000;    // 定时器时钟频率 (1MHz)

// 吸盘控制任务属性定义
const osThreadAttr_t suctionTaskAttribute = {
        .name = "suctionTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};
osThreadId_t suctionTaskHandle;

/**
 * @brief 将SBUS输入映射到PWM脉宽
 */
float MapSuctionToPulseWidth(int16_t sbus_input) {
    // 只使用正值控制风扇速度（负值 = 风扇关闭）
    if (sbus_input <= 0) {
        return MIN_FAN_PULSE_WIDTH;
    }

    // 标准化到 0-1 范围
    float normalized_input = (float)sbus_input / SBUS_MAX_VALUE;
    if (normalized_input > 1.0f) normalized_input = 1.0f;

    // 映射到脉宽范围
    return MIN_FAN_PULSE_WIDTH + (normalized_input * (MAX_FAN_PULSE_WIDTH - MIN_FAN_PULSE_WIDTH));
}

/**
 * @brief 吸盘控制任务
 */
extern "C" void suctionTask(void* arg) {
    UNUSED(arg);

    if (!suction_fan) {
        osDelay(SUCTION_TASK_DELAY);
        return;
    }

    // 启动PWM输出
    suction_fan->Start();
    suction_fan->SetPulseWidth(MIN_FAN_PULSE_WIDTH);  // 初始化时风扇关闭

    while (true) {
        // 只在遥控模式下执行吸盘控制
        if (current_control_mode != REMOTE_CONTROL_MODE) {
            suction_fan->SetPulseWidth(MIN_FAN_PULSE_WIDTH);
            osDelay(SUCTION_TASK_DELAY);
            continue;
        }

        // 检查SBUS连接状态
        if (!sbus || !sbus->connection_flag_) {
            suction_fan->SetPulseWidth(MIN_FAN_PULSE_WIDTH);
            osDelay(SUCTION_TASK_DELAY);
            continue;
        }

        // 读取SBUS通道2 (ch[1])用于吸盘控制，并应用死区
        int16_t suction_raw = ApplyDeadzone(sbus->ch[1], SBUS_SUCTION_DEADZONE);

        // 将SBUS输入映射到PWM脉宽
        float target_pulse_width = MapSuctionToPulseWidth(suction_raw);

        // 设置风扇速度
        suction_fan->SetPulseWidth(target_pulse_width);

        osDelay(SUCTION_TASK_DELAY);
    }
}

//==================================================================================================
// IMU模块
//==================================================================================================

#define IMU_RX_SIGNAL (1 << 1)

// IMU任务属性定义
const osThreadAttr_t imuTaskAttribute = {
        .name = "imuTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};
osThreadId_t imuTaskHandle;

/**
 * @brief 自定义IMU类
 */
class IMU : public bsp::IMU_typeC {
public:
    using bsp::IMU_typeC::IMU_typeC;

protected:
    void RxCompleteCallback() final {
        osThreadFlagsSet(imuTaskHandle, IMU_RX_SIGNAL);
    }
};

static IMU* imu = nullptr;

/**
 * @brief IMU任务
 */
extern "C" void imuTask(void* arg) {
    UNUSED(arg);

    if (imu) {
        imu->Calibrate();  // IMU校准
    }

    while (true) {
        uint32_t flags = osThreadFlagsWait(IMU_RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & IMU_RX_SIGNAL && imu) {
            imu->Update();  // 更新IMU数据
        }
        osDelay(IMU_TASK_DELAY);
    }
}

//==================================================================================================
// 自检模块
//==================================================================================================

// 自检任务属性定义
const osThreadAttr_t selfTestTaskAttribute = {
        .name = "selfTestTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};
osThreadId_t selfTestTaskHandle;

// 自检设备对象
static bsp::Buzzer* buzzer = nullptr;
static display::OLED* OLED = nullptr;

// 超级马里奥音乐数组
using Note = bsp::BuzzerNote;
static bsp::BuzzerNoteDelayed Mario[] = {
        {Note::Mi3M, 80}, {Note::Silent, 80}, {Note::Mi3M, 80}, {Note::Silent, 240},
        {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::Do1M, 80}, {Note::Silent, 80},
        {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::So5M, 80}, {Note::Silent, 560},
        {Note::So5L, 80}, {Note::Silent, 0}, {Note::Finish, 0}
};

// 设备连接状态标志
static bool left_motor_flag = false;      // 左轮电机连接状态
static bool right_motor_flag = false;     // 右轮电机连接状态
static bool transform_motor_flag = false; // 变形电机连接状态
static bool sbus_flag = false;            // SBUS连接状态
static bool computer_flag = false;        // 上位机连接状态

/**
 * @brief 自检任务
 */
extern "C" void selfTestTask(void* arg) {
    UNUSED(arg);
    osDelay(SELFTEST_TASK_DELAY);

    if (OLED && buzzer) {
        // 启动画面
        OLED->ShowIlliniRMLOGO();
        buzzer->SingSong(Mario, [](uint32_t milli) { osDelay(milli); });
        OLED->OperateGram(display::PEN_CLEAR);
        OLED->RefreshGram();

        // 显示固定标签 - 重新调整位置避免覆盖
        OLED->ShowString(0, 0, (uint8_t*)"LW");    // 左轮电机 (列0-1)
        OLED->ShowString(0, 4, (uint8_t*)"RW");    // 右轮电机 (列4-5)
        OLED->ShowString(0, 8, (uint8_t*)"TF");    // 变形电机 (列8-9)
        OLED->ShowString(0, 12, (uint8_t*)"RC");   // 遥控器 (列12-13)
        OLED->ShowString(0, 16, (uint8_t*)"PC");   // 上位机 (列16-17)
    }

    uint32_t start_time = osKernelGetTickCount();

    while (true) {
        // 检测设备连接状态
        if (left_wheel_motor) left_wheel_motor->connection_flag_ = false;
        if (right_wheel_motor) right_wheel_motor->connection_flag_ = false;
        if (transform_motor) transform_motor->connection_flag_ = false;
        if (sbus) sbus->connection_flag_ = false;

        osDelay(SELFTEST_TASK_DELAY);

        // 读取连接状态
        left_motor_flag = left_wheel_motor ? left_wheel_motor->connection_flag_ : false;
        right_motor_flag = right_wheel_motor ? right_wheel_motor->connection_flag_ : false;
        transform_motor_flag = transform_motor ? transform_motor->connection_flag_ : false;
        sbus_flag = sbus ? sbus->connection_flag_ : false;

        if (OLED) {
            // 行0: 设备状态指示块 - 调整位置避免覆盖文字
            OLED->ShowBlock(0, 2, left_motor_flag);      // 列2-3
            OLED->ShowBlock(0, 6, right_motor_flag);     // 列6-7
            OLED->ShowBlock(0, 10, transform_motor_flag); // 列10-11
            OLED->ShowBlock(0, 14, sbus_flag);           // 列14-15
            OLED->ShowBlock(0, 18, computer_flag);       // 列18-19

            // 行1: 控制模式 + 变形模式 + 运行时间
            const char* mode_str = current_control_mode == EMERGENCY_STOP_MODE ? "STOP" :
                                   current_control_mode == REMOTE_CONTROL_MODE ? "RC" : "PC";
            const char* transform_str = current_transform_mode == VEHICLE_MODE ? "CAR" : "FLY";

            uint32_t runtime_sec = (osKernelGetTickCount() - start_time) / 1000;
            uint32_t hours = runtime_sec / 3600;
            uint32_t minutes = (runtime_sec % 3600) / 60;
            uint32_t seconds = runtime_sec % 60;

            OLED->Printf(1, 0, "M:%s T:%s %02d:%02d:%02d",
                         mode_str, transform_str, hours, minutes, seconds);

            // 行2: 变形电机角度 - 修复浮点数显示
            if (transform_motor && transform_motor->connection_flag_) {
                // 转换为整数显示（保留一位小数）
                int angle_int = (int)(transform_motor->GetTheta() * 573);  // 573 = 180*10/π
                OLED->Printf(2, 0, "TF: %4d.%d deg", angle_int/10, abs(angle_int%10));
            } else {
                OLED->Printf(2, 0, "TF: --- deg");
            }

            // 行3: IMU欧拉角 - 修复浮点数显示
            if (imu) {
                // 转换为整数显示（保留一位小数）
                int pitch_int = (int)(imu->INS_angle[1] * 573);  // 573 = 180*10/π
                int roll_int = (int)(imu->INS_angle[2] * 573);
                int yaw_int = (int)(imu->INS_angle[0] * 573);

                OLED->Printf(3, 0, "P:%d.%d R:%d.%d Y:%d.%d",
                             pitch_int/10, abs(pitch_int%10),
                             roll_int/10, abs(roll_int%10),
                             yaw_int/10, abs(yaw_int%10));
            } else {
                OLED->Printf(3, 0, "P:--- R:--- Y:---");
            }

            // 行4: 电推杆状态
            OLED->Printf(2, 12, "ACT:%s", actuator_pulsing ? "PULSE" : "IDLE");

            OLED->RefreshGram();
        }

        osDelay(100);
    }
}

//==================================================================================================
// 通信模块
//==================================================================================================

#define USB_RX_SIGNAL (1 << 0)

// 通信任务属性定义
const osThreadAttr_t communicationTaskAttribute = {
        .name = "communicationTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};
osThreadId_t communicationTaskHandle;

/**
 * @brief 自定义通信USB类
 */
class CustomCommUSB : public bsp::VirtualUSB {
protected:
    void RxCompleteCallback() override final {
        osThreadFlagsSet(communicationTaskHandle, USB_RX_SIGNAL);
    }
};

static CustomCommUSB* comm_usb = nullptr;
static communication::MinipcPort* minipc_session = nullptr;

// 上位机连接状态检测
static uint32_t last_pc_data_time = 0;                  // 上次收到上位机数据的时间
static const uint32_t PC_TIMEOUT_MS = 2000;             // 上位机超时时间2秒

/**
 * @brief 获取系统时间戳
 */
uint32_t GetSystemTimestamp() {
    return osKernelGetTickCount();
}

/**
 * @brief 发送IMU数据到上位机
 */
void SendIMUData() {
    if (!imu || !minipc_session || !comm_usb) return;

    communication::imu_data_t imu_data;

    const float* accel_data = imu->GetAccel();
    const float* gyro_data = imu->GetGyro();

    imu_data.accel_x = accel_data[0];
    imu_data.accel_y = accel_data[1];
    imu_data.accel_z = accel_data[2];
    imu_data.gyro_x = gyro_data[0];
    imu_data.gyro_y = gyro_data[1];
    imu_data.gyro_z = gyro_data[2];
    imu_data.pitch = imu->INS_angle[1];
    imu_data.roll = imu->INS_angle[2];
    imu_data.yaw = imu->INS_angle[0];
    imu_data.temperature = imu->Temp;
    imu_data.timestamp = GetSystemTimestamp();

    uint8_t packet[minipc_session->MAX_PACKET_LENGTH];
    minipc_session->Pack(packet, &imu_data, communication::IMU_CMD_ID);
    comm_usb->Write(packet, minipc_session->GetPacketLen(communication::IMU_CMD_ID));
}

/**
 * @brief 发送里程计数据到上位机
 */
void SendOdometryData() {
    if (!left_wheel_motor || !right_wheel_motor || !minipc_session || !comm_usb) return;

    communication::odometry_data_t odom_data;

    static const float WHEEL_RADIUS = 0.076f;    // 轮子半径（76mm）
    static const float WHEEL_BASE = 0.335f;      // 轮距（335mm）

    float left_wheel_speed = left_wheel_motor->GetOmega();
    float right_wheel_speed = right_wheel_motor->GetOmega();

    float linear_vel = (left_wheel_speed + right_wheel_speed) * WHEEL_RADIUS / 2.0f;
    float angular_vel = (right_wheel_speed - left_wheel_speed) * WHEEL_RADIUS / WHEEL_BASE;

    static int32_t left_encoder_total = 0;
    static int32_t right_encoder_total = 0;
    left_encoder_total += (int32_t)(left_wheel_speed * 0.05f * 8192 / (2 * 3.14159f));   // 50ms周期
    right_encoder_total += (int32_t)(right_wheel_speed * 0.05f * 8192 / (2 * 3.14159f));

    odom_data.left_encoder = left_encoder_total;
    odom_data.right_encoder = right_encoder_total;
    odom_data.left_wheel_speed = left_wheel_speed;
    odom_data.right_wheel_speed = right_wheel_speed;
    odom_data.linear_velocity = linear_vel;
    odom_data.angular_velocity = angular_vel;
    odom_data.wheel_base = WHEEL_BASE;
    odom_data.wheel_radius = WHEEL_RADIUS;
    odom_data.timestamp = GetSystemTimestamp();

    uint8_t packet[minipc_session->MAX_PACKET_LENGTH];
    minipc_session->Pack(packet, &odom_data, communication::ODOMETRY_CMD_ID);
    comm_usb->Write(packet, minipc_session->GetPacketLen(communication::ODOMETRY_CMD_ID));
}

/**
 * @brief 发送系统状态到上位机
 */
void SendSystemStatus() {
    if (!minipc_session || !comm_usb) return;

    communication::system_status_t status;

    status.robot_mode = (uint8_t)current_control_mode;
    status.transform_mode = (uint8_t)current_transform_mode;
    status.sbus_connected = sbus ? sbus->connection_flag_ : 0;
    status.imu_connected = imu ? 1 : 0;
    status.vl_motor_online = left_wheel_motor ? left_wheel_motor->connection_flag_ : 0;
    status.vr_motor_online = right_wheel_motor ? right_wheel_motor->connection_flag_ : 0;
    status.tf_motor_online = transform_motor ? transform_motor->connection_flag_ : 0;
    status.reserved = 0;
    status.timestamp = GetSystemTimestamp();

    uint8_t packet[minipc_session->MAX_PACKET_LENGTH];
    minipc_session->Pack(packet, &status, communication::SYSTEM_STATUS_CMD_ID);
    comm_usb->Write(packet, minipc_session->GetPacketLen(communication::SYSTEM_STATUS_CMD_ID));
}

/**
 * @brief 处理接收到的运动控制指令
 */
void HandleMotionCommand(const communication::status_data_t* status) {
    if (!status || current_control_mode != COMPUTER_CONTROL_MODE) return;

    float target_linear = status->target_linear_vel;
    float target_angular = status->target_angular_vel;

    // 紧急停止处理
    if (status->emergency_stop) {
        if (osMutexAcquire(motor_control_mutex, 10) == osOK) {  // 10ms超时
            if (left_wheel_motor) left_wheel_motor->SetOutput(0);
            if (right_wheel_motor) right_wheel_motor->SetOutput(0);

            // 立即发送停止命令
            if (left_wheel_motor && right_wheel_motor &&
                (left_wheel_motor->connection_flag_ || right_wheel_motor->connection_flag_)) {
                control::MotorCANBase* motors[2] = {left_wheel_motor, right_wheel_motor};
                control::MotorCANBase::TransmitOutput(motors, 2);
            }

            osMutexRelease(motor_control_mutex);
        }
        return;
    }

    static const float WHEEL_RADIUS = 0.076f;
    static const float WHEEL_BASE = 0.335f;
    static const float MAX_WHEEL_SPEED = 10.0f;

    float left_wheel_vel = (target_linear - target_angular * WHEEL_BASE / 2.0f) / WHEEL_RADIUS;
    float right_wheel_vel = (target_linear + target_angular * WHEEL_BASE / 2.0f) / WHEEL_RADIUS;

    left_wheel_vel = clip<float>(left_wheel_vel, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
    right_wheel_vel = clip<float>(right_wheel_vel, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);

    if (left_wheel_motor && right_wheel_motor && left_motor_pid && right_motor_pid) {
        float left_error = left_wheel_vel - left_wheel_motor->GetOmega();
        float right_error = right_wheel_vel - right_wheel_motor->GetOmega();

        int16_t left_output = left_motor_pid->ComputeConstrainedOutput(left_error);
        int16_t right_output = right_motor_pid->ComputeConstrainedOutput(right_error);

        // 使用互斥锁保护电机控制
        if (osMutexAcquire(motor_control_mutex, 10) == osOK) {  // 10ms超时
            left_wheel_motor->SetOutput(left_output);
            right_wheel_motor->SetOutput(right_output);

            // 发送CAN消息（这是之前缺失的关键代码）
            if (left_wheel_motor->connection_flag_ || right_wheel_motor->connection_flag_) {
                control::MotorCANBase* motors[2] = {left_wheel_motor, right_wheel_motor};
                control::MotorCANBase::TransmitOutput(motors, 2);
            }

            osMutexRelease(motor_control_mutex);
        }
    }
}

/**
 * @brief 通信任务（50ms延迟）
 *
 * 功能：
 * 1. 接收上位机指令并处理
 * 2. 定时发送IMU、里程计和系统状态数据
 * 3. 基于接收数据超时检测上位机连接状态
 */
extern "C" void communicationTask(void* arg) {
    UNUSED(arg);

    minipc_session = new communication::MinipcPort();

    uint8_t *data;
    uint32_t length;

    uint32_t last_imu_send = 0;
    uint32_t last_odom_send = 0;
    uint32_t last_status_send = 0;

    const uint32_t IMU_SEND_PERIOD = 10;      // 10ms = 100Hz
    const uint32_t ODOM_SEND_PERIOD = 20;     // 20ms = 50Hz
    const uint32_t STATUS_SEND_PERIOD = 200;  // 200ms = 5Hz

    while (true) {
        uint32_t current_time = osKernelGetTickCount();

        // 检查接收数据
        uint32_t flags = osThreadFlagsWait(USB_RX_SIGNAL, osFlagsWaitAny, 0);
        if (flags & USB_RX_SIGNAL && comm_usb) {
            length = comm_usb->Read(&data);
            if (length > 0 && minipc_session) {
                // 收到数据，更新上位机数据接收时间戳
                last_pc_data_time = current_time;

                minipc_session->ParseUartBuffer(data, length);

                if (minipc_session->GetValidFlag()) {
                    uint8_t cmd_id = minipc_session->GetCmdId();
                    const communication::status_data_t* status = minipc_session->GetStatus();

                    switch (cmd_id) {
                        case communication::MOTION_CMD_ID:
                            HandleMotionCommand(status);
                            break;
                        case communication::SELFCHECK_CMD_ID:
                            // 处理自检指令
                            break;
                        default:
                            break;
                    }
                }
            }
        }

        // 基于接收数据超时判断上位机连接状态
        // 只有在PC_TIMEOUT_MS(2秒)内收到过数据才认为上位机在线
        computer_flag = (current_time - last_pc_data_time) < PC_TIMEOUT_MS;

        // 定时发送数据
        if (current_time - last_imu_send >= IMU_SEND_PERIOD) {
            SendIMUData();
            last_imu_send = current_time;
        }

        if (current_time - last_odom_send >= ODOM_SEND_PERIOD) {
            SendOdometryData();
            last_odom_send = current_time;
        }

        if (current_time - last_status_send >= STATUS_SEND_PERIOD) {
            SendSystemStatus();
            last_status_send = current_time;
        }

        osDelay(COMMUNICATION_TASK_DELAY);  // 50ms延迟
    }
}

//==================================================================================================
// LED控制模块
//==================================================================================================

static display::RGB* led = nullptr;

// LED任务属性定义
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

/**
 * @brief LED控制任务（RGB流水灯效果）
 */
extern "C" void ledTask(void* arg) {
    UNUSED(arg);

    int RGB_FLOW_COLOR_CHANGE_TIME = 300;
    uint32_t RGB_flow_color[3] = {0xFFFF0000, 0xFF00FF00, 0xFF0000FF};

    float delta_alpha, delta_red, delta_green, delta_blue;
    float alpha, red, green, blue;
    uint32_t aRGB;
    int i = 0;

    while (true) {
        alpha = (RGB_flow_color[i] & 0xFF000000) >> 24;
        red = ((RGB_flow_color[i] & 0x00FF0000) >> 16);
        green = ((RGB_flow_color[i] & 0x0000FF00) >> 8);
        blue = ((RGB_flow_color[i] & 0x000000FF) >> 0);

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

        for (int j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; ++j) {
            alpha += delta_alpha;
            red += delta_red;
            green += delta_green;
            blue += delta_blue;

            aRGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 |
                   ((uint32_t)(green)) << 8 | ((uint32_t)(blue)) << 0;

            if (led) {
                led->Display(aRGB);
            }

            osDelay(LED_TASK_DELAY);
        }

        ++i;
        i = i % 3;
    }
}

//==================================================================================================
// 控制模式管理
//==================================================================================================

/**
 * @brief 紧急停止所有电机
 */
void EmergencyStopAllMotors() {
    if (osMutexAcquire(motor_control_mutex, 100) == osOK) {  // 100ms超时，紧急情况下必须获得锁
        if (left_wheel_motor) left_wheel_motor->SetOutput(0);
        if (right_wheel_motor) right_wheel_motor->SetOutput(0);
        if (transform_motor) transform_motor->SetOutput(0);

        // 发送停止命令
        if (left_wheel_motor && right_wheel_motor &&
            (left_wheel_motor->connection_flag_ || right_wheel_motor->connection_flag_)) {
            control::MotorCANBase* motors[2] = {left_wheel_motor, right_wheel_motor};
            control::MotorCANBase::TransmitOutput(motors, 2);
        }

        if (transform_motor && transform_motor->connection_flag_) {
            control::MotorCANBase* motors[1] = {transform_motor};
            control::MotorCANBase::TransmitOutput(motors, 1);
        }

        osMutexRelease(motor_control_mutex);
    }
}

/**
 * @brief 判断当前控制模式
 */
control_mode_t GetControlMode() {
    if (!sbus || !sbus->connection_flag_) {
        return EMERGENCY_STOP_MODE;
    }

    uint32_t current_time = HAL_GetTick();
    if (current_time - sbus->timestamp > SBUS_TIMEOUT_MS) {
        return EMERGENCY_STOP_MODE;
    }

    // 根据通道6（ch[5]）判断控制模式
    int16_t mode_channel = sbus->ch[5];

    if (mode_channel < MODE_LOWER_THRESHOLD) {
        return EMERGENCY_STOP_MODE;      // 急停模式
    } else if (mode_channel > MODE_UPPER_THRESHOLD) {
        return COMPUTER_CONTROL_MODE;    // 上位机控制模式
    } else {
        return REMOTE_CONTROL_MODE;      // 遥控控制模式
    }
}

/**
 * @brief 判断当前变形模式
 */
transform_mode_t GetTransformMode() {
    if (!sbus || !sbus->connection_flag_) {
        return VEHICLE_MODE;  // 默认车载模式
    }

    // 根据通道5（ch[4]）判断变形模式
    int16_t transform_channel = sbus->ch[4];

    return (transform_channel > 0) ? FLIGHT_MODE : VEHICLE_MODE;
}

/**
 * @brief 处理控制模式切换
 */
void HandleModeSwitch() {
    if (current_control_mode != previous_control_mode) {
        switch (current_control_mode) {
            case EMERGENCY_STOP_MODE:
                // 急停模式：立即停止所有电机并重置PID
                EmergencyStopAllMotors();
                if (left_motor_pid) left_motor_pid->Reset();
                if (right_motor_pid) right_motor_pid->Reset();
                break;
            case REMOTE_CONTROL_MODE:
                // 遥控模式：重置PID控制器
                if (left_motor_pid) left_motor_pid->Reset();
                if (right_motor_pid) right_motor_pid->Reset();
                break;
            case COMPUTER_CONTROL_MODE:
                // 上位机模式：重置PID控制器
                if (left_motor_pid) left_motor_pid->Reset();
                if (right_motor_pid) right_motor_pid->Reset();
                break;
        }
        previous_control_mode = current_control_mode;
    }
}

//==================================================================================================
// RTOS初始化
//==================================================================================================

/**
 * @brief RTOS系统和硬件初始化
 */
extern "C" void RTOS_Init() {
    // 创建电机控制互斥锁
    motor_control_mutex = osMutexNew(NULL);
    if (motor_control_mutex == NULL) {
        // 互斥锁创建失败，系统无法正常工作
        while(1) {
            // 错误指示，可以添加LED闪烁或其他指示
            HAL_Delay(100);
        }
    }

    // 串口打印初始化
    print_use_uart(&huart1);

    // IMU初始化
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

    // SBUS遥控器初始化
    sbus = new remote::SBUS(&huart3);

    // 蜂鸣器初始化
    buzzer = new bsp::Buzzer(&htim4, 3, 1000000);

    // OLED显示器初始化
    OLED = new display::OLED(&hi2c2, 0x3C);

    // RGB LED初始化
    led = new display::RGB(&htim5, 3, 2, 1, 1000000);

    // CAN总线初始化
    can1 = new bsp::CAN(&hcan1, true);

    // 车辆电机初始化（左轮：CAN ID 1，右轮：CAN ID 2）
    left_wheel_motor = new control::Motor2006(can1, 0x201);   // 左轮电机
    right_wheel_motor = new control::Motor2006(can1, 0x202);  // 右轮电机

    // 变形电机初始化（3508：CAN ID 3）
    transform_motor = new control::Motor3508(can1, 0x203);    // 变形电机

    // PID控制器初始化
    left_motor_pid = new control::ConstrainedPID(left_motor_pid_params[0],
                                                 left_motor_pid_params[1],
                                                 left_motor_pid_params[2],
                                                 3000.0f, 10000.0f);

    right_motor_pid = new control::ConstrainedPID(right_motor_pid_params[0],
                                                  right_motor_pid_params[1],
                                                  right_motor_pid_params[2],
                                                  3000.0f, 10000.0f);

    // USB通信初始化
    comm_usb = new CustomCommUSB();
    comm_usb->SetupRx(512);
    comm_usb->SetupTx(512);

    // 初始化上位机连接状态检测
    last_pc_data_time = 0;  // 启动时设为0，确保在未收到数据前显示为离线状态

    // 吸盘风扇PWM初始化（TIM1_CH1）
    suction_fan = new bsp::PWM(&htim1, 1, FAN_TIMER_CLOCK, FAN_PWM_FREQUENCY, MIN_FAN_PULSE_WIDTH);

    // 电推杆GPIO初始化（PI7）
    actuator_gpio = new bsp::GPIO(GPIOI, GPIO_PIN_7);
    actuator_gpio->High();  // 初始化为高电平（无效状态）
}

//==================================================================================================
// RTOS任务初始化
//==================================================================================================

/**
 * @brief 创建所有RTOS任务
 */
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
// RTOS默认任务
//==================================================================================================

/**
 * @brief RTOS默认任务 - 管理控制模式
 */
extern "C" void RTOS_Default_Task(const void* args) {
    UNUSED(args);

    while (true) {
        // 获取当前控制模式和变形模式
        current_control_mode = GetControlMode();
        current_transform_mode = GetTransformMode();

        // 处理模式切换
        HandleModeSwitch();

        // 在急停模式下，持续确保所有电机停止
        if (current_control_mode == EMERGENCY_STOP_MODE) {
            EmergencyStopAllMotors();
        }

        osDelay(DEFAULT_TASK_DELAY);
    }
}