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


// Params Initialization
static control::MotorCANBase* vl_motor = nullptr;
static control::MotorCANBase* vr_motor = nullptr;

// Vehicle control parameters
static const float MAX_VEHICLE_SPEED = 5.0f;  // Maximum vehicle speed in rad/s
static const float TURN_SENSITIVITY = 0.5f;   // Turning sensitivity factor
static const int SBUS_MAX_VALUE = 660;        // SBUS maximum channel value
static const int SBUS_DEADZONE = 50;          // Dead zone for SBUS channels

// PID parameters for motor speed control
static float vl_motor_pid_params[3] = {8.0f, 0.1f, 0.0f};  // [kp, ki, kd] for left motor
static float vr_motor_pid_params[3] = {8.0f, 0.1f, 0.0f};  // [kp, ki, kd] for right motor

// PID controllers for vehicle motors
static control::ConstrainedPID* vl_motor_pid = nullptr;
static control::ConstrainedPID* vr_motor_pid = nullptr;

// Vehicle state variables
static float target_linear_speed = 0.0f;   // Target linear speed
static float target_angular_speed = 0.0f;  // Target angular speed
static float left_wheel_speed = 0.0f;      // Target left wheel speed
static float right_wheel_speed = 0.0f;     // Target right wheel speed

// Function to apply dead zone to SBUS input
int16_t ApplyDeadzone(int16_t input, int16_t deadzone) {
    if (abs(input) < deadzone) {
        return 0;
    }
    return input;
}

// Function to normalize SBUS input to [-1, 1]
float NormalizeSBUS(int16_t input) {
    return (float)input / SBUS_MAX_VALUE;
}

// Function to calculate differential drive speeds
void CalculateDifferentialSpeeds(float linear_speed, float angular_speed,
                                 float* left_speed, float* right_speed) {
    // Differential drive kinematics
    // For a differential drive robot:
    // v_left = linear_speed - (wheel_base/2) * angular_speed
    // v_right = linear_speed + (wheel_base/2) * angular_speed
    //
    // Simplified version where we directly use angular_speed as turn rate
    *left_speed = linear_speed - angular_speed;
    *right_speed = linear_speed + angular_speed;

    // Limit speeds to maximum
    *left_speed = clip<float>(*left_speed, -MAX_VEHICLE_SPEED, MAX_VEHICLE_SPEED);
    *right_speed = clip<float>(*right_speed, -MAX_VEHICLE_SPEED, MAX_VEHICLE_SPEED);
}

extern "C" void vehicleTask(void* arg) {
    UNUSED(arg);
    while (true){
        // Check if SBUS is connected
        if (!sbus->connection_flag_) {
            // If no SBUS connection, stop motors
            vl_motor->SetOutput(0);
            vr_motor->SetOutput(0);
//            control::MotorCANBase* motors[2] = {vl_motor, vr_motor};
//            control::MotorCANBase::TransmitOutput(motors, 2);
            osDelay(VEHICLE_TASK_DELAY);
            continue;
        }

        // Read SBUS channels with dead zone
        int16_t throttle_raw = ApplyDeadzone(sbus->ch[2], SBUS_DEADZONE);    // Channel 3 (throttle)
        int16_t steering_raw = ApplyDeadzone(sbus->ch[3], SBUS_DEADZONE);    // Channel 4 (steering)

        // Normalize inputs to [-1, 1]
        float throttle = NormalizeSBUS(throttle_raw);
        float steering = NormalizeSBUS(steering_raw);

        // Calculate target speeds
        target_linear_speed = throttle * MAX_VEHICLE_SPEED;
        target_angular_speed = steering * MAX_VEHICLE_SPEED * TURN_SENSITIVITY;

        // Calculate differential speeds for left and right wheels
        CalculateDifferentialSpeeds(target_linear_speed, target_angular_speed,
                                    &left_wheel_speed, &right_wheel_speed);

        // Calculate PID outputs
        float left_error = left_wheel_speed - vl_motor->GetOmega();
        float right_error = right_wheel_speed - vr_motor->GetOmega();

        int16_t left_output = vl_motor_pid->ComputeConstrainedOutput(left_error);
        int16_t right_output = vr_motor_pid->ComputeConstrainedOutput(right_error);

        // Set motor outputs
        vl_motor->SetOutput(left_output);
        vr_motor->SetOutput(right_output);

        // Transmit CAN messages for both motors
        control::MotorCANBase* motors[2] = {vl_motor, vr_motor};
        control::MotorCANBase::TransmitOutput(motors, 2);

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
        .priority = (osPriority_t)osPriorityNormal,
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
        .priority = (osPriority_t)osPriorityNormal,
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
        .priority = (osPriority_t)osPriorityNormal,
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
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};
osThreadId_t communicationTaskHandle;

class CustomCommUSB : public bsp::VirtualUSB {
protected:
    void RxCompleteCallback() override final { osThreadFlagsSet(communicationTaskHandle, USB_RX_SIGNAL); }
};


static CustomCommUSB* comm_usb = nullptr;
static communication::MinipcPort* minipc_session = nullptr;
static uint32_t send_counter = 0;


// 从系统获取时间戳的函数
uint32_t GetSystemTimestamp() {
    return osKernelGetTickCount();
}


// 发送IMU数据
void SendIMUData() {
    if (imu == nullptr) return;

    communication::imu_data_t imu_data;

    // 使用新添加的公共接口获取IMU数据
    const float* accel_data = imu->GetAccel();
    const float* gyro_data = imu->GetGyro();

    imu_data.accel_x = accel_data[0];
    imu_data.accel_y = accel_data[1];
    imu_data.accel_z = accel_data[2];
    imu_data.gyro_x = gyro_data[0];
    imu_data.gyro_y = gyro_data[1];
    imu_data.gyro_z = gyro_data[2];

    // 使用公共成员的姿态角数据
    imu_data.pitch = imu->INS_angle[1];  // INS_PITCH_ADDRESS_OFFSET = 1
    imu_data.roll = imu->INS_angle[2];   // INS_ROLL_ADDRESS_OFFSET = 2
    imu_data.yaw = imu->INS_angle[0];    // INS_YAW_ADDRESS_OFFSET = 0
    imu_data.temperature = imu->Temp;    // 公共成员
    imu_data.timestamp = GetSystemTimestamp();

    // 打包并发送
    uint8_t packet[minipc_session->MAX_PACKET_LENGTH];
    minipc_session->Pack(packet, &imu_data, communication::IMU_CMD_ID);
    comm_usb->Write(packet, minipc_session->GetPacketLen(communication::IMU_CMD_ID));
}


// 发送里程计数据
void SendOdometryData() {
    if (vl_motor == nullptr || vr_motor == nullptr) return;

    communication::odometry_data_t odom_data;

    // 计算轮式里程计数据
    static const float WHEEL_RADIUS = 0.076f;    // 轮子半径 76mm
    static const float WHEEL_BASE = 0.335f;      // 轮距 335mm

    // 获取电机转速并转换为轮速
    float left_wheel_speed = vl_motor->GetOmega();   // rad/s
    float right_wheel_speed = vr_motor->GetOmega();  // rad/s

    // 计算机器人线速度和角速度
    float linear_vel = (left_wheel_speed + right_wheel_speed) * WHEEL_RADIUS / 2.0f;
    float angular_vel = (right_wheel_speed - left_wheel_speed) * WHEEL_RADIUS / WHEEL_BASE;

    // 累积编码器计数（简化实现）
    static int32_t left_encoder_total = 0;
    static int32_t right_encoder_total = 0;
    left_encoder_total += (int32_t)(left_wheel_speed * 0.02f * 8192 / (2 * 3.14159f)); // 假设2ms周期
    right_encoder_total += (int32_t)(right_wheel_speed * 0.02f * 8192 / (2 * 3.14159f));

    // 填充里程计数据
    odom_data.left_encoder = left_encoder_total;
    odom_data.right_encoder = right_encoder_total;
    odom_data.left_wheel_speed = left_wheel_speed;
    odom_data.right_wheel_speed = right_wheel_speed;
    odom_data.linear_velocity = linear_vel;
    odom_data.angular_velocity = angular_vel;
    odom_data.wheel_base = WHEEL_BASE;
    odom_data.wheel_radius = WHEEL_RADIUS;
    odom_data.timestamp = GetSystemTimestamp();

    // 打包并发送
    uint8_t packet[minipc_session->MAX_PACKET_LENGTH];
    minipc_session->Pack(packet, &odom_data, communication::ODOMETRY_CMD_ID);
    comm_usb->Write(packet, minipc_session->GetPacketLen(communication::ODOMETRY_CMD_ID));
}


// 发送系统状态
void SendSystemStatus() {
    communication::system_status_t status;

    // 机器人模式判断
    if (sbus != nullptr && sbus->connection_flag_) {
        if (sbus->ch[4] > 500) {  // 通道5上位，飞行模式
            status.robot_mode = 1;  // 自动导航模式
            status.transform_mode = 1;  // 飞行模式
        } else {
            status.robot_mode = 0;  // 手动模式
            status.transform_mode = 0;  // 车载模式
        }
    } else {
        status.robot_mode = 0;
        status.transform_mode = 0;
    }

    // 设备连接状态
    status.sbus_connected = (sbus != nullptr) ? sbus->connection_flag_ : 0;
    status.imu_connected = (imu != nullptr) ? 1 : 0;  // IMU一般都在线
    status.vl_motor_online = (vl_motor != nullptr) ? vl_motor->connection_flag_ : 0;
    status.vr_motor_online = (vr_motor != nullptr) ? vr_motor->connection_flag_ : 0;
    status.tf_motor_online = (tf_motor != nullptr) ? tf_motor->connection_flag_ : 0;
    status.reserved = 0;

    status.timestamp = GetSystemTimestamp();

    // 打包并发送
    uint8_t packet[minipc_session->MAX_PACKET_LENGTH];
    minipc_session->Pack(packet, &status, communication::SYSTEM_STATUS_CMD_ID);
    comm_usb->Write(packet, minipc_session->GetPacketLen(communication::SYSTEM_STATUS_CMD_ID));
}


// 处理接收到的运动控制指令
void HandleMotionCommand(const communication::status_data_t* status) {
    if (status == nullptr) return;

    // 获取目标速度
    float target_linear = status->target_linear_vel;   // m/s
    float target_angular = status->target_angular_vel; // rad/s

    // 检查紧急停止
    if (status->emergency_stop) {
        // 紧急停止，直接设置电机输出为0
        if (vl_motor) vl_motor->SetOutput(0);
        if (vr_motor) vr_motor->SetOutput(0);
        return;
    }

    // 运动学逆解：将线速度和角速度转换为左右轮速度
    static const float WHEEL_RADIUS = 0.076f;    // 轮子半径
    static const float WHEEL_BASE = 0.335f;      // 轮距

    float left_wheel_vel = (target_linear - target_angular * WHEEL_BASE / 2.0f) / WHEEL_RADIUS;
    float right_wheel_vel = (target_linear + target_angular * WHEEL_BASE / 2.0f) / WHEEL_RADIUS;

    // 速度限制
    const float MAX_WHEEL_SPEED = 10.0f; // rad/s
    left_wheel_vel = clip<float>(left_wheel_vel, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
    right_wheel_vel = clip<float>(right_wheel_vel, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);

    // 使用现有的PID控制器进行速度控制
    if (vl_motor && vr_motor && vl_motor_pid && vr_motor_pid) {
        // 计算速度误差
        float left_error = left_wheel_vel - vl_motor->GetOmega();
        float right_error = right_wheel_vel - vr_motor->GetOmega();

        // 使用PID控制器计算输出
        int16_t left_output = vl_motor_pid->ComputeConstrainedOutput(left_error);
        int16_t right_output = vr_motor_pid->ComputeConstrainedOutput(right_error);

        vl_motor->SetOutput(left_output);
        vr_motor->SetOutput(right_output);
    }
}


extern "C" void communicationTask(void* arg) {
    UNUSED(arg);

    // 初始化协议会话
    minipc_session = new communication::MinipcPort();

    uint8_t *data;
    uint32_t length;
    uint32_t last_imu_send = 0;      // 100Hz发送IMU
    uint32_t last_odom_send = 0;     // 50Hz发送里程计
    uint32_t last_status_send = 0;   // 5Hz发送系统状态

    const uint32_t IMU_SEND_PERIOD = 10;      // 10ms = 100Hz
    const uint32_t ODOM_SEND_PERIOD = 20;     // 20ms = 50Hz
    const uint32_t STATUS_SEND_PERIOD = 200;  // 200ms = 5Hz

    while(true) {
        uint32_t current_time = osKernelGetTickCount();

        // 检查是否有接收数据
        uint32_t flags = osThreadFlagsWait(USB_RX_SIGNAL, osFlagsWaitAny, 0);

        if (flags & USB_RX_SIGNAL) {
            // 处理接收到的数据
            length = comm_usb->Read(&data);
            if (length > 0) {
                minipc_session->ParseUartBuffer(data, length);

                // 检查数据是否有效
                if (minipc_session->GetValidFlag()) {
                    uint8_t cmd_id = minipc_session->GetCmdId();
                    const communication::status_data_t* status = minipc_session->GetStatus();

                    switch (cmd_id) {
                        case communication::MOTION_CMD_ID:
                            // 处理运动控制指令
                            HandleMotionCommand(status);
                            break;

                        case communication::SELFCHECK_CMD_ID:
                            // 处理自检指令
                            if (status->mode == 1) {
                                // Echo模式，原样返回
                                communication::selfcheck_data_t selfcheck_data;
                                selfcheck_data.mode = 1;
                                selfcheck_data.debug_int = status->debug_int;

                                uint8_t packet[minipc_session->MAX_PACKET_LENGTH];
                                minipc_session->Pack(packet, &selfcheck_data, communication::SELFCHECK_CMD_ID);
                                comm_usb->Write(packet, minipc_session->GetPacketLen(communication::SELFCHECK_CMD_ID));
                            } else if (status->mode == 2) {
                                // ID模式，返回设备ID
                                communication::selfcheck_data_t selfcheck_data;
                                selfcheck_data.mode = 2;
                                selfcheck_data.debug_int = 130; // 设备ID

                                uint8_t packet[minipc_session->MAX_PACKET_LENGTH];
                                minipc_session->Pack(packet, &selfcheck_data, communication::SELFCHECK_CMD_ID);
                                comm_usb->Write(packet, minipc_session->GetPacketLen(communication::SELFCHECK_CMD_ID));
                            }
                            break;

                        default:
                            // 其他指令暂不处理
                            break;
                    }
                }
            }
        }

        // 定时发送数据
        // 100Hz发送IMU数据
        if (current_time - last_imu_send >= IMU_SEND_PERIOD) {
            SendIMUData();
            last_imu_send = current_time;
        }

        // 50Hz发送里程计数据
        if (current_time - last_odom_send >= ODOM_SEND_PERIOD) {
            SendOdometryData();
            last_odom_send = current_time;
        }

        // 5Hz发送系统状态
        if (current_time - last_status_send >= STATUS_SEND_PERIOD) {
            SendSystemStatus();
            last_status_send = current_time;

            // 更新上位机连接标志
            computer_flag = true; // 简化实现，实际应根据通信情况判断
        }

        osDelay(1); // 1ms延迟，实际约1000Hz任务频率
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
    vl_motor_pid = new control::ConstrainedPID(vl_motor_pid_params[0],
                                               vl_motor_pid_params[1],
                                               vl_motor_pid_params[2],
                                               3000.0f,  // max_iout
                                               10000.0f); // max_out

    vr_motor_pid = new control::ConstrainedPID(vr_motor_pid_params[0],
                                               vr_motor_pid_params[1],
                                               vr_motor_pid_params[2],
                                               3000.0f,  // max_iout
                                               10000.0f); // max_out
    // Transform initialization
    tf_motor = new control::Motor3508(can1, 0x203);  // 变形电机，CAN ID 3
    // USB
    comm_usb = new CustomCommUSB();
    comm_usb->SetupRx(512);
    comm_usb->SetupTx(512);
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
