/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2023 RoboMaster.                                          *
 *  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                          *
 ****************************************************************************/

#pragma once

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "crc8.h"

namespace communication {

// WARNING: CRC8 works only for total length < 64 bytes.
typedef struct {
  char header[2];
  uint16_t seq_num;
  // data_length might not be necessary because all command length are fixed
  uint8_t data_length; // length of data as in bytes, data length as an arry * 4 = data_length
  uint8_t cmd_id;
  int32_t *data;
  uint8_t crc8_checksum;
  char tail[2];
} __packed minipc_data_t;
// This might be not necessary at all

// RED is 0; BLUE is one
typedef struct {
  uint8_t my_color;
} __packed color_data_t;

typedef struct {
  float rel_yaw;
  float rel_pitch;
  // Search Target is 0. Move Yoke is 1.
  uint8_t mode;
  uint8_t debug_int;
} __packed gimbal_data_t;

typedef struct {
  float vx;
  float vy;
  float vw;
} __packed chassis_data_t;

typedef struct {
  // FLUSH is 0. ECHO is 1. ID is 2.
  uint8_t mode;
  uint8_t debug_int;
} __packed selfcheck_data_t;

typedef struct {
  float floats[6];
} __packed arm_data_t;

// summary of all information transmitted between minipc and stm32
    typedef struct {
        uint8_t my_color;
        float rel_yaw;
        float rel_pitch;
        uint8_t mode;
        uint8_t debug_int;
        float vx;
        float vy;
        float vw;
        float floats[6];

        float accel_x, accel_y, accel_z;
        float gyro_x, gyro_y, gyro_z;
        float pitch, roll, yaw;
        float imu_temperature;
        uint32_t imu_timestamp;

        int32_t left_encoder, right_encoder;
        float left_wheel_speed, right_wheel_speed;
        float linear_velocity, angular_velocity;
        float wheel_base, wheel_radius;
        uint32_t odometry_timestamp;

        uint8_t robot_mode;
        uint8_t transform_mode;
        uint8_t device_status;
        uint32_t system_timestamp;

        float target_linear_vel, target_angular_vel;
        uint8_t emergency_stop;
        uint32_t motion_timestamp;
    } __packed status_data_t;

/* ===== IMU_DATA 0x05 100Hz (44字节) ===== */
typedef struct {
    float accel_x;              // 加速度X (m/s²)
    float accel_y;              // 加速度Y (m/s²)
    float accel_z;              // 加速度Z (m/s²)
    float gyro_x;               // 角速度X (rad/s)
    float gyro_y;               // 角速度Y (rad/s)
    float gyro_z;               // 角速度Z (rad/s)
    float pitch;                // 俯仰角 (rad)
    float roll;                 // 横滚角 (rad)
    float yaw;                  // 偏航角 (rad)
    float temperature;          // 温度 (℃)
    uint32_t timestamp;         // 时间戳 (毫秒)
} __packed imu_data_t;

/* ===== ODOMETRY_DATA 0x06 50Hz (36字节) ===== */
// 轮式里程计数据，用于位置估计和建图
typedef struct {
    int32_t left_encoder;       // 左轮编码器累积计数
    int32_t right_encoder;      // 右轮编码器累积计数
    float left_wheel_speed;     // 左轮实际速度 (rad/s)
    float right_wheel_speed;    // 右轮实际速度 (rad/s)
    float linear_velocity;      // 机器人线速度 (m/s)
    float angular_velocity;     // 机器人角速度 (rad/s)
    float wheel_base;           // 轮距 (m)
    float wheel_radius;         // 轮子半径 (m)
    uint32_t timestamp;         // 时间戳 (毫秒)
} __packed odometry_data_t;

/* ===== SYSTEM_STATUS 0x07 5Hz (8字节) ===== */
// 系统整体状态，用于监控和诊断
typedef struct {
    uint8_t robot_mode;         // 0:手动, 1:自动导航, 2:建图模式
    uint8_t transform_mode;     // 0:车载模式, 1:飞行模式, 2:变形中

    // 设备连接状态（位域）
    uint8_t sbus_connected : 1;     // SBUS遥控器连接
    uint8_t imu_connected : 1;      // IMU连接状态
    uint8_t vl_motor_online : 1;    // 左轮电机在线
    uint8_t vr_motor_online : 1;    // 右轮电机在线
    uint8_t tf_motor_online : 1;    // 变形电机在线
    uint8_t reserved : 3;           // 保留位

    uint32_t timestamp;         // 时间戳 (毫秒)
} __packed system_status_t;

/* ===== MOTION_CMD 0x08 (12字节) ===== */
// 运动控制指令，从上位机发送给下位机
typedef struct {
    float target_linear_vel;    // 目标线速度 (m/s)
    float target_angular_vel;   // 目标角速度 (rad/s)
    uint8_t emergency_stop;     // 紧急停止标志 (0:正常, 1:停止)
    uint8_t reserved[3];        // 保留字节，4字节对齐
    uint32_t timestamp;         // 时间戳 (毫秒)
} __packed motion_cmd_t;


// GIMBAL_CMD_ID  : 0x00 Autoaim gimbal RelYaw RelPitch
// COLOR_CMD_ID   : 0x01
// CHASSIS_CMD_ID : 0x02
// SELFCHECK_CMD_ID :0x03
// ARM_CMD-ID     : 0x04
// IMU_CMD_ID     : 0x05 IMU sensor data
// ODOMETRY_CMD_ID: 0x06 Wheel odometry data
// SYSTEM_STATUS_CMD_ID: 0x07 System status
// MOTION_CMD_ID  : 0x08 Motion commands
// TOTAL_NUM_OF_ID: length of the enum
enum CMD_ID {
    GIMBAL_CMD_ID = 0x00,
    COLOR_CMD_ID = 0x01,
    CHASSIS_CMD_ID = 0x02,
    SELFCHECK_CMD_ID = 0x03,
    ARM_CMD_ID = 0x04,
    // 新增建图导航命令ID
    IMU_CMD_ID = 0x05,           // IMU数据
    ODOMETRY_CMD_ID = 0x06,      // 里程计数据
    SYSTEM_STATUS_CMD_ID = 0x07, // 系统状态
    MOTION_CMD_ID = 0x08,        // 运动指令
    TOTAL_NUM_OF_ID
};

// WARNING: THIS CLASS IS NOT THREAD SAFE!!!
// See docs/comm_protocol.md in vision repo for docs
class MinipcPort {
 public:
  MinipcPort();

  /**
   * @brief Pack data into a packet array
   * @note For the smallest length of the packet, see CMD_TO_LEN[] or GetPacketLength()
   */
  void Pack(uint8_t* packet, void* data, uint8_t cmd_id);
  void PackGimbalData(uint8_t* packet, gimbal_data_t* data);
  void PackColorData(uint8_t* packet, color_data_t* data);
  void PackChassisData(uint8_t* packet, chassis_data_t* data);
  void PackSelfcheckData(uint8_t* packet, selfcheck_data_t* data);
  void PackArmData(uint8_t* packet, arm_data_t* data);
  void PackIMUData(uint8_t* packet, imu_data_t* data);
  void PackOdometryData(uint8_t* packet, odometry_data_t* data);
  void PackSystemStatus(uint8_t* packet, system_status_t* data);
  void PackMotionCmd(uint8_t* packet, motion_cmd_t* data);

  /**
   * @brief Total length of packet in bytes
   *  Header/tail/crc8 included.
   */
  uint8_t GetPacketLen(uint8_t cmd_id);

  /**
   * @brief parse and handle the uart buffer
   * @param data: pointer to the uart buffer
   *        len:  length of the data received in bytes
   */
  void ParseUartBuffer(const uint8_t* data, int32_t len);

  /**
   * @brief Return the cmd_id of the most recently parsed packet
   */
  uint8_t GetCmdId(void);

  /**
   * @brief Get command status of the robot
   */
  const status_data_t* GetStatus(void);

  /**
   * @brief Get the valid flag, 1 when the packet is valid, 0 otherwise
   * @note  Flag can only be acquired once. Once asked, flag will be reset to 0 (invalid)
   */
  uint8_t GetValidFlag(void);
  uint16_t GetSeqnum(void);
  uint32_t GetValidPacketCnt(void);

    /**
     * Length of the data section ONLY in bytes. Header/tail/crc8 (total len = 9) NOT included.
     * Gimbal       CMD: id = 0x00, total packet length = 19 - 9 = 10
     * Color        CMD: id = 0x01, total packet length = 10 - 9 = 1
     * Chassis      CMD: id = 0x02, total packet length = 21 - 9 = 12
     * Selfcheck    CMD: id = 0x03, total packet length = 11 - 9 = 2
     * Arm          CMD: id = 0x04, total packet length = 33 - 9 = 24
     * IMU          CMD: id = 0x05, total packet length = 53 - 9 = 44
     * Odometry     CMD: id = 0x06, total packet length = 45 - 9 = 36
     * SystemStatus CMD: id = 0x07, total packet length = 17 - 9 = 8
     * MotionCmd    CMD: id = 0x08, total packet length = 21 - 9 = 12
     */
    static constexpr uint8_t CMD_TO_LEN[TOTAL_NUM_OF_ID] = {
            sizeof(gimbal_data_t),      // GIMBAL_CMD_ID = 10字节
            sizeof(color_data_t),       // COLOR_CMD_ID = 1字节
            sizeof(chassis_data_t),     // CHASSIS_CMD_ID = 12字节
            sizeof(selfcheck_data_t),   // SELFCHECK_CMD_ID = 2字节
            sizeof(arm_data_t),         // ARM_CMD_ID = 24字节
            sizeof(imu_data_t),         // IMU_CMD_ID = 44字节
            sizeof(odometry_data_t),    // ODOMETRY_CMD_ID = 36字节
            sizeof(system_status_t),    // SYSTEM_STATUS_CMD_ID = 8字节
            sizeof(motion_cmd_t),       // MOTION_CMD_ID = 16字节
    };

    static constexpr uint8_t MAX_PACKET_LENGTH = 53;  // 更新最大包长度 (44+9)
    static constexpr uint8_t MIN_PACKET_LENGTH = 10;
    // sum of header and tail = 9. Total packet length = data length (CMD_TO_LEN) + 9
    static constexpr uint8_t HT_LEN = 9;

 private:
  /**
   * @brief Add header and tail to the packet array based on cmd_id
   * @note For the smallest length of the packet, see CMD_TO_LEN[]
   *       The sum of length for header and tail is 8 bytes
   */
  void AddHeaderTail (uint8_t* packet, uint8_t cmd_id);

  /**
   * @brief Add CRC8 checksum for the packet array based on cmd_id
   *       CRC8 calulated based on the entire array except the tail ('ED')
   * @note Only call this function after packet has data and header/tails written
   *       The length of CRC8 checksum is 1 byte
   */
  void AddCRC8 (uint8_t* packet, int8_t cmd_id);

  // Wrapper of ParseData(uint8_t), do some verification.
  void VerifyAndParseData();

  // Assume that the possible_packet is a complete and verified message
  void ParseData(uint8_t cmd_id);

  uint8_t cmd_id_;
  status_data_t status_;
  uint8_t possible_packet[MAX_PACKET_LENGTH];
  // keep track of the index of the current packet
  // in case of 1 packet being sent in multiple uart transmissions
  int buffer_index_;

  // Least current available sequence number
  uint16_t seqnum_;
  uint8_t valid_flag_;
  uint32_t valid_packet_cnt_ = 0;
}; /* class MinipcPort */

} /* namespace communication */

