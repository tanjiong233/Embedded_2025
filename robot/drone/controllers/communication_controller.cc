#include "communication_controller.h"
#include "cmsis_os.h"

void CommunicationController::Init(SystemStateManager* state_manager,
                                   VehicleController* vehicle_ctrl,
                                   IMUController* imu_ctrl,
                                   SelfTestController* self_test) {
    state_manager_ = state_manager;
    vehicle_ctrl_ = vehicle_ctrl;
    imu_ctrl_ = imu_ctrl;
    self_test_ = self_test;

    comm_usb_ = std::make_unique<CustomCommUSB>();
    comm_usb_->SetupRx(512);
    comm_usb_->SetupTx(512);

    minipc_session_ = std::make_unique<communication::MinipcPort>();

    last_pc_data_time_ = 0;
}

void CommunicationController::SetTaskHandle(osThreadId_t handle) {
    if (comm_usb_) {
        comm_usb_->SetTaskHandle(handle);
    }
}

void CommunicationController::Execute() {
    uint32_t current_time = osKernelGetTickCount();

    // Check received data
    uint8_t* data;
    uint32_t length;

    uint32_t flags = osThreadFlagsWait(USB_RX_SIGNAL, osFlagsWaitAny, 0);
    if (flags & USB_RX_SIGNAL && comm_usb_) {
        length = comm_usb_->Read(&data);
        if (length > 0 && minipc_session_) {
            last_pc_data_time_ = current_time;

            minipc_session_->ParseUartBuffer(data, length);

            if (minipc_session_->GetValidFlag()) {
                uint8_t cmd_id = minipc_session_->GetCmdId();
                const communication::status_data_t* status = minipc_session_->GetStatus();

                switch (cmd_id) {
                    case communication::MOTION_CMD_ID:
                        HandleMotionCommand(status);
                        break;
                    case communication::SELFCHECK_CMD_ID:
                        HandleSelfcheckCommand(status);
                        break;
                    default:
                        break;
                }
            }
        }
    }

    // Update computer connection status
    bool computer_connected = (current_time - last_pc_data_time_) < PC_TIMEOUT_MS;
    if (self_test_) {
        self_test_->SetComputerStatus(computer_connected);
    }

    // Send periodic data
    if (current_time - last_imu_send_ >= IMU_SEND_PERIOD) {
        SendIMUData();
        last_imu_send_ = current_time;
    }

    if (current_time - last_odom_send_ >= ODOM_SEND_PERIOD) {
        SendOdometryData();
        last_odom_send_ = current_time;
    }

    if (current_time - last_status_send_ >= STATUS_SEND_PERIOD) {
        SendSystemStatus();
        last_status_send_ = current_time;
    }

    osDelay(TASK_DELAY_MS);
}

void CommunicationController::HandleMotionCommand(const communication::status_data_t* status) {
    if (!status || state_manager_->GetCurrentControlMode() != ControlMode::COMPUTER_CONTROL) return;

    if (status->emergency_stop) {
        vehicle_ctrl_->EmergencyStop();
        return;
    }

    vehicle_ctrl_->SetComputerControl(status->target_linear_vel, status->target_angular_vel);
}

void CommunicationController::HandleSelfcheckCommand(const communication::status_data_t* status) {
    if (!status) return;

    selfcheck_stats_.rx_count++;

    communication::selfcheck_data_t response;
    bool send_response = true;

    switch (status->mode) {
        case 0: // PING
            response.mode = 0;
            response.debug_int = status->debug_int;
            selfcheck_stats_.last_ping_time = osKernelGetTickCount();
            selfcheck_stats_.last_ping_seq = status->debug_int;
            break;

        case 1: // ECHO
            response.mode = 1;
            response.debug_int = status->debug_int;
            break;

        case 2: // STATUS
            response.mode = 2;
            response.debug_int = GetSelfcheckStatusValue(status->debug_int);
            break;

        case 3: // RESET
            response.mode = 3;
            response.debug_int = ExecuteSelfcheckReset(status->debug_int);
            break;

        default:
            selfcheck_stats_.error_count++;
            send_response = false;
            break;
    }

    if (send_response) {
        SendSelfcheckResponse(&response);
    }
}

uint8_t CommunicationController::GetSelfcheckStatusValue(uint8_t query_type) {
    switch (query_type) {
        case 0: return static_cast<uint8_t>(selfcheck_stats_.rx_count & 0xFF);
        case 1: return static_cast<uint8_t>((osKernelGetTickCount() / 1000) & 0xFF);
        case 2: return static_cast<uint8_t>(selfcheck_stats_.error_count & 0xFF);
        case 3: return static_cast<uint8_t>(selfcheck_stats_.tx_count & 0xFF);
        case 4:
            if (selfcheck_stats_.last_ping_time > 0) {
                uint32_t delay = osKernelGetTickCount() - selfcheck_stats_.last_ping_time;
                return static_cast<uint8_t>(delay > 255 ? 255 : delay);
            }
            return 255;
        default: return 0xFF;
    }
}

uint8_t CommunicationController::ExecuteSelfcheckReset(uint8_t reset_type) {
    switch (reset_type) {
        case 0:
            selfcheck_stats_.tx_count = 0;
            selfcheck_stats_.rx_count = 0;
            return 0;
        case 1:
            selfcheck_stats_.error_count = 0;
            return 0;
        case 2:
            selfcheck_stats_ = {};
            return 0;
        default:
            return 1;
    }
}

void CommunicationController::SendSelfcheckResponse(const communication::selfcheck_data_t* response_data) {
    if (!response_data || !minipc_session_ || !comm_usb_) return;

    uint8_t packet[minipc_session_->MAX_PACKET_LENGTH];
    minipc_session_->Pack(packet, (void*)response_data, communication::SELFCHECK_CMD_ID);
    comm_usb_->Write(packet, minipc_session_->GetPacketLen(communication::SELFCHECK_CMD_ID));

    selfcheck_stats_.tx_count++;
}

void CommunicationController::SendIMUData() {
    if (!imu_ctrl_ || !imu_ctrl_->GetIMU() || !minipc_session_ || !comm_usb_) return;

    communication::imu_data_t imu_data;
    auto imu = imu_ctrl_->GetIMU();

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
    imu_data.timestamp = osKernelGetTickCount();

    uint8_t packet[minipc_session_->MAX_PACKET_LENGTH];
    minipc_session_->Pack(packet, &imu_data, communication::IMU_CMD_ID);
    comm_usb_->Write(packet, minipc_session_->GetPacketLen(communication::IMU_CMD_ID));
}

void CommunicationController::SendOdometryData() {
    auto left_motor = vehicle_ctrl_->GetLeftMotor();
    auto right_motor = vehicle_ctrl_->GetRightMotor();

    if (!left_motor || !right_motor || !minipc_session_ || !comm_usb_) return;

    communication::odometry_data_t odom_data;

    static constexpr float WHEEL_RADIUS = 0.076f;
    static constexpr float WHEEL_BASE = 0.335f;

    float left_wheel_speed = left_motor->GetOmega();
    float right_wheel_speed = right_motor->GetOmega();

    float linear_vel = (left_wheel_speed + right_wheel_speed) * WHEEL_RADIUS / 2.0f;
    float angular_vel = (right_wheel_speed - left_wheel_speed) * WHEEL_RADIUS / WHEEL_BASE;

    static int32_t left_encoder_total = 0;
    static int32_t right_encoder_total = 0;
    left_encoder_total += static_cast<int32_t>(left_wheel_speed * 0.05f * 8192 / (2 * 3.14159f));
    right_encoder_total += static_cast<int32_t>(right_wheel_speed * 0.05f * 8192 / (2 * 3.14159f));

    odom_data.left_encoder = left_encoder_total;
    odom_data.right_encoder = right_encoder_total;
    odom_data.left_wheel_speed = left_wheel_speed;
    odom_data.right_wheel_speed = right_wheel_speed;
    odom_data.linear_velocity = linear_vel;
    odom_data.angular_velocity = angular_vel;
    odom_data.wheel_base = WHEEL_BASE;
    odom_data.wheel_radius = WHEEL_RADIUS;
    odom_data.timestamp = osKernelGetTickCount();

    uint8_t packet[minipc_session_->MAX_PACKET_LENGTH];
    minipc_session_->Pack(packet, &odom_data, communication::ODOMETRY_CMD_ID);
    comm_usb_->Write(packet, minipc_session_->GetPacketLen(communication::ODOMETRY_CMD_ID));
}

void CommunicationController::SendSystemStatus() {
    if (!minipc_session_ || !comm_usb_) return;

    communication::system_status_t status;

    status.robot_mode = static_cast<uint8_t>(state_manager_->GetCurrentControlMode());
    status.transform_mode = static_cast<uint8_t>(state_manager_->GetCurrentTransformMode());
    status.sbus_connected = (vehicle_ctrl_->GetLeftMotor() && vehicle_ctrl_->GetLeftMotor()->connection_flag_) ? 1 : 0;
    status.imu_connected = imu_ctrl_->GetIMU() ? 1 : 0;
    status.vl_motor_online = (vehicle_ctrl_->GetLeftMotor() && vehicle_ctrl_->GetLeftMotor()->connection_flag_) ? 1 : 0;
    status.vr_motor_online = (vehicle_ctrl_->GetRightMotor() && vehicle_ctrl_->GetRightMotor()->connection_flag_) ? 1 : 0;
    status.tf_motor_online = 0; // Not implemented yet
    status.reserved = 0;
    status.timestamp = osKernelGetTickCount();

    uint8_t packet[minipc_session_->MAX_PACKET_LENGTH];
    minipc_session_->Pack(packet, &status, communication::SYSTEM_STATUS_CMD_ID);
    comm_usb_->Write(packet, minipc_session_->GetPacketLen(communication::SYSTEM_STATUS_CMD_ID));
}