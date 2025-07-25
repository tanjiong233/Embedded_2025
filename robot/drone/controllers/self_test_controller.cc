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

#include "self_test_controller.h"
#include "cmsis_os.h"

void SelfTestController::Init(I2C_HandleTypeDef* hi2c, TIM_HandleTypeDef* htim,
                              VehicleController* vehicle_ctrl,
                              TransformController* transform_ctrl,
                              remote::SBUS* sbus,
                              SystemStateManager* state_manager,
                              IMUController* imu_ctrl) {
    vehicle_ctrl_ = vehicle_ctrl;
    transform_ctrl_ = transform_ctrl;
    sbus_ = sbus;
    state_manager_ = state_manager;
    imu_ctrl_ = imu_ctrl;

    OLED_ = std::make_unique<display::OLED>(hi2c, 0x3C);
    buzzer_ = std::make_unique<bsp::Buzzer>(htim, 3, 1000000);
}

void SelfTestController::Execute() {
    if (!startup_sequence_played_) {
        ShowStartupSequence();
        startup_sequence_played_ = true;
    }
    UpdateDeviceStatus();
    UpdateOLEDDisplay();
    osDelay(TASK_DELAY_MS);
}

void SelfTestController::SetComputerStatus(bool connected) {
    device_status_.computer = connected;
}

void SelfTestController::ShowStartupSequence() {
    if (!OLED_ || !buzzer_) return;

    OLED_->ShowIlliniRMLOGO();

    // Super Mario theme
    using Note = bsp::BuzzerNote;
    static bsp::BuzzerNoteDelayed Mario[] = {
            {Note::Mi3M, 80}, {Note::Silent, 80}, {Note::Mi3M, 80}, {Note::Silent, 240},
            {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::Do1M, 80}, {Note::Silent, 80},
            {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::So5M, 80}, {Note::Silent, 560},
            {Note::So5L, 80}, {Note::Silent, 0}, {Note::Finish, 0}
    };

    buzzer_->SingSong(Mario, [](uint32_t milli) { osDelay(milli); });
    OLED_->OperateGram(display::PEN_CLEAR);
    OLED_->RefreshGram();

    // Display fixed labels
    OLED_->ShowString(0, 0, (uint8_t*)"LW");
    OLED_->ShowString(0, 4, (uint8_t*)"RW");
    OLED_->ShowString(0, 8, (uint8_t*)"TF");
    OLED_->ShowString(0, 12, (uint8_t*)"RC");
    OLED_->ShowString(0, 16, (uint8_t*)"PC");

    start_time_ = osKernelGetTickCount();
}

void SelfTestController::UpdateDeviceStatus() {
    // Reset connection flags
    if (auto left = vehicle_ctrl_->GetLeftMotor()) left->connection_flag_ = false;
    if (auto right = vehicle_ctrl_->GetRightMotor()) right->connection_flag_ = false;
    if (auto transform = transform_ctrl_->GetTransformMotor()) transform->connection_flag_ = false;
    if (sbus_) sbus_->connection_flag_ = false;

    osDelay(TASK_DELAY_MS);

    // Read status
    auto* left_motor = vehicle_ctrl_->GetLeftMotor();
    auto* right_motor = vehicle_ctrl_->GetRightMotor();
    auto* transform_motor = transform_ctrl_->GetTransformMotor();

    device_status_.left_motor = left_motor && left_motor->connection_flag_;
    device_status_.right_motor = right_motor && right_motor->connection_flag_;
    device_status_.transform_motor = transform_motor && transform_motor->connection_flag_;
    device_status_.sbus = sbus_ && sbus_->connection_flag_;
}

void SelfTestController::UpdateOLEDDisplay() {
    if (!OLED_) return;

    // Row 0: Device status indicators
    OLED_->ShowBlock(0, 2, device_status_.left_motor);
    OLED_->ShowBlock(0, 6, device_status_.right_motor);
    OLED_->ShowBlock(0, 10, device_status_.transform_motor);
    OLED_->ShowBlock(0, 14, device_status_.sbus);
    OLED_->ShowBlock(0, 18, device_status_.computer);

    // Row 1: Control mode + Transform mode + Runtime
    const char* mode_str = state_manager_->GetCurrentControlMode() == ControlMode::EMERGENCY_STOP ? "STOP" :
                           state_manager_->GetCurrentControlMode() == ControlMode::REMOTE_CONTROL ? "RC" : "PC";
    const char* transform_str = state_manager_->GetCurrentTransformMode() == TransformMode::VEHICLE ? "CAR" : "FLY";

    uint32_t runtime_sec = (osKernelGetTickCount() - start_time_) / 1000;
    uint32_t hours = runtime_sec / 3600;
    uint32_t minutes = (runtime_sec % 3600) / 60;
    uint32_t seconds = runtime_sec % 60;

    OLED_->Printf(1, 0, "M:%s T:%s %02d:%02d:%02d",
                  mode_str, transform_str, hours, minutes, seconds);

    // Row 2: Transform motor angle
    if (auto motor = transform_ctrl_->GetTransformMotor(); motor && motor->connection_flag_) {
        int angle_int = static_cast<int>(motor->GetTheta() * 573);
        OLED_->Printf(2, 0, "TF: %4d.%d deg", angle_int/10, abs(angle_int%10));
    } else {
        OLED_->Printf(2, 0, "TF: --- deg");
    }

    // Row 3: IMU Euler angles
    if (auto imu = imu_ctrl_->GetIMU()) {
        int pitch_int = static_cast<int>(imu->INS_angle[1] * 573);
        int roll_int = static_cast<int>(imu->INS_angle[2] * 573);
        int yaw_int = static_cast<int>(imu->INS_angle[0] * 573);

        OLED_->Printf(3, 0, "P:%d.%d R:%d.%d Y:%d.%d",
                      pitch_int/10, abs(pitch_int%10),
                      roll_int/10, abs(roll_int%10),
                      yaw_int/10, abs(yaw_int%10));
    } else {
        OLED_->Printf(3, 0, "P:--- R:--- Y:---");
    }

    OLED_->RefreshGram();
}