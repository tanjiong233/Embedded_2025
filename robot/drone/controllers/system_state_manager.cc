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

#include "system_state_manager.h"
#include "main.h"

void SystemStateManager::Init(remote::SBUS* sbus_ptr) {
    sbus_ = sbus_ptr;
    motor_control_mutex_ = osMutexNew(nullptr);
    if (motor_control_mutex_ == nullptr) {
        // Mutex creation failed, system cannot work properly
        while(1) {
            HAL_Delay(100);
        }
    }
}

void SystemStateManager::UpdateControlModes() {
    current_control_mode_ = DetermineControlMode();
    current_transform_mode_ = DetermineTransformMode();

    if (current_control_mode_ != previous_control_mode_) {
        HandleModeSwitch();
        previous_control_mode_ = current_control_mode_;
    }

    if (current_transform_mode_ != previous_transform_mode_) {
        previous_transform_mode_ = current_transform_mode_;
    }
}

bool SystemStateManager::IsInEmergencyStop() const {
    return current_control_mode_ == ControlMode::EMERGENCY_STOP;
}

void SystemStateManager::SetPIDResetCallback(std::function<void()> callback) {
    pid_reset_callback_ = callback;
}

ControlMode SystemStateManager::DetermineControlMode() {
    if (!sbus_ || !sbus_->connection_flag_) {
        return ControlMode::EMERGENCY_STOP;
    }

    uint32_t current_time = HAL_GetTick();
    if (current_time - sbus_->timestamp > SBUS_TIMEOUT_MS) {
        return ControlMode::EMERGENCY_STOP;
    }

    int16_t mode_channel = sbus_->ch[5];

    if (mode_channel < MODE_LOWER_THRESHOLD) {
        return ControlMode::EMERGENCY_STOP;
    } else if (mode_channel > MODE_UPPER_THRESHOLD) {
        return ControlMode::COMPUTER_CONTROL;
    } else {
        return ControlMode::REMOTE_CONTROL;
    }
}

TransformMode SystemStateManager::DetermineTransformMode() {
    if (!sbus_ || !sbus_->connection_flag_) {
        return TransformMode::VEHICLE;
    }

    int16_t transform_channel = sbus_->ch[4];
    return (transform_channel > 0) ? TransformMode::FLIGHT : TransformMode::VEHICLE;
}

void SystemStateManager::HandleModeSwitch() {
    if (current_control_mode_ == ControlMode::EMERGENCY_STOP) {
        // Emergency stop mode: immediately stop all motors and reset PID
        if (pid_reset_callback_) {
            pid_reset_callback_();
        }
    }
}