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

#include <functional>
#include "cmsis_os.h"
#include "sbus.h"
#include "../common/control_modes.h"

//==================================================================================================
// System State Manager Class
//==================================================================================================

class SystemStateManager {
public:
    SystemStateManager() = default;

    // Delete copy constructor and assignment operator
    SystemStateManager(const SystemStateManager&) = delete;
    SystemStateManager& operator=(const SystemStateManager&) = delete;

    void Init(remote::SBUS* sbus_ptr);

    ControlMode GetCurrentControlMode() const { return current_control_mode_; }
    TransformMode GetCurrentTransformMode() const { return current_transform_mode_; }
    osMutexId_t GetMotorControlMutex() const { return motor_control_mutex_; }

    /**
     * @brief Update control modes based on SBUS input
     */
    void UpdateControlModes();

    bool IsInEmergencyStop() const;

    void SetPIDResetCallback(std::function<void()> callback);

private:
    static constexpr int16_t MODE_UPPER_THRESHOLD = 400;
    static constexpr int16_t MODE_LOWER_THRESHOLD = -400;
    static constexpr uint32_t SBUS_TIMEOUT_MS = 500;

    remote::SBUS* sbus_ = nullptr;
    osMutexId_t motor_control_mutex_ = nullptr;

    ControlMode current_control_mode_ = ControlMode::EMERGENCY_STOP;
    ControlMode previous_control_mode_ = ControlMode::EMERGENCY_STOP;
    TransformMode current_transform_mode_ = TransformMode::VEHICLE;
    TransformMode previous_transform_mode_ = TransformMode::VEHICLE;

    std::function<void()> pid_reset_callback_;

    ControlMode DetermineControlMode();
    TransformMode DetermineTransformMode();
    void HandleModeSwitch();
};