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

#include <memory>
#include <array>
#include "bsp_can.h"
#include "motor.h"
#include "sbus.h"
#include "system_state_manager.h"
#include "../common/control_modes.h"
#include "../common/task_config.h"

//==================================================================================================
// Vehicle Control Class
//==================================================================================================

class VehicleController {
public:
    static constexpr uint32_t TASK_DELAY_MS = VEHICLE_TASK_DELAY_MS;

    VehicleController() = default;

    void Init(bsp::CAN* can, SystemStateManager* state_manager, remote::SBUS* sbus);

    void Execute();

    void EmergencyStop();

    // Allow computer control access to motors
    void SetComputerControl(float linear_vel, float angular_vel);

    control::Motor2006* GetLeftMotor() { return left_motor_.get(); }
    control::Motor2006* GetRightMotor() { return right_motor_.get(); }

private:
    static constexpr float MAX_VEHICLE_SPEED = 5.0f;
    static constexpr float TURN_SENSITIVITY = 0.5f;
    static constexpr int SBUS_MAX_VALUE = 660;
    static constexpr int SBUS_DEADZONE = 50;
    static constexpr std::array<float, 3> PID_PARAMS = {8.0f, 0.1f, 0.0f};

    bsp::CAN* can_ = nullptr;
    SystemStateManager* state_manager_ = nullptr;
    remote::SBUS* sbus_ = nullptr;

    std::unique_ptr<control::Motor2006> left_motor_;
    std::unique_ptr<control::Motor2006> right_motor_;
    std::unique_ptr<control::ConstrainedPID> left_pid_;
    std::unique_ptr<control::ConstrainedPID> right_pid_;

    float target_linear_speed_ = 0.0f;
    float target_angular_speed_ = 0.0f;
    float left_wheel_speed_ = 0.0f;
    float right_wheel_speed_ = 0.0f;

    static int16_t ApplyDeadzone(int16_t input, int16_t deadzone);
    static float NormalizeSBUS(int16_t input);

    void ProcessSBUSInputs();
    void CalculateDifferentialSpeeds();
    void ControlMotors();
    void StopMotors();
    void SendMotorCommands();
    void ResetPIDs();
};