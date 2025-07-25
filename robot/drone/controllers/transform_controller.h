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
#include "bsp_can.h"
#include "bsp_gpio.h"
#include "motor.h"
#include "system_state_manager.h"
#include "../common/control_modes.h"
#include "../common/task_config.h"

//==================================================================================================
// Transform Control Class
//==================================================================================================

class TransformController {
public:
    static constexpr uint32_t TASK_DELAY_MS = TRANSFORM_TASK_DELAY_MS;

    enum class TransformState {
        IDLE = 0,
        FORWARD_ROTATING,
        ACTUATOR_EXTENDING,
        BACKWARD_ROTATING,
        ACTUATOR_RETRACTING,
        COMPLETED
    };

    enum class ActuatorState {
        STOPPED = 0,
        EXTENDING,
        RETRACTING
    };

    TransformController() = default;

    void Init(bsp::CAN* can, SystemStateManager* state_manager);

    void Execute();

    control::Motor3508* GetTransformMotor() { return transform_motor_.get(); }

private:
    static TransformController* instance_;

    static constexpr uint32_t ACTUATOR_PULSE_WIDTH = 100;
    static constexpr uint32_t ACTUATOR_OPERATION_DELAY = 5000;
    static constexpr float TRANSFORM_MOTOR_SPEED = 2.0f;
    static constexpr float TRANSFORM_MOTOR_ACCELERATION = 10.0f;

    bsp::CAN* can_ = nullptr;
    SystemStateManager* state_manager_ = nullptr;

    std::unique_ptr<control::Motor3508> transform_motor_;
    std::unique_ptr<control::ServoMotor> transform_servo_;
    std::unique_ptr<bsp::GPIO> actuator_gpio_;

    TransformState current_state_ = TransformState::IDLE;
    ActuatorState actuator_state_ = ActuatorState::STOPPED;

    uint32_t last_pulse_time_ = 0;
    uint32_t operation_start_time_ = 0;
    bool actuator_pulsing_ = false;
    bool motor_jammed_ = false;
    int pulse_count_ = 0;

    enum class ActuatorTarget {
        NONE = 0,
        EXTENDING,
        RETRACTING
    } actuator_target_ = ActuatorTarget::NONE;

    TransformMode previous_mode_ = TransformMode::VEHICLE;

    static void JamCallback(control::ServoMotor* servo, const control::servo_jam_t data);

    void SendActuatorPulse();
    void UpdateActuatorPulse();
    void StartActuatorExtension();
    void StartActuatorRetraction();
    bool IsActuatorAtTarget();
    void UpdateActuatorAutoControl();
    void StartMotorForwardRotation();
    void StartMotorBackwardRotation();
    void StopTransformMotor();
    bool IsActuatorOperationComplete();
    void HandleTransformModeChange();
    void RunStateMachine();
};