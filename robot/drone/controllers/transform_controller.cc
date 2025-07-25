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

#include "transform_controller.h"
#include "cmsis_os.h"
#include "main.h"

TransformController* TransformController::instance_ = nullptr;

void TransformController::Init(bsp::CAN* can, SystemStateManager* state_manager) {
    can_ = can;
    state_manager_ = state_manager;
    instance_ = this;

    // Initialize transform motor
    transform_motor_ = std::make_unique<control::Motor3508>(can_, 0x203);

    // Initialize servo motor
    control::servo_t servo_data;
    servo_data.motor = transform_motor_.get();
    servo_data.max_speed = TRANSFORM_MOTOR_SPEED;
    servo_data.max_acceleration = TRANSFORM_MOTOR_ACCELERATION;
    servo_data.transmission_ratio = M3508P19_RATIO;
    servo_data.omega_pid_param = new float[3]{30, 0.2, 50};
    servo_data.max_iout = 1000;
    servo_data.max_out = 8000;

    transform_servo_ = std::make_unique<control::ServoMotor>(servo_data);
    transform_servo_->RegisterJamCallback(JamCallback, 0.6);

    // Initialize actuator GPIO
    actuator_gpio_ = std::make_unique<bsp::GPIO>(GPIOI, GPIO_PIN_7);
    actuator_gpio_->High();
}

void TransformController::Execute() {
    UpdateActuatorPulse();
    HandleTransformModeChange();
    RunStateMachine();

    if (transform_servo_) {
        transform_servo_->CalcOutput();

        if (transform_motor_ && transform_motor_->connection_flag_) {
            control::MotorCANBase* motors[1] = {transform_motor_.get()};
            control::MotorCANBase::TransmitOutput(motors, 1);
        }
    }

    osDelay(TASK_DELAY_MS);
}

void TransformController::JamCallback(control::ServoMotor* servo, const control::servo_jam_t data) {
    UNUSED(servo);
    UNUSED(data);
    if (instance_) {
        instance_->motor_jammed_ = true;
    }
}

void TransformController::SendActuatorPulse() {
    if (!actuator_gpio_ || actuator_pulsing_) return;

    actuator_pulsing_ = true;
    last_pulse_time_ = HAL_GetTick();
    actuator_gpio_->Low();
    pulse_count_++;

    switch (pulse_count_ % 4) {
        case 1: actuator_state_ = ActuatorState::EXTENDING; break;
        case 2: actuator_state_ = ActuatorState::STOPPED; break;
        case 3: actuator_state_ = ActuatorState::RETRACTING; break;
        case 0: actuator_state_ = ActuatorState::STOPPED; break;
    }
}

void TransformController::UpdateActuatorPulse() {
    if (!actuator_pulsing_ || !actuator_gpio_) return;

    uint32_t current_time = HAL_GetTick();
    if (current_time - last_pulse_time_ >= ACTUATOR_PULSE_WIDTH) {
        actuator_gpio_->High();
        actuator_pulsing_ = false;
    }
}

void TransformController::StartActuatorExtension() {
    actuator_target_ = ActuatorTarget::EXTENDING;
    operation_start_time_ = HAL_GetTick();

    if (actuator_state_ != ActuatorState::EXTENDING &&
        actuator_state_ != ActuatorState::STOPPED ||
        (actuator_state_ == ActuatorState::STOPPED && pulse_count_ % 4 != 2)) {
        SendActuatorPulse();
    }
}

void TransformController::StartActuatorRetraction() {
    actuator_target_ = ActuatorTarget::RETRACTING;
    operation_start_time_ = HAL_GetTick();

    if (actuator_state_ != ActuatorState::RETRACTING &&
        actuator_state_ != ActuatorState::STOPPED ||
        (actuator_state_ == ActuatorState::STOPPED && pulse_count_ % 4 != 0)) {
        SendActuatorPulse();
    }
}

bool TransformController::IsActuatorAtTarget() {
    switch (actuator_target_) {
        case ActuatorTarget::EXTENDING:
            return (actuator_state_ == ActuatorState::EXTENDING ||
                    (actuator_state_ == ActuatorState::STOPPED && pulse_count_ % 4 == 2));
        case ActuatorTarget::RETRACTING:
            return (actuator_state_ == ActuatorState::RETRACTING ||
                    (actuator_state_ == ActuatorState::STOPPED && pulse_count_ % 4 == 0));
        default:
            return true;
    }
}

void TransformController::UpdateActuatorAutoControl() {
    if (IsActuatorAtTarget() || actuator_pulsing_) return;

    if (actuator_target_ != ActuatorTarget::NONE) {
        SendActuatorPulse();
    }
}

void TransformController::StartMotorForwardRotation() {
    if (!transform_servo_) return;

    motor_jammed_ = false;
    float current_angle = transform_servo_->GetTheta();
    float target_angle = current_angle + 2 * PI;
    transform_servo_->SetTarget(target_angle, true);
}

void TransformController::StartMotorBackwardRotation() {
    if (!transform_servo_) return;

    motor_jammed_ = false;
    float current_angle = transform_servo_->GetTheta();
    float target_angle = current_angle - 2 * PI;
    transform_servo_->SetTarget(target_angle, true);
}

void TransformController::StopTransformMotor() {
    if (!transform_servo_) return;

    float current_angle = transform_servo_->GetTheta();
    transform_servo_->SetTarget(current_angle, true);
}

bool TransformController::IsActuatorOperationComplete() {
    uint32_t current_time = HAL_GetTick();
    return (current_time - operation_start_time_ >= ACTUATOR_OPERATION_DELAY);
}

void TransformController::HandleTransformModeChange() {
    auto current_mode = state_manager_->GetCurrentTransformMode();

    if (current_mode != previous_mode_) {
        if (previous_mode_ == TransformMode::VEHICLE && current_mode == TransformMode::FLIGHT) {
            current_state_ = TransformState::FORWARD_ROTATING;
            StartMotorForwardRotation();
        } else if (previous_mode_ == TransformMode::FLIGHT && current_mode == TransformMode::VEHICLE) {
            current_state_ = TransformState::ACTUATOR_RETRACTING;
            StartActuatorRetraction();
        }

        previous_mode_ = current_mode;
    }
}

void TransformController::RunStateMachine() {
    switch (current_state_) {
        case TransformState::IDLE:
            break;

        case TransformState::FORWARD_ROTATING:
            if (motor_jammed_) {
                StopTransformMotor();
                current_state_ = TransformState::ACTUATOR_EXTENDING;
                StartActuatorExtension();
            } else if (transform_servo_ && transform_servo_->Holding()) {
                float current_angle = transform_servo_->GetTheta();
                float target_angle = current_angle + 2 * PI;
                transform_servo_->SetTarget(target_angle, true);
            }
            break;

        case TransformState::ACTUATOR_EXTENDING:
            UpdateActuatorAutoControl();
            if (IsActuatorOperationComplete() && IsActuatorAtTarget()) {
                actuator_target_ = ActuatorTarget::NONE;
                current_state_ = TransformState::COMPLETED;
            }
            break;

        case TransformState::BACKWARD_ROTATING:
            if (motor_jammed_) {
                StopTransformMotor();
                current_state_ = TransformState::COMPLETED;
            } else if (transform_servo_ && transform_servo_->Holding()) {
                float current_angle = transform_servo_->GetTheta();
                float target_angle = current_angle - 2 * PI;
                transform_servo_->SetTarget(target_angle, true);
            }
            break;

        case TransformState::ACTUATOR_RETRACTING:
            UpdateActuatorAutoControl();
            if (IsActuatorOperationComplete() && IsActuatorAtTarget()) {
                actuator_target_ = ActuatorTarget::NONE;
                current_state_ = TransformState::BACKWARD_ROTATING;
                StartMotorBackwardRotation();
            }
            break;

        case TransformState::COMPLETED:
            actuator_target_ = ActuatorTarget::NONE;
            current_state_ = TransformState::IDLE;
            break;
    }
}