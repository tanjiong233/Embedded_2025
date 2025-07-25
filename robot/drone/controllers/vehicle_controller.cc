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

#include "vehicle_controller.h"
#include "cmsis_os.h"

void VehicleController::Init(bsp::CAN* can, SystemStateManager* state_manager, remote::SBUS* sbus) {
    can_ = can;
    state_manager_ = state_manager;
    sbus_ = sbus;

    // Initialize motors
    left_motor_ = std::make_unique<control::Motor2006>(can_, 0x201);
    right_motor_ = std::make_unique<control::Motor2006>(can_, 0x202);

    // Initialize PID controllers
    left_pid_ = std::make_unique<control::ConstrainedPID>(
            PID_PARAMS[0], PID_PARAMS[1], PID_PARAMS[2], 3000.0f, 10000.0f);
    right_pid_ = std::make_unique<control::ConstrainedPID>(
            PID_PARAMS[0], PID_PARAMS[1], PID_PARAMS[2], 3000.0f, 10000.0f);

    // Set PID reset callback
    state_manager_->SetPIDResetCallback([this]() { ResetPIDs(); });
}

void VehicleController::Execute() {
    // Check if we should execute control
    if (state_manager_->GetCurrentTransformMode() != TransformMode::VEHICLE ||
        state_manager_->GetCurrentControlMode() != ControlMode::REMOTE_CONTROL) {
        osDelay(TASK_DELAY_MS);
        return;
    }

    // Check SBUS connection
    if (!sbus_ || !sbus_->connection_flag_) {
        StopMotors();
        osDelay(TASK_DELAY_MS);
        return;
    }

    // Read and process SBUS inputs
    ProcessSBUSInputs();

    // Calculate wheel speeds
    CalculateDifferentialSpeeds();

    // Control motors
    ControlMotors();

    osDelay(TASK_DELAY_MS);
}

void VehicleController::EmergencyStop() {
    if (osMutexAcquire(state_manager_->GetMotorControlMutex(), 100) == osOK) {
        if (left_motor_) left_motor_->SetOutput(0);
        if (right_motor_) right_motor_->SetOutput(0);

        SendMotorCommands();

        osMutexRelease(state_manager_->GetMotorControlMutex());
    }
}

void VehicleController::SetComputerControl(float linear_vel, float angular_vel) {
    static constexpr float WHEEL_RADIUS = 0.076f;
    static constexpr float WHEEL_BASE = 0.335f;
    static constexpr float MAX_WHEEL_SPEED = 10.0f;

    float left_wheel_vel = (linear_vel - angular_vel * WHEEL_BASE / 2.0f) / WHEEL_RADIUS;
    float right_wheel_vel = (linear_vel + angular_vel * WHEEL_BASE / 2.0f) / WHEEL_RADIUS;

    left_wheel_vel = clip<float>(left_wheel_vel, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
    right_wheel_vel = clip<float>(right_wheel_vel, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);

    if (left_motor_ && right_motor_ && left_pid_ && right_pid_) {
        float left_error = left_wheel_vel - left_motor_->GetOmega();
        float right_error = right_wheel_vel - right_motor_->GetOmega();

        int16_t left_output = left_pid_->ComputeConstrainedOutput(left_error);
        int16_t right_output = right_pid_->ComputeConstrainedOutput(right_error);

        if (osMutexAcquire(state_manager_->GetMotorControlMutex(), 10) == osOK) {
            left_motor_->SetOutput(left_output);
            right_motor_->SetOutput(right_output);
            SendMotorCommands();
            osMutexRelease(state_manager_->GetMotorControlMutex());
        }
    }
}

int16_t VehicleController::ApplyDeadzone(int16_t input, int16_t deadzone) {
    return (abs(input) < deadzone) ? 0 : input;
}

float VehicleController::NormalizeSBUS(int16_t input) {
    return static_cast<float>(input) / SBUS_MAX_VALUE;
}

void VehicleController::ProcessSBUSInputs() {
    int16_t throttle_raw = ApplyDeadzone(sbus_->ch[2], SBUS_DEADZONE);
    int16_t steering_raw = ApplyDeadzone(sbus_->ch[3], SBUS_DEADZONE);

    float throttle = NormalizeSBUS(throttle_raw);
    float steering = NormalizeSBUS(steering_raw);

    target_linear_speed_ = throttle * MAX_VEHICLE_SPEED;
    target_angular_speed_ = steering * MAX_VEHICLE_SPEED * TURN_SENSITIVITY;
}

void VehicleController::CalculateDifferentialSpeeds() {
    left_wheel_speed_ = target_linear_speed_ - target_angular_speed_;
    right_wheel_speed_ = target_linear_speed_ + target_angular_speed_;

    left_wheel_speed_ = clip<float>(left_wheel_speed_, -MAX_VEHICLE_SPEED, MAX_VEHICLE_SPEED);
    right_wheel_speed_ = clip<float>(right_wheel_speed_, -MAX_VEHICLE_SPEED, MAX_VEHICLE_SPEED);
}

void VehicleController::ControlMotors() {
    if (!left_motor_ || !right_motor_ || !left_pid_ || !right_pid_) return;

    float left_error = left_wheel_speed_ - left_motor_->GetOmega();
    float right_error = right_wheel_speed_ - right_motor_->GetOmega();

    int16_t left_output = left_pid_->ComputeConstrainedOutput(left_error);
    int16_t right_output = right_pid_->ComputeConstrainedOutput(right_error);

    if (osMutexAcquire(state_manager_->GetMotorControlMutex(), 10) == osOK) {
        left_motor_->SetOutput(left_output);
        right_motor_->SetOutput(right_output);
        SendMotorCommands();
        osMutexRelease(state_manager_->GetMotorControlMutex());
    }
}

void VehicleController::StopMotors() {
    if (left_motor_) left_motor_->SetOutput(0);
    if (right_motor_) right_motor_->SetOutput(0);
    SendMotorCommands();
}

void VehicleController::SendMotorCommands() {
    if (left_motor_ && right_motor_ &&
        (left_motor_->connection_flag_ || right_motor_->connection_flag_)) {
        control::MotorCANBase* motors[2] = {left_motor_.get(), right_motor_.get()};
        control::MotorCANBase::TransmitOutput(motors, 2);
    }
}

void VehicleController::ResetPIDs() {
    StopMotors();
    if (left_pid_) left_pid_->Reset();
    if (right_pid_) right_pid_->Reset();
}