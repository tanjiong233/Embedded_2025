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
#include "bsp_pwm.h"
#include "bsp_gpio.h"

//==================================================================================================
// Global Variables Definition
//==================================================================================================

static remote::SBUS* sbus;        ///< SBUS remote controller object
static bsp::CAN* can1 = nullptr;  ///< CAN bus object

static osMutexId_t motor_control_mutex;  ///< Motor control mutex

/// Task delay time definitions (milliseconds) - adjusted according to README requirements
static const int VEHICLE_TASK_DELAY = 2;        ///< Vehicle control task delay
static const int TRANSFORM_TASK_DELAY = 10;     ///< Transform control task delay
static const int SUCTION_TASK_DELAY = 20;       ///< Suction control task delay
static const int IMU_TASK_DELAY = 1;            ///< IMU task delay
static const int SELFTEST_TASK_DELAY = 100;     ///< Self-test task delay
static const int LED_TASK_DELAY = 1;            ///< LED task delay
static const int COMMUNICATION_TASK_DELAY = 50; ///< Communication task delay
static const int DEFAULT_TASK_DELAY = 100;      ///< Default task delay

//==================================================================================================
// Control Mode Definitions
//==================================================================================================

/**
 * @brief Control mode enumeration
 */
typedef enum {
    EMERGENCY_STOP_MODE = 0,    ///< Emergency stop mode (channel 6 down position)
    REMOTE_CONTROL_MODE = 1,    ///< Remote control mode (channel 6 middle position)
    COMPUTER_CONTROL_MODE = 2   ///< Computer control mode (channel 6 up position)
} control_mode_t;

/**
 * @brief Transform mode enumeration
 */
typedef enum {
    VEHICLE_MODE = 0,           ///< Vehicle mode (channel 5 down position)
    FLIGHT_MODE = 1             ///< Flight mode (channel 5 up position)
} transform_mode_t;

/// System state variables
static control_mode_t current_control_mode = EMERGENCY_STOP_MODE;
static control_mode_t previous_control_mode = EMERGENCY_STOP_MODE;
static transform_mode_t current_transform_mode = VEHICLE_MODE;
static transform_mode_t previous_transform_mode = VEHICLE_MODE;

/// Control mode threshold definitions
static const int16_t MODE_UPPER_THRESHOLD = 400;   ///< Upper position mode threshold
static const int16_t MODE_LOWER_THRESHOLD = -400;  ///< Lower position mode threshold
static const uint32_t SBUS_TIMEOUT_MS = 500;       ///< SBUS timeout in milliseconds

//==================================================================================================
// Vehicle Control Module
//==================================================================================================

/// Vehicle control task attributes definition
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

/// Vehicle motor objects (left wheel: CAN ID 1, right wheel: CAN ID 2)
static control::MotorCANBase* left_wheel_motor = nullptr;   ///< Left wheel motor
static control::MotorCANBase* right_wheel_motor = nullptr;  ///< Right wheel motor

/// Vehicle control parameters
static const float MAX_VEHICLE_SPEED = 5.0f;  ///< Maximum vehicle speed (rad/s)
static const float TURN_SENSITIVITY = 0.5f;   ///< Turn sensitivity coefficient
static const int SBUS_MAX_VALUE = 660;        ///< SBUS maximum channel value
static const int SBUS_DEADZONE = 50;          ///< SBUS deadzone range

/// Motor PID control parameters
static float left_motor_pid_params[3] = {8.0f, 0.1f, 0.0f};   ///< Left wheel motor PID parameters
static float right_motor_pid_params[3] = {8.0f, 0.1f, 0.0f};  ///< Right wheel motor PID parameters

/// PID controller objects
static control::ConstrainedPID* left_motor_pid = nullptr;   ///< Left wheel motor PID controller
static control::ConstrainedPID* right_motor_pid = nullptr;  ///< Right wheel motor PID controller

/// Vehicle motion state variables
static float target_linear_speed = 0.0f;    ///< Target linear velocity
static float target_angular_speed = 0.0f;   ///< Target angular velocity
static float left_wheel_speed = 0.0f;       ///< Left wheel target speed
static float right_wheel_speed = 0.0f;      ///< Right wheel target speed

/**
 * @brief Apply deadzone processing to SBUS input
 * @param input    SBUS input value
 * @param deadzone Deadzone threshold
 * @return         Processed input value
 */
int16_t ApplyDeadzone(int16_t input, int16_t deadzone) {
    return (abs(input) < deadzone) ? 0 : input;
}

/**
 * @brief Normalize SBUS input to [-1, 1] range
 * @param input SBUS input value
 * @return      Normalized value in [-1, 1] range
 */
float NormalizeSBUS(int16_t input) {
    return (float)input / SBUS_MAX_VALUE;
}

/**
 * @brief Calculate differential drive left and right wheel speeds
 * @param linear_speed  Target linear speed
 * @param angular_speed Target angular speed
 * @param left_speed    Output left wheel speed
 * @param right_speed   Output right wheel speed
 */
void CalculateDifferentialSpeeds(float linear_speed, float angular_speed,
                                 float* left_speed, float* right_speed) {
    // Differential drive kinematics: simplified version
    *left_speed = linear_speed - angular_speed;
    *right_speed = linear_speed + angular_speed;

    // Limit speeds to maximum value range
    *left_speed = clip<float>(*left_speed, -MAX_VEHICLE_SPEED, MAX_VEHICLE_SPEED);
    *right_speed = clip<float>(*right_speed, -MAX_VEHICLE_SPEED, MAX_VEHICLE_SPEED);
}

/**
 * @brief Vehicle control task
 * @param arg Task argument (unused)
 */
extern "C" void vehicleTask(void* arg) {
    UNUSED(arg);

    while (true) {
        // Execute vehicle control only in vehicle mode and remote control mode
        if (current_transform_mode != VEHICLE_MODE ||
            current_control_mode != REMOTE_CONTROL_MODE) {
            // In non-remote mode, don't send any motor commands to avoid conflicts with other control modes
            osDelay(VEHICLE_TASK_DELAY);
            continue;
        }

        // Check SBUS connection status
        if (!sbus || !sbus->connection_flag_) {
            // SBUS not connected, stop motors
            if (left_wheel_motor) left_wheel_motor->SetOutput(0);
            if (right_wheel_motor) right_wheel_motor->SetOutput(0);

            if (left_wheel_motor && right_wheel_motor &&
                (left_wheel_motor->connection_flag_ || right_wheel_motor->connection_flag_)) {
                control::MotorCANBase* motors[2] = {left_wheel_motor, right_wheel_motor};
                control::MotorCANBase::TransmitOutput(motors, 2);
            }
            osDelay(VEHICLE_TASK_DELAY);
            continue;
        }

        // Read SBUS channels and apply deadzone processing
        // According to README: channel 3 for throttle, channel 4 for steering
        int16_t throttle_raw = ApplyDeadzone(sbus->ch[2], SBUS_DEADZONE);    // Channel 3 (ch[2])
        int16_t steering_raw = ApplyDeadzone(sbus->ch[3], SBUS_DEADZONE);    // Channel 4 (ch[3])

        // Normalize inputs to [-1, 1] range
        float throttle = NormalizeSBUS(throttle_raw);
        float steering = NormalizeSBUS(steering_raw);

        // Calculate target speeds
        target_linear_speed = throttle * MAX_VEHICLE_SPEED;
        target_angular_speed = steering * MAX_VEHICLE_SPEED * TURN_SENSITIVITY;

        // Calculate differential drive left and right wheel speeds
        CalculateDifferentialSpeeds(target_linear_speed, target_angular_speed,
                                    &left_wheel_speed, &right_wheel_speed);

        // Calculate PID outputs
        if (left_wheel_motor && right_wheel_motor && left_motor_pid && right_motor_pid) {
            float left_error = left_wheel_speed - left_wheel_motor->GetOmega();
            float right_error = right_wheel_speed - right_wheel_motor->GetOmega();

            int16_t left_output = left_motor_pid->ComputeConstrainedOutput(left_error);
            int16_t right_output = right_motor_pid->ComputeConstrainedOutput(right_error);

            // Use mutex to protect motor control
            if (osMutexAcquire(motor_control_mutex, 10) == osOK) {  // 10ms timeout
                // Set motor outputs
                left_wheel_motor->SetOutput(left_output);
                right_wheel_motor->SetOutput(right_output);

                // Send CAN messages
                if (left_wheel_motor->connection_flag_ || right_wheel_motor->connection_flag_) {
                    control::MotorCANBase* motors[2] = {left_wheel_motor, right_wheel_motor};
                    control::MotorCANBase::TransmitOutput(motors, 2);
                }

                osMutexRelease(motor_control_mutex);
            }
        }

        osDelay(VEHICLE_TASK_DELAY);
    }
}

//==================================================================================================
// Transform Control Module
//==================================================================================================

/// Transform motor objects (3508: CAN ID 3)
static control::MotorCANBase* transform_motor = nullptr;  ///< Transform motor
static control::ServoMotor* transform_servo = nullptr;    ///< Transform servo motor

static bsp::GPIO* actuator_gpio = nullptr;  ///< Actuator pulse signal GPIO (PI7)

/// Transform control task attributes definition
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

/**
 * @brief Transform state enumeration
 */
typedef enum {
    TRANSFORM_IDLE = 0,                    ///< Idle state
    TRANSFORM_FORWARD_ROTATING,            ///< Forward rotation in progress (vehicle→flight)
    TRANSFORM_ACTUATOR_EXTENDING,          ///< Actuator extension in progress
    TRANSFORM_BACKWARD_ROTATING,           ///< Backward rotation in progress (flight→vehicle)
    TRANSFORM_ACTUATOR_RETRACTING,         ///< Actuator retraction in progress
    TRANSFORM_COMPLETED                    ///< Transform completed
} transform_state_t;

/**
 * @brief Actuator state enumeration (push-stop-pull-stop cycle)
 */
typedef enum {
    ACTUATOR_STOPPED = 0,                  ///< Stopped state
    ACTUATOR_EXTENDING,                    ///< Extending state
    ACTUATOR_RETRACTING                    ///< Retracting state
} actuator_state_t;

/**
 * @brief Actuator target state enumeration
 */
typedef enum {
    ACTUATOR_TARGET_NONE = 0,        ///< No target
    ACTUATOR_TARGET_EXTENDING,       ///< Target: extending state
    ACTUATOR_TARGET_RETRACTING      ///< Target: retracting state
} actuator_target_t;

/// Transform control parameters
static const uint32_t ACTUATOR_PULSE_WIDTH = 100;        ///< Pulse width 100ms
static const uint32_t ACTUATOR_OPERATION_DELAY = 5000;   ///< Actuator operation delay 5 seconds
static const float TRANSFORM_MOTOR_SPEED = 2.0f;         ///< Transform motor speed (rad/s)
static const float TRANSFORM_MOTOR_ACCELERATION = 10.0f; ///< Transform motor acceleration (rad/s^2)

/// Transform control state variables
static transform_state_t current_transform_state = TRANSFORM_IDLE;
static actuator_state_t current_actuator_state = ACTUATOR_STOPPED;
static actuator_target_t actuator_target = ACTUATOR_TARGET_NONE;
static uint32_t last_actuator_pulse_time = 0;           ///< Last pulse time
static uint32_t transform_operation_start_time = 0;     ///< Transform operation start time
static bool actuator_pulsing = false;                   ///< Whether pulse is being sent
static bool transform_motor_jammed = false;             ///< Motor jam flag
static int actuator_pulse_count = 0;                    ///< Actuator pulse counter

/**
 * @brief Transform motor jam callback function
 * @param servo Servo motor instance
 * @param data  Jam data
 */
void transform_jam_callback(control::ServoMotor* servo, const control::servo_jam_t data) {
    UNUSED(servo);
    UNUSED(data);
    transform_motor_jammed = true;
}

/**
 * @brief Send actuator pulse signal
 * @note Actuator logic: active low pulse, low level duration at least 100ms
 *       Each signal changes state: push-stop-pull-stop-push cycle
 */
void SendActuatorPulse() {
    if (!actuator_gpio || actuator_pulsing) return;

    actuator_pulsing = true;
    last_actuator_pulse_time = HAL_GetTick();
    actuator_gpio->Low();  // Start active low pulse
    actuator_pulse_count++;

    // Update actuator state (4 state cycle: push-stop-pull-stop)
    switch (actuator_pulse_count % 4) {
        case 1:
            current_actuator_state = ACTUATOR_EXTENDING;
            break;
        case 2:
            current_actuator_state = ACTUATOR_STOPPED;
            break;
        case 3:
            current_actuator_state = ACTUATOR_RETRACTING;
            break;
        case 0:
            current_actuator_state = ACTUATOR_STOPPED;
            break;
    }
}

/**
 * @brief Update actuator pulse state
 */
void UpdateActuatorPulse() {
    if (!actuator_pulsing || !actuator_gpio) return;

    uint32_t current_time = HAL_GetTick();
    if (current_time - last_actuator_pulse_time >= ACTUATOR_PULSE_WIDTH) {
        actuator_gpio->High();  // End pulse
        actuator_pulsing = false;
    }
}

/**
 * @brief Start actuator retraction operation
 */
void StartActuatorRetraction() {
    actuator_target = ACTUATOR_TARGET_RETRACTING;
    transform_operation_start_time = HAL_GetTick();

    // Send first pulse if needed based on current state
    if (current_actuator_state != ACTUATOR_RETRACTING &&
        current_actuator_state != ACTUATOR_STOPPED ||
        (current_actuator_state == ACTUATOR_STOPPED && actuator_pulse_count % 4 != 0)) {
        SendActuatorPulse();
    }

}

/**
 * @brief Check if actuator has reached target state
 * @return true if target state is reached, false otherwise
 */
bool IsActuatorAtTarget() {
    switch (actuator_target) {
        case ACTUATOR_TARGET_EXTENDING:
            return (current_actuator_state == ACTUATOR_EXTENDING ||
                    (current_actuator_state == ACTUATOR_STOPPED && actuator_pulse_count % 4 == 2));
        case ACTUATOR_TARGET_RETRACTING:
            return (current_actuator_state == ACTUATOR_RETRACTING ||
                    (current_actuator_state == ACTUATOR_STOPPED && actuator_pulse_count % 4 == 0));
        default:
            return true;
    }
}

/**
 * @brief Update actuator automatic control
 */
void UpdateActuatorAutoControl() {
    // If target state is reached, no need to continue
    if (IsActuatorAtTarget()) {
        return;
    }

    // If currently sending pulse, wait for completion
    if (actuator_pulsing) {
        return;
    }

    // If there is a target state but not reached, continue sending pulses
    if (actuator_target != ACTUATOR_TARGET_NONE) {
        SendActuatorPulse();
    }
}

/**
 * @brief Start transform motor forward rotation
 */
void StartMotorForwardRotation() {
    if (!transform_servo) return;

    transform_motor_jammed = false;
    float current_angle = transform_servo->GetTheta();
    float target_angle = current_angle + 2 * PI;  // Forward rotation one turn as target
    transform_servo->SetTarget(target_angle, true);
}

/**
 * @brief Start transform motor backward rotation
 */
void StartMotorBackwardRotation() {
    if (!transform_servo) return;

    transform_motor_jammed = false;
    float current_angle = transform_servo->GetTheta();
    float target_angle = current_angle - 2 * PI;  // Backward rotation one turn as target
    transform_servo->SetTarget(target_angle, true);
}

/**
 * @brief Stop transform motor
 */
void StopTransformMotor() {
    if (!transform_servo) return;

    float current_angle = transform_servo->GetTheta();
    transform_servo->SetTarget(current_angle, true);  // Set current position as target
}

/**
 * @brief Check if actuator operation is complete
 * @return true if operation is complete, false otherwise
 */
bool IsActuatorOperationComplete() {
    uint32_t current_time = HAL_GetTick();
    return (current_time - transform_operation_start_time >= ACTUATOR_OPERATION_DELAY);
}

/**
 * @brief Handle transform mode change
 */
void HandleTransformModeChange() {
    // Check if transform mode has changed
    if (current_transform_mode != previous_transform_mode) {

        if (previous_transform_mode == VEHICLE_MODE && current_transform_mode == FLIGHT_MODE) {
            // Vehicle mode → Flight mode: start forward motor rotation
            current_transform_state = TRANSFORM_FORWARD_ROTATING;
            StartMotorForwardRotation();
        } else if (previous_transform_mode == FLIGHT_MODE && current_transform_mode == VEHICLE_MODE) {
            // Flight mode → Vehicle mode: start actuator retraction
            current_transform_state = TRANSFORM_ACTUATOR_RETRACTING;
            StartActuatorRetraction();
        }

        previous_transform_mode = current_transform_mode;
    }
}

/**
 * @brief Start actuator extension operation
 */
void StartActuatorExtension() {
    actuator_target = ACTUATOR_TARGET_EXTENDING;
    transform_operation_start_time = HAL_GetTick();

    // Send first pulse if needed based on current state
    if (current_actuator_state != ACTUATOR_EXTENDING &&
        current_actuator_state != ACTUATOR_STOPPED ||
        (current_actuator_state == ACTUATOR_STOPPED && actuator_pulse_count % 4 != 2)) {
        SendActuatorPulse();
    }
}

/**
 * @brief Transform control state machine
 */
void TransformStateMachine() {
    uint32_t current_time = HAL_GetTick();

    switch (current_transform_state) {
        case TRANSFORM_IDLE:
            // Idle state, waiting for mode change
            break;

        case TRANSFORM_FORWARD_ROTATING:
            // Forward rotation in progress, waiting for jam
            if (transform_motor_jammed) {
                // Jam detected, stop motor and start actuator extension
                StopTransformMotor();
                current_transform_state = TRANSFORM_ACTUATOR_EXTENDING;
                StartActuatorExtension();
            } else if (transform_servo && transform_servo->Holding()) {
                // If motor reached target position but not jammed, continue rotation
                float current_angle = transform_servo->GetTheta();
                float target_angle = current_angle + 2 * PI;
                transform_servo->SetTarget(target_angle, true);
            }
            break;

        case TRANSFORM_ACTUATOR_EXTENDING:
            // Actuator extension in progress
            UpdateActuatorAutoControl();  // Automatically handle multi-step pulses

            if (IsActuatorOperationComplete()) {
                // Check if really reached extended state
                if (IsActuatorAtTarget()) {
                    actuator_target = ACTUATOR_TARGET_NONE;  // Clear target
                    current_transform_state = TRANSFORM_COMPLETED;
                }
            }
            break;

        case TRANSFORM_BACKWARD_ROTATING:
            // Backward rotation in progress, waiting for jam
            if (transform_motor_jammed) {
                // Jam detected, stop motor
                StopTransformMotor();
                current_transform_state = TRANSFORM_COMPLETED;
            } else if (transform_servo && transform_servo->Holding()) {
                // If motor reached target position but not jammed, continue rotation
                float current_angle = transform_servo->GetTheta();
                float target_angle = current_angle - 2 * PI;
                transform_servo->SetTarget(target_angle, true);
            }
            break;

        case TRANSFORM_ACTUATOR_RETRACTING:
            // Actuator retraction in progress
            UpdateActuatorAutoControl();  // Automatically handle multi-step pulses

            if (IsActuatorOperationComplete()) {
                // Check if really reached retracted state
                if (IsActuatorAtTarget()) {
                    actuator_target = ACTUATOR_TARGET_NONE;  // Clear target
                    current_transform_state = TRANSFORM_BACKWARD_ROTATING;
                    StartMotorBackwardRotation();
                }
            }
            break;

        case TRANSFORM_COMPLETED:
            // Transform completed, return to idle state
            actuator_target = ACTUATOR_TARGET_NONE;  // Ensure target is cleared
            current_transform_state = TRANSFORM_IDLE;
            break;
    }
}

/**
 * @brief Transform control task
 * @param arg Task argument (unused)
 */
extern "C" void transformTask(void* arg) {
    UNUSED(arg);

    // Wait for system initialization completion
    osDelay(1000);

    while (true) {
        // Update actuator pulse state
        UpdateActuatorPulse();

        // Handle transform mode change
        HandleTransformModeChange();

        // Run transform control state machine
        TransformStateMachine();

        // Calculate transform motor output
        if (transform_servo) {
            transform_servo->CalcOutput();

            // Send motor control commands
            if (transform_motor && transform_motor->connection_flag_) {
                control::MotorCANBase* motors[1] = {transform_motor};
                control::MotorCANBase::TransmitOutput(motors, 1);
            }
        }

        osDelay(TRANSFORM_TASK_DELAY);
    }
}

//==================================================================================================
// Suction Control Module
//==================================================================================================

static bsp::PWM* suction_fan = nullptr;  ///< Suction fan PWM controller

/// Suction control parameters
static const float MIN_FAN_PULSE_WIDTH = 1000.0f;   ///< Minimum pulse width (microseconds) - fan off
static const float MAX_FAN_PULSE_WIDTH = 2000.0f;   ///< Maximum pulse width (microseconds) - maximum speed
static const int SBUS_SUCTION_DEADZONE = 30;        ///< Suction channel deadzone
static const uint32_t FAN_PWM_FREQUENCY = 1500;     ///< Fan PWM frequency (Hz)
static const uint32_t FAN_TIMER_CLOCK = 1000000;    ///< Timer clock frequency (1MHz)

/// Suction control task attributes definition
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

/**
 * @brief Map SBUS input to PWM pulse width
 * @param sbus_input SBUS input value
 * @return           Mapped pulse width
 */
float MapSuctionToPulseWidth(int16_t sbus_input) {
    // Only use positive values to control fan speed (negative values = fan off)
    if (sbus_input <= 0) {
        return MIN_FAN_PULSE_WIDTH;
    }

    // Normalize to 0-1 range
    float normalized_input = (float)sbus_input / SBUS_MAX_VALUE;
    if (normalized_input > 1.0f) normalized_input = 1.0f;

    // Map to pulse width range
    return MIN_FAN_PULSE_WIDTH + (normalized_input * (MAX_FAN_PULSE_WIDTH - MIN_FAN_PULSE_WIDTH));
}

/**
 * @brief Suction control task
 * @param arg Task argument (unused)
 */
extern "C" void suctionTask(void* arg) {
    UNUSED(arg);

    if (!suction_fan) {
        osDelay(SUCTION_TASK_DELAY);
        return;
    }

    // Start PWM output
    suction_fan->Start();
    suction_fan->SetPulseWidth(MIN_FAN_PULSE_WIDTH);  // Initialize with fan off

    while (true) {
        // Execute suction control only in remote control mode
        if (current_control_mode != REMOTE_CONTROL_MODE) {
            suction_fan->SetPulseWidth(MIN_FAN_PULSE_WIDTH);
            osDelay(SUCTION_TASK_DELAY);
            continue;
        }

        // Check SBUS connection status
        if (!sbus || !sbus->connection_flag_) {
            suction_fan->SetPulseWidth(MIN_FAN_PULSE_WIDTH);
            osDelay(SUCTION_TASK_DELAY);
            continue;
        }

        // Read SBUS channel 2 (ch[1]) for suction control and apply deadzone
        int16_t suction_raw = ApplyDeadzone(sbus->ch[1], SBUS_SUCTION_DEADZONE);

        // Map SBUS input to PWM pulse width
        float target_pulse_width = MapSuctionToPulseWidth(suction_raw);

        // Set fan speed
        suction_fan->SetPulseWidth(target_pulse_width);

        osDelay(SUCTION_TASK_DELAY);
    }
}

//==================================================================================================
// IMU Module
//==================================================================================================

#define IMU_RX_SIGNAL (1 << 1)

/// IMU task attributes definition
const osThreadAttr_t imuTaskAttribute = {
        .name = "imuTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};
osThreadId_t imuTaskHandle;

/**
 * @brief Custom IMU class
 */
class IMU : public bsp::IMU_typeC {
public:
    using bsp::IMU_typeC::IMU_typeC;

protected:
    void RxCompleteCallback() final {
        osThreadFlagsSet(imuTaskHandle, IMU_RX_SIGNAL);
    }
};

static IMU* imu = nullptr;

/**
 * @brief IMU task
 * @param arg Task argument (unused)
 */
extern "C" void imuTask(void* arg) {
    UNUSED(arg);

    if (imu) {
        imu->Calibrate();  // IMU calibration
    }

    while (true) {
        uint32_t flags = osThreadFlagsWait(IMU_RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & IMU_RX_SIGNAL && imu) {
            imu->Update();  // Update IMU data
        }
        osDelay(IMU_TASK_DELAY);
    }
}

//==================================================================================================
// Self-Test Module
//==================================================================================================

/// Self-test task attributes definition
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

/// Self-test device objects
static bsp::Buzzer* buzzer = nullptr;
static display::OLED* OLED = nullptr;

/// Super Mario music array
using Note = bsp::BuzzerNote;
static bsp::BuzzerNoteDelayed Mario[] = {
        {Note::Mi3M, 80}, {Note::Silent, 80}, {Note::Mi3M, 80}, {Note::Silent, 240},
        {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::Do1M, 80}, {Note::Silent, 80},
        {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::So5M, 80}, {Note::Silent, 560},
        {Note::So5L, 80}, {Note::Silent, 0}, {Note::Finish, 0}
};

/// Device connection status flags
static bool left_motor_flag = false;      ///< Left wheel motor connection status
static bool right_motor_flag = false;     ///< Right wheel motor connection status
static bool transform_motor_flag = false; ///< Transform motor connection status
static bool sbus_flag = false;            ///< SBUS connection status
static bool computer_flag = false;        ///< Computer connection status

/**
 * @brief Self-test task
 * @param arg Task argument (unused)
 */
extern "C" void selfTestTask(void* arg) {
    UNUSED(arg);
    osDelay(SELFTEST_TASK_DELAY);

    if (OLED && buzzer) {
        // Startup screen
        OLED->ShowIlliniRMLOGO();
        buzzer->SingSong(Mario, [](uint32_t milli) { osDelay(milli); });
        OLED->OperateGram(display::PEN_CLEAR);
        OLED->RefreshGram();

        // Display fixed labels - readjusted positions to avoid overlap
        OLED->ShowString(0, 0, (uint8_t*)"LW");    // Left wheel motor (column 0-1)
        OLED->ShowString(0, 4, (uint8_t*)"RW");    // Right wheel motor (column 4-5)
        OLED->ShowString(0, 8, (uint8_t*)"TF");    // Transform motor (column 8-9)
        OLED->ShowString(0, 12, (uint8_t*)"RC");   // Remote controller (column 12-13)
        OLED->ShowString(0, 16, (uint8_t*)"PC");   // Computer (column 16-17)
    }

    uint32_t start_time = osKernelGetTickCount();

    while (true) {
        // Detect device connection status
        if (left_wheel_motor) left_wheel_motor->connection_flag_ = false;
        if (right_wheel_motor) right_wheel_motor->connection_flag_ = false;
        if (transform_motor) transform_motor->connection_flag_ = false;
        if (sbus) sbus->connection_flag_ = false;

        osDelay(SELFTEST_TASK_DELAY);

        // Read connection status
        left_motor_flag = left_wheel_motor ? left_wheel_motor->connection_flag_ : false;
        right_motor_flag = right_wheel_motor ? right_wheel_motor->connection_flag_ : false;
        transform_motor_flag = transform_motor ? transform_motor->connection_flag_ : false;
        sbus_flag = sbus ? sbus->connection_flag_ : false;

        if (OLED) {
            // Row 0: Device status indicator blocks - adjusted positions to avoid text overlap
            OLED->ShowBlock(0, 2, left_motor_flag);      // Column 2-3
            OLED->ShowBlock(0, 6, right_motor_flag);     // Column 6-7
            OLED->ShowBlock(0, 10, transform_motor_flag); // Column 10-11
            OLED->ShowBlock(0, 14, sbus_flag);           // Column 14-15
            OLED->ShowBlock(0, 18, computer_flag);       // Column 18-19

            // Row 1: Control mode + Transform mode + Runtime
            const char* mode_str = current_control_mode == EMERGENCY_STOP_MODE ? "STOP" :
                                   current_control_mode == REMOTE_CONTROL_MODE ? "RC" : "PC";
            const char* transform_str = current_transform_mode == VEHICLE_MODE ? "CAR" : "FLY";

            uint32_t runtime_sec = (osKernelGetTickCount() - start_time) / 1000;
            uint32_t hours = runtime_sec / 3600;
            uint32_t minutes = (runtime_sec % 3600) / 60;
            uint32_t seconds = runtime_sec % 60;

            OLED->Printf(1, 0, "M:%s T:%s %02d:%02d:%02d",
                         mode_str, transform_str, hours, minutes, seconds);

            // Row 2: Transform motor angle - fix floating point display
            if (transform_motor && transform_motor->connection_flag_) {
                // Convert to integer display (retain one decimal place)
                int angle_int = (int)(transform_motor->GetTheta() * 573);  // 573 = 180*10/π
                OLED->Printf(2, 0, "TF: %4d.%d deg", angle_int/10, abs(angle_int%10));
            } else {
                OLED->Printf(2, 0, "TF: --- deg");
            }

            // Row 3: IMU Euler angles - fix floating point display
            if (imu) {
                // Convert to integer display (retain one decimal place)
                int pitch_int = (int)(imu->INS_angle[1] * 573);  // 573 = 180*10/π
                int roll_int = (int)(imu->INS_angle[2] * 573);
                int yaw_int = (int)(imu->INS_angle[0] * 573);

                OLED->Printf(3, 0, "P:%d.%d R:%d.%d Y:%d.%d",
                             pitch_int/10, abs(pitch_int%10),
                             roll_int/10, abs(roll_int%10),
                             yaw_int/10, abs(yaw_int%10));
            } else {
                OLED->Printf(3, 0, "P:--- R:--- Y:---");
            }

            // Row 4: Actuator status
            OLED->Printf(2, 12, "ACT:%s", actuator_pulsing ? "PULSE" : "IDLE");

            OLED->RefreshGram();
        }

        osDelay(100);
    }
}

//==================================================================================================
// Communication Module
//==================================================================================================

#define USB_RX_SIGNAL (1 << 0)

/// Communication task attributes definition
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

/**
 * @brief Custom communication USB class
 */
class CustomCommUSB : public bsp::VirtualUSB {
protected:
    void RxCompleteCallback() override final {
        osThreadFlagsSet(communicationTaskHandle, USB_RX_SIGNAL);
    }
};

static CustomCommUSB* comm_usb = nullptr;
static communication::MinipcPort* minipc_session = nullptr;

/// Computer connection status detection
static uint32_t last_pc_data_time = 0;                  ///< Last time computer data was received
static const uint32_t PC_TIMEOUT_MS = 2000;             ///< Computer timeout 2 seconds

/// Selfcheck统计变量
static uint32_t selfcheck_tx_count = 0;        ///< Selfcheck发送计数
static uint32_t selfcheck_rx_count = 0;        ///< Selfcheck接收计数
static uint32_t selfcheck_error_count = 0;     ///< Selfcheck错误计数
static uint32_t selfcheck_last_ping_time = 0;  ///< 最后PING时间
static uint8_t selfcheck_last_ping_seq = 0;    ///< 最后PING序列号

/**
 * @brief 获取状态值
 * @param query_type 查询类型
 * @return 对应的状态值
 */
uint8_t GetSelfcheckStatusValue(uint8_t query_type) {
    switch (query_type) {
        case 0: // 通信统计 - 返回接收计数的低8位
            return (uint8_t)(selfcheck_rx_count & 0xFF);
        case 1: // 系统运行时间 - 返回运行时间(秒)的低8位
            return (uint8_t)((osKernelGetTickCount() / 1000) & 0xFF);
        case 2: // 错误计数
            return (uint8_t)(selfcheck_error_count & 0xFF);
        case 3: // 发送计数
            return (uint8_t)(selfcheck_tx_count & 0xFF);
        case 4: // 最后PING延迟 (毫秒)
            if (selfcheck_last_ping_time > 0) {
                uint32_t delay = osKernelGetTickCount() - selfcheck_last_ping_time;
                return (uint8_t)(delay > 255 ? 255 : delay);
            }
            return 255; // 表示无效
        default:
            return 0xFF; // 无效查询
    }
}

/**
 * @brief 执行重置操作
 * @param reset_type 重置类型
 * @return 重置结果 (0=成功, 非0=失败)
 */
uint8_t ExecuteSelfcheckReset(uint8_t reset_type) {
    switch (reset_type) {
        case 0: // 重置通信计数器
            selfcheck_tx_count = 0;
            selfcheck_rx_count = 0;
            return 0;
        case 1: // 重置错误计数器
            selfcheck_error_count = 0;
            return 0;
        case 2: // 重置所有统计
            selfcheck_tx_count = 0;
            selfcheck_rx_count = 0;
            selfcheck_error_count = 0;
            selfcheck_last_ping_time = 0;
            selfcheck_last_ping_seq = 0;
            return 0;
        default:
            return 1; // 无效重置类型
    }
}

/**
 * @brief 发送selfcheck响应
 * @param response_data 响应数据
 */
void SendSelfcheckResponse(const communication::selfcheck_data_t* response_data) {
    if (!response_data || !minipc_session || !comm_usb) return;

    uint8_t packet[minipc_session->MAX_PACKET_LENGTH];
    minipc_session->Pack(packet, (void*)response_data, communication::SELFCHECK_CMD_ID);
    comm_usb->Write(packet, minipc_session->GetPacketLen(communication::SELFCHECK_CMD_ID));

    selfcheck_tx_count++;
}

/**
 * @brief 处理selfcheck命令
 * @param status 接收到的状态数据
 */
void HandleSelfcheckCommand(const communication::status_data_t* status) {
    if (!status) return;

    uint8_t mode = status->mode;
    uint8_t debug_int = status->debug_int;

    // 更新接收计数
    selfcheck_rx_count++;

    communication::selfcheck_data_t response;
    bool send_response = true;

    switch (mode) {
        case 0: // PING测试
            response.mode = 0;
            response.debug_int = debug_int; // 原样返回序列号
            selfcheck_last_ping_time = osKernelGetTickCount();
            selfcheck_last_ping_seq = debug_int;
            break;

        case 1: // ECHO测试
            response.mode = 1;
            response.debug_int = debug_int; // 原样返回数据
            break;

        case 2: // STATUS查询
            response.mode = 2;
            response.debug_int = GetSelfcheckStatusValue(debug_int);
            break;

        case 3: // RESET命令
            response.mode = 3;
            response.debug_int = ExecuteSelfcheckReset(debug_int);
            break;

        default:
            // 无效模式，记录错误但不回复
            selfcheck_error_count++;
            send_response = false;
            break;
    }

    // 发送回复
    if (send_response) {
        SendSelfcheckResponse(&response);
    }
}

/**
 * @brief Get system timestamp
 * @return Current system timestamp
 */
uint32_t GetSystemTimestamp() {
    return osKernelGetTickCount();
}

/**
 * @brief Send IMU data to computer
 */
void SendIMUData() {
    if (!imu || !minipc_session || !comm_usb) return;

    communication::imu_data_t imu_data;

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
    imu_data.timestamp = GetSystemTimestamp();

    uint8_t packet[minipc_session->MAX_PACKET_LENGTH];
    minipc_session->Pack(packet, &imu_data, communication::IMU_CMD_ID);
    comm_usb->Write(packet, minipc_session->GetPacketLen(communication::IMU_CMD_ID));
}

/**
 * @brief Send odometry data to computer
 */
void SendOdometryData() {
    if (!left_wheel_motor || !right_wheel_motor || !minipc_session || !comm_usb) return;

    communication::odometry_data_t odom_data;

    static const float WHEEL_RADIUS = 0.076f;    ///< Wheel radius (76mm)
    static const float WHEEL_BASE = 0.335f;      ///< Wheel base (335mm)

    float left_wheel_speed = left_wheel_motor->GetOmega();
    float right_wheel_speed = right_wheel_motor->GetOmega();

    float linear_vel = (left_wheel_speed + right_wheel_speed) * WHEEL_RADIUS / 2.0f;
    float angular_vel = (right_wheel_speed - left_wheel_speed) * WHEEL_RADIUS / WHEEL_BASE;

    static int32_t left_encoder_total = 0;
    static int32_t right_encoder_total = 0;
    left_encoder_total += (int32_t)(left_wheel_speed * 0.05f * 8192 / (2 * 3.14159f));   // 50ms period
    right_encoder_total += (int32_t)(right_wheel_speed * 0.05f * 8192 / (2 * 3.14159f));

    odom_data.left_encoder = left_encoder_total;
    odom_data.right_encoder = right_encoder_total;
    odom_data.left_wheel_speed = left_wheel_speed;
    odom_data.right_wheel_speed = right_wheel_speed;
    odom_data.linear_velocity = linear_vel;
    odom_data.angular_velocity = angular_vel;
    odom_data.wheel_base = WHEEL_BASE;
    odom_data.wheel_radius = WHEEL_RADIUS;
    odom_data.timestamp = GetSystemTimestamp();

    uint8_t packet[minipc_session->MAX_PACKET_LENGTH];
    minipc_session->Pack(packet, &odom_data, communication::ODOMETRY_CMD_ID);
    comm_usb->Write(packet, minipc_session->GetPacketLen(communication::ODOMETRY_CMD_ID));
}

/**
 * @brief Send system status to computer
 */
void SendSystemStatus() {
    if (!minipc_session || !comm_usb) return;

    communication::system_status_t status;

    status.robot_mode = (uint8_t)current_control_mode;
    status.transform_mode = (uint8_t)current_transform_mode;
    status.sbus_connected = sbus ? sbus->connection_flag_ : 0;
    status.imu_connected = imu ? 1 : 0;
    status.vl_motor_online = left_wheel_motor ? left_wheel_motor->connection_flag_ : 0;
    status.vr_motor_online = right_wheel_motor ? right_wheel_motor->connection_flag_ : 0;
    status.tf_motor_online = transform_motor ? transform_motor->connection_flag_ : 0;
    status.reserved = 0;
    status.timestamp = GetSystemTimestamp();

    uint8_t packet[minipc_session->MAX_PACKET_LENGTH];
    minipc_session->Pack(packet, &status, communication::SYSTEM_STATUS_CMD_ID);
    comm_usb->Write(packet, minipc_session->GetPacketLen(communication::SYSTEM_STATUS_CMD_ID));
}

/**
 * @brief Handle received motion control commands
 * @param status Received status data
 */
void HandleMotionCommand(const communication::status_data_t* status) {
    if (!status || current_control_mode != COMPUTER_CONTROL_MODE) return;

    float target_linear = status->target_linear_vel;
    float target_angular = status->target_angular_vel;

    // Emergency stop handling
    if (status->emergency_stop) {
        if (osMutexAcquire(motor_control_mutex, 10) == osOK) {  // 10ms timeout
            if (left_wheel_motor) left_wheel_motor->SetOutput(0);
            if (right_wheel_motor) right_wheel_motor->SetOutput(0);

            // Send stop command immediately
            if (left_wheel_motor && right_wheel_motor &&
                (left_wheel_motor->connection_flag_ || right_wheel_motor->connection_flag_)) {
                control::MotorCANBase* motors[2] = {left_wheel_motor, right_wheel_motor};
                control::MotorCANBase::TransmitOutput(motors, 2);
            }

            osMutexRelease(motor_control_mutex);
        }
        return;
    }

    static const float WHEEL_RADIUS = 0.076f;
    static const float WHEEL_BASE = 0.335f;
    static const float MAX_WHEEL_SPEED = 10.0f;

    float left_wheel_vel = (target_linear - target_angular * WHEEL_BASE / 2.0f) / WHEEL_RADIUS;
    float right_wheel_vel = (target_linear + target_angular * WHEEL_BASE / 2.0f) / WHEEL_RADIUS;

    left_wheel_vel = clip<float>(left_wheel_vel, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
    right_wheel_vel = clip<float>(right_wheel_vel, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);

    if (left_wheel_motor && right_wheel_motor && left_motor_pid && right_motor_pid) {
        float left_error = left_wheel_vel - left_wheel_motor->GetOmega();
        float right_error = right_wheel_vel - right_wheel_motor->GetOmega();

        int16_t left_output = left_motor_pid->ComputeConstrainedOutput(left_error);
        int16_t right_output = right_motor_pid->ComputeConstrainedOutput(right_error);

        // Use mutex to protect motor control
        if (osMutexAcquire(motor_control_mutex, 10) == osOK) {  // 10ms timeout
            left_wheel_motor->SetOutput(left_output);
            right_wheel_motor->SetOutput(right_output);

            // Send CAN messages (this was the missing critical code before)
            if (left_wheel_motor->connection_flag_ || right_wheel_motor->connection_flag_) {
                control::MotorCANBase* motors[2] = {left_wheel_motor, right_wheel_motor};
                control::MotorCANBase::TransmitOutput(motors, 2);
            }

            osMutexRelease(motor_control_mutex);
        }
    }
}

/**
 * @brief Communication task (50ms delay)
 *
 * Functions:
 * 1. Receive and process computer commands
 * 2. Send IMU, odometry and system status data periodically
 * 3. Detect computer connection status based on data reception timeout
 *
 * @param arg Task argument (unused)
 */
extern "C" void communicationTask(void* arg) {
    UNUSED(arg);

    minipc_session = new communication::MinipcPort();

    uint8_t *data;
    uint32_t length;

    uint32_t last_imu_send = 0;
    uint32_t last_odom_send = 0;
    uint32_t last_status_send = 0;

    const uint32_t IMU_SEND_PERIOD = 10;      ///< 10ms = 100Hz
    const uint32_t ODOM_SEND_PERIOD = 20;     ///< 20ms = 50Hz
    const uint32_t STATUS_SEND_PERIOD = 200;  ///< 200ms = 5Hz

    while (true) {
        uint32_t current_time = osKernelGetTickCount();

        // Check received data
        uint32_t flags = osThreadFlagsWait(USB_RX_SIGNAL, osFlagsWaitAny, 0);
        if (flags & USB_RX_SIGNAL && comm_usb) {
            length = comm_usb->Read(&data);
            if (length > 0 && minipc_session) {
                // Data received, update computer data reception timestamp
                last_pc_data_time = current_time;

                minipc_session->ParseUartBuffer(data, length);

                if (minipc_session->GetValidFlag()) {
                    uint8_t cmd_id = minipc_session->GetCmdId();
                    const communication::status_data_t* status = minipc_session->GetStatus();

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

        // Determine computer connection status based on data reception timeout
        // Only consider computer online if data was received within PC_TIMEOUT_MS (2 seconds)
        computer_flag = (current_time - last_pc_data_time) < PC_TIMEOUT_MS;

        // Send data periodically
        if (current_time - last_imu_send >= IMU_SEND_PERIOD) {
            SendIMUData();
            last_imu_send = current_time;
        }

        if (current_time - last_odom_send >= ODOM_SEND_PERIOD) {
            SendOdometryData();
            last_odom_send = current_time;
        }

        if (current_time - last_status_send >= STATUS_SEND_PERIOD) {
            SendSystemStatus();
            last_status_send = current_time;
        }

        osDelay(COMMUNICATION_TASK_DELAY);  // 50ms delay
    }
}

//==================================================================================================
// LED Control Module
//==================================================================================================

static display::RGB* led = nullptr;

/// LED task attributes definition
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

/**
 * @brief LED control task (RGB flowing light effect)
 * @param arg Task argument (unused)
 */
extern "C" void ledTask(void* arg) {
    UNUSED(arg);

    int RGB_FLOW_COLOR_CHANGE_TIME = 300;
    uint32_t RGB_flow_color[3] = {0xFFFF0000, 0xFF00FF00, 0xFF0000FF};

    float delta_alpha, delta_red, delta_green, delta_blue;
    float alpha, red, green, blue;
    uint32_t aRGB;
    int i = 0;

    while (true) {
        alpha = (RGB_flow_color[i] & 0xFF000000) >> 24;
        red = ((RGB_flow_color[i] & 0x00FF0000) >> 16);
        green = ((RGB_flow_color[i] & 0x0000FF00) >> 8);
        blue = ((RGB_flow_color[i] & 0x000000FF) >> 0);

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

        for (int j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; ++j) {
            alpha += delta_alpha;
            red += delta_red;
            green += delta_green;
            blue += delta_blue;

            aRGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 |
                   ((uint32_t)(green)) << 8 | ((uint32_t)(blue)) << 0;

            if (led) {
                led->Display(aRGB);
            }

            osDelay(LED_TASK_DELAY);
        }

        ++i;
        i = i % 3;
    }
}

//==================================================================================================
// Control Mode Management
//==================================================================================================

/**
 * @brief Emergency stop all motors
 */
void EmergencyStopAllMotors() {
    if (osMutexAcquire(motor_control_mutex, 100) == osOK) {  // 100ms timeout, must acquire lock in emergency
        if (left_wheel_motor) left_wheel_motor->SetOutput(0);
        if (right_wheel_motor) right_wheel_motor->SetOutput(0);
        if (transform_motor) transform_motor->SetOutput(0);

        // Send stop commands
        if (left_wheel_motor && right_wheel_motor &&
            (left_wheel_motor->connection_flag_ || right_wheel_motor->connection_flag_)) {
            control::MotorCANBase* motors[2] = {left_wheel_motor, right_wheel_motor};
            control::MotorCANBase::TransmitOutput(motors, 2);
        }

        if (transform_motor && transform_motor->connection_flag_) {
            control::MotorCANBase* motors[1] = {transform_motor};
            control::MotorCANBase::TransmitOutput(motors, 1);
        }

        osMutexRelease(motor_control_mutex);
    }
}

/**
 * @brief Determine current control mode
 * @return Current control mode
 */
control_mode_t GetControlMode() {
    if (!sbus || !sbus->connection_flag_) {
        return EMERGENCY_STOP_MODE;
    }

    uint32_t current_time = HAL_GetTick();
    if (current_time - sbus->timestamp > SBUS_TIMEOUT_MS) {
        return EMERGENCY_STOP_MODE;
    }

    // Determine control mode based on channel 6 (ch[5])
    int16_t mode_channel = sbus->ch[5];

    if (mode_channel < MODE_LOWER_THRESHOLD) {
        return EMERGENCY_STOP_MODE;      // Emergency stop mode
    } else if (mode_channel > MODE_UPPER_THRESHOLD) {
        return COMPUTER_CONTROL_MODE;    // Computer control mode
    } else {
        return REMOTE_CONTROL_MODE;      // Remote control mode
    }
}

/**
 * @brief Determine current transform mode
 * @return Current transform mode
 */
transform_mode_t GetTransformMode() {
    if (!sbus || !sbus->connection_flag_) {
        return VEHICLE_MODE;  // Default vehicle mode
    }

    // Determine transform mode based on channel 5 (ch[4])
    int16_t transform_channel = sbus->ch[4];

    return (transform_channel > 0) ? FLIGHT_MODE : VEHICLE_MODE;
}

/**
 * @brief Handle control mode switching
 */
void HandleModeSwitch() {
    if (current_control_mode != previous_control_mode) {
        switch (current_control_mode) {
            case EMERGENCY_STOP_MODE:
                // Emergency stop mode: immediately stop all motors and reset PID
                EmergencyStopAllMotors();
                if (left_motor_pid) left_motor_pid->Reset();
                if (right_motor_pid) right_motor_pid->Reset();
                break;
            case REMOTE_CONTROL_MODE:
                // Remote control mode: reset PID controllers
                if (left_motor_pid) left_motor_pid->Reset();
                if (right_motor_pid) right_motor_pid->Reset();
                break;
            case COMPUTER_CONTROL_MODE:
                // Computer control mode: reset PID controllers
                if (left_motor_pid) left_motor_pid->Reset();
                if (right_motor_pid) right_motor_pid->Reset();
                break;
        }
        previous_control_mode = current_control_mode;
    }
}

//==================================================================================================
// RTOS Initialization
//==================================================================================================

/**
 * @brief RTOS system and hardware initialization
 */
extern "C" void RTOS_Init() {
    // Create motor control mutex
    motor_control_mutex = osMutexNew(NULL);
    if (motor_control_mutex == NULL) {
        // Mutex creation failed, system cannot work properly
        while(1) {
            // Error indication, can add LED blinking or other indication
            HAL_Delay(100);
        }
    }

    // Serial print initialization
    print_use_uart(&huart1);

    // IMU initialization
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

    // SBUS remote controller initialization
    sbus = new remote::SBUS(&huart3);

    // Buzzer initialization
    buzzer = new bsp::Buzzer(&htim4, 3, 1000000);

    // OLED display initialization
    OLED = new display::OLED(&hi2c2, 0x3C);

    // RGB LED initialization
    led = new display::RGB(&htim5, 3, 2, 1, 1000000);

    // CAN bus initialization
    can1 = new bsp::CAN(&hcan1, true);

    // Vehicle motor initialization (left wheel: CAN ID 1, right wheel: CAN ID 2)
    left_wheel_motor = new control::Motor2006(can1, 0x201);   // Left wheel motor
    right_wheel_motor = new control::Motor2006(can1, 0x202);  // Right wheel motor

    // Transform motor initialization (3508: CAN ID 3)
    transform_motor = new control::Motor3508(can1, 0x203);    // Transform motor

    // Transform motor servo initialization
    control::servo_t transform_servo_data;
    transform_servo_data.motor = transform_motor;
    transform_servo_data.max_speed = TRANSFORM_MOTOR_SPEED;
    transform_servo_data.max_acceleration = TRANSFORM_MOTOR_ACCELERATION;
    transform_servo_data.transmission_ratio = M3508P19_RATIO;
    transform_servo_data.omega_pid_param = new float[3]{30, 0.2, 50};
    transform_servo_data.max_iout = 1000;
    transform_servo_data.max_out = 8000;
    transform_servo = new control::ServoMotor(transform_servo_data);

    // Register jam detection callback, threshold set to 0.6 (adjustable based on actual conditions)
    transform_servo->RegisterJamCallback(transform_jam_callback, 0.6);

    // Actuator GPIO initialization (PI7)
    actuator_gpio = new bsp::GPIO(GPIOI, GPIO_PIN_7);
    actuator_gpio->High();  // Initialize to high level (inactive state)

    // PID controller initialization
    left_motor_pid = new control::ConstrainedPID(left_motor_pid_params[0],
                                                 left_motor_pid_params[1],
                                                 left_motor_pid_params[2],
                                                 3000.0f, 10000.0f);

    right_motor_pid = new control::ConstrainedPID(right_motor_pid_params[0],
                                                  right_motor_pid_params[1],
                                                  right_motor_pid_params[2],
                                                  3000.0f, 10000.0f);

    // USB communication initialization
    comm_usb = new CustomCommUSB();
    comm_usb->SetupRx(512);
    comm_usb->SetupTx(512);

    // Initialize computer connection status detection
    last_pc_data_time = 0;  // Set to 0 at startup to ensure offline display before receiving data

    // Suction fan PWM initialization (TIM1_CH1)
    suction_fan = new bsp::PWM(&htim1, 1, FAN_TIMER_CLOCK, FAN_PWM_FREQUENCY, MIN_FAN_PULSE_WIDTH);
}

//==================================================================================================
// RTOS Task Initialization
//==================================================================================================

/**
 * @brief Create all RTOS tasks
 */
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

/**
 * @brief RTOS default task - manage control modes
 * @param args Task arguments (unused)
 */
extern "C" void RTOS_Default_Task(const void* args) {
    UNUSED(args);

    while (true) {
        // Get current control mode and transform mode
        current_control_mode = GetControlMode();
        current_transform_mode = GetTransformMode();

        // Handle mode switching
        HandleModeSwitch();

        // In emergency stop mode, continuously ensure all motors are stopped
        if (current_control_mode == EMERGENCY_STOP_MODE) {
            EmergencyStopAllMotors();
        }

        osDelay(DEFAULT_TASK_DELAY);
    }
}