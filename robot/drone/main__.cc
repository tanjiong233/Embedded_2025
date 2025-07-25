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

#include <memory>
#include <functional>
#include <array>

//==================================================================================================
// Control Mode Definitions
//==================================================================================================

/**
 * @brief Control mode enumeration
 */
enum class ControlMode : uint8_t {
    EMERGENCY_STOP = 0,    ///< Emergency stop mode (channel 6 down position)
    REMOTE_CONTROL = 1,    ///< Remote control mode (channel 6 middle position)
    COMPUTER_CONTROL = 2   ///< Computer control mode (channel 6 up position)
};

/**
 * @brief Transform mode enumeration
 */
enum class TransformMode : uint8_t {
    VEHICLE = 0,           ///< Vehicle mode (channel 5 down position)
    FLIGHT = 1             ///< Flight mode (channel 5 up position)
};

//==================================================================================================
// System State Manager Class
//==================================================================================================

class SystemStateManager {
public:
    SystemStateManager() = default;

    // Delete copy constructor and assignment operator
    SystemStateManager(const SystemStateManager&) = delete;
    SystemStateManager& operator=(const SystemStateManager&) = delete;

    void Init(remote::SBUS* sbus_ptr) {
        sbus_ = sbus_ptr;
        motor_control_mutex_ = osMutexNew(nullptr);
        if (motor_control_mutex_ == nullptr) {
            // Mutex creation failed, system cannot work properly
            while(1) {
                HAL_Delay(100);
            }
        }
    }

    ControlMode GetCurrentControlMode() const { return current_control_mode_; }
    TransformMode GetCurrentTransformMode() const { return current_transform_mode_; }
    osMutexId_t GetMotorControlMutex() const { return motor_control_mutex_; }

    /**
     * @brief Update control modes based on SBUS input
     */
    void UpdateControlModes() {
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

    bool IsInEmergencyStop() const {
        return current_control_mode_ == ControlMode::EMERGENCY_STOP;
    }

    void SetPIDResetCallback(std::function<void()> callback) {
        pid_reset_callback_ = callback;
    }

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

    ControlMode DetermineControlMode() {
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

    TransformMode DetermineTransformMode() {
        if (!sbus_ || !sbus_->connection_flag_) {
            return TransformMode::VEHICLE;
        }

        int16_t transform_channel = sbus_->ch[4];
        return (transform_channel > 0) ? TransformMode::FLIGHT : TransformMode::VEHICLE;
    }

    void HandleModeSwitch() {
        if (current_control_mode_ == ControlMode::EMERGENCY_STOP) {
            // Emergency stop mode: immediately stop all motors and reset PID
            if (pid_reset_callback_) {
                pid_reset_callback_();
            }
        }
    }
};

//==================================================================================================
// Vehicle Control Class
//==================================================================================================

class VehicleController {
public:
    static constexpr uint32_t TASK_DELAY_MS = 2;

    VehicleController() = default;

    void Init(bsp::CAN* can, SystemStateManager* state_manager, remote::SBUS* sbus) {
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

    void Execute() {
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

    void EmergencyStop() {
        if (osMutexAcquire(state_manager_->GetMotorControlMutex(), 100) == osOK) {
            if (left_motor_) left_motor_->SetOutput(0);
            if (right_motor_) right_motor_->SetOutput(0);

            SendMotorCommands();

            osMutexRelease(state_manager_->GetMotorControlMutex());
        }
    }

    // Allow computer control access to motors
    void SetComputerControl(float linear_vel, float angular_vel) {
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

    static int16_t ApplyDeadzone(int16_t input, int16_t deadzone) {
        return (abs(input) < deadzone) ? 0 : input;
    }

    static float NormalizeSBUS(int16_t input) {
        return static_cast<float>(input) / SBUS_MAX_VALUE;
    }

    void ProcessSBUSInputs() {
        int16_t throttle_raw = ApplyDeadzone(sbus_->ch[2], SBUS_DEADZONE);
        int16_t steering_raw = ApplyDeadzone(sbus_->ch[3], SBUS_DEADZONE);

        float throttle = NormalizeSBUS(throttle_raw);
        float steering = NormalizeSBUS(steering_raw);

        target_linear_speed_ = throttle * MAX_VEHICLE_SPEED;
        target_angular_speed_ = steering * MAX_VEHICLE_SPEED * TURN_SENSITIVITY;
    }

    void CalculateDifferentialSpeeds() {
        left_wheel_speed_ = target_linear_speed_ - target_angular_speed_;
        right_wheel_speed_ = target_linear_speed_ + target_angular_speed_;

        left_wheel_speed_ = clip<float>(left_wheel_speed_, -MAX_VEHICLE_SPEED, MAX_VEHICLE_SPEED);
        right_wheel_speed_ = clip<float>(right_wheel_speed_, -MAX_VEHICLE_SPEED, MAX_VEHICLE_SPEED);
    }

    void ControlMotors() {
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

    void StopMotors() {
        if (left_motor_) left_motor_->SetOutput(0);
        if (right_motor_) right_motor_->SetOutput(0);
        SendMotorCommands();
    }

    void SendMotorCommands() {
        if (left_motor_ && right_motor_ &&
            (left_motor_->connection_flag_ || right_motor_->connection_flag_)) {
            control::MotorCANBase* motors[2] = {left_motor_.get(), right_motor_.get()};
            control::MotorCANBase::TransmitOutput(motors, 2);
        }
    }

    void ResetPIDs() {
        StopMotors();
        if (left_pid_) left_pid_->Reset();
        if (right_pid_) right_pid_->Reset();
    }
};

//==================================================================================================
// Transform Control Class
//==================================================================================================

class TransformController {
public:
    static constexpr uint32_t TASK_DELAY_MS = 10;

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

    void Init(bsp::CAN* can, SystemStateManager* state_manager) {
        can_ = can;
        state_manager_ = state_manager;

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

    void Execute() {
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

    static void JamCallback(control::ServoMotor* servo, const control::servo_jam_t data) {
        UNUSED(servo);
        UNUSED(data);
        if (instance_) {
            instance_->motor_jammed_ = true;
        }
    }

    void SendActuatorPulse() {
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

    void UpdateActuatorPulse() {
        if (!actuator_pulsing_ || !actuator_gpio_) return;

        uint32_t current_time = HAL_GetTick();
        if (current_time - last_pulse_time_ >= ACTUATOR_PULSE_WIDTH) {
            actuator_gpio_->High();
            actuator_pulsing_ = false;
        }
    }

    void StartActuatorExtension() {
        actuator_target_ = ActuatorTarget::EXTENDING;
        operation_start_time_ = HAL_GetTick();

        if (actuator_state_ != ActuatorState::EXTENDING &&
            actuator_state_ != ActuatorState::STOPPED ||
            (actuator_state_ == ActuatorState::STOPPED && pulse_count_ % 4 != 2)) {
            SendActuatorPulse();
        }
    }

    void StartActuatorRetraction() {
        actuator_target_ = ActuatorTarget::RETRACTING;
        operation_start_time_ = HAL_GetTick();

        if (actuator_state_ != ActuatorState::RETRACTING &&
            actuator_state_ != ActuatorState::STOPPED ||
            (actuator_state_ == ActuatorState::STOPPED && pulse_count_ % 4 != 0)) {
            SendActuatorPulse();
        }
    }

    bool IsActuatorAtTarget() {
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

    void UpdateActuatorAutoControl() {
        if (IsActuatorAtTarget() || actuator_pulsing_) return;

        if (actuator_target_ != ActuatorTarget::NONE) {
            SendActuatorPulse();
        }
    }

    void StartMotorForwardRotation() {
        if (!transform_servo_) return;

        motor_jammed_ = false;
        float current_angle = transform_servo_->GetTheta();
        float target_angle = current_angle + 2 * PI;
        transform_servo_->SetTarget(target_angle, true);
    }

    void StartMotorBackwardRotation() {
        if (!transform_servo_) return;

        motor_jammed_ = false;
        float current_angle = transform_servo_->GetTheta();
        float target_angle = current_angle - 2 * PI;
        transform_servo_->SetTarget(target_angle, true);
    }

    void StopTransformMotor() {
        if (!transform_servo_) return;

        float current_angle = transform_servo_->GetTheta();
        transform_servo_->SetTarget(current_angle, true);
    }

    bool IsActuatorOperationComplete() {
        uint32_t current_time = HAL_GetTick();
        return (current_time - operation_start_time_ >= ACTUATOR_OPERATION_DELAY);
    }

    void HandleTransformModeChange() {
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

    void RunStateMachine() {
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
};
TransformController* TransformController::instance_ = nullptr;
//==================================================================================================
// Suction Control Class
//==================================================================================================

class SuctionController {
public:
    static constexpr uint32_t TASK_DELAY_MS = 20;

    SuctionController() = default;

    void Init(TIM_HandleTypeDef* htim, SystemStateManager* state_manager, remote::SBUS* sbus) {
        state_manager_ = state_manager;
        sbus_ = sbus;

        suction_fan_ = std::make_unique<bsp::PWM>(
                htim, 1, FAN_TIMER_CLOCK, FAN_PWM_FREQUENCY, MIN_FAN_PULSE_WIDTH);

        suction_fan_->Start();
        suction_fan_->SetPulseWidth(MIN_FAN_PULSE_WIDTH);
    }

    void Execute() {
        if (!suction_fan_) {
            osDelay(TASK_DELAY_MS);
            return;
        }

        if (state_manager_->GetCurrentControlMode() != ControlMode::REMOTE_CONTROL) {
            suction_fan_->SetPulseWidth(MIN_FAN_PULSE_WIDTH);
            osDelay(TASK_DELAY_MS);
            return;
        }

        if (!sbus_ || !sbus_->connection_flag_) {
            suction_fan_->SetPulseWidth(MIN_FAN_PULSE_WIDTH);
            osDelay(TASK_DELAY_MS);
            return;
        }

        int16_t suction_raw = ApplyDeadzone(sbus_->ch[1], SBUS_SUCTION_DEADZONE);
        float target_pulse_width = MapSuctionToPulseWidth(suction_raw);
        suction_fan_->SetPulseWidth(target_pulse_width);

        osDelay(TASK_DELAY_MS);
    }

private:
    static constexpr float MIN_FAN_PULSE_WIDTH = 1000.0f;
    static constexpr float MAX_FAN_PULSE_WIDTH = 2000.0f;
    static constexpr int SBUS_SUCTION_DEADZONE = 30;
    static constexpr int SBUS_MAX_VALUE = 660;
    static constexpr uint32_t FAN_PWM_FREQUENCY = 1500;
    static constexpr uint32_t FAN_TIMER_CLOCK = 1000000;

    SystemStateManager* state_manager_ = nullptr;
    remote::SBUS* sbus_ = nullptr;
    std::unique_ptr<bsp::PWM> suction_fan_;

    static int16_t ApplyDeadzone(int16_t input, int16_t deadzone) {
        return (abs(input) < deadzone) ? 0 : input;
    }

    static float MapSuctionToPulseWidth(int16_t sbus_input) {
        if (sbus_input <= 0) {
            return MIN_FAN_PULSE_WIDTH;
        }

        float normalized_input = static_cast<float>(sbus_input) / SBUS_MAX_VALUE;
        if (normalized_input > 1.0f) normalized_input = 1.0f;

        return MIN_FAN_PULSE_WIDTH + (normalized_input * (MAX_FAN_PULSE_WIDTH - MIN_FAN_PULSE_WIDTH));
    }
};

//==================================================================================================
// IMU Controller Class
//==================================================================================================

class IMUController {
public:
    static constexpr uint32_t TASK_DELAY_MS = 1;
    static constexpr uint32_t RX_SIGNAL = (1 << 1);

    class CustomIMU : public bsp::IMU_typeC {
    public:
        using bsp::IMU_typeC::IMU_typeC;

        void SetTaskHandle(osThreadId_t handle) {
            task_handle_ = handle;
        }

    protected:
        void RxCompleteCallback() final {
            if (task_handle_) {
                osThreadFlagsSet(task_handle_, RX_SIGNAL);
            }
        }

    private:
        osThreadId_t task_handle_ = nullptr;
    };

    IMUController() = default;

    void Init(const bsp::IMU_typeC_init_t& imu_init) {
        imu_ = std::make_unique<CustomIMU>(imu_init, false);
        if (imu_) {
            imu_->Calibrate();
        }
    }

    void SetTaskHandle(osThreadId_t handle) {
        if (imu_) {
            imu_->SetTaskHandle(handle);
        }
    }

    void Execute() {
        uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & RX_SIGNAL && imu_) {
            imu_->Update();
        }
        osDelay(TASK_DELAY_MS);
    }

    CustomIMU* GetIMU() { return imu_.get(); }

private:
    std::unique_ptr<CustomIMU> imu_;
};

//==================================================================================================
// LED Controller Class
//==================================================================================================

class LEDController {
public:
    static constexpr uint32_t TASK_DELAY_MS = 1;
    static constexpr int RGB_FLOW_COLOR_CHANGE_TIME = 300;

    LEDController() = default;

    void Init(TIM_HandleTypeDef* htim) {
        led_ = std::make_unique<display::RGB>(htim, 3, 2, 1, 1000000);
    }

    void Execute() {
        if (!led_) {
            osDelay(TASK_DELAY_MS);
            return;
        }

        static constexpr std::array<uint32_t, 3> RGB_flow_color = {
                0xFFFF0000, 0xFF00FF00, 0xFF0000FF
        };

        float alpha = (RGB_flow_color[color_index_] & 0xFF000000) >> 24;
        float red = ((RGB_flow_color[color_index_] & 0x00FF0000) >> 16);
        float green = ((RGB_flow_color[color_index_] & 0x0000FF00) >> 8);
        float blue = ((RGB_flow_color[color_index_] & 0x000000FF) >> 0);

        float delta_alpha = static_cast<float>((RGB_flow_color[(color_index_ + 1) % 3] & 0xFF000000) >> 24) -
                            static_cast<float>((RGB_flow_color[color_index_] & 0xFF000000) >> 24);
        float delta_red = static_cast<float>((RGB_flow_color[(color_index_ + 1) % 3] & 0x00FF0000) >> 16) -
                          static_cast<float>((RGB_flow_color[color_index_] & 0x00FF0000) >> 16);
        float delta_green = static_cast<float>((RGB_flow_color[(color_index_ + 1) % 3] & 0x0000FF00) >> 8) -
                            static_cast<float>((RGB_flow_color[color_index_] & 0x0000FF00) >> 8);
        float delta_blue = static_cast<float>((RGB_flow_color[(color_index_ + 1) % 3] & 0x000000FF) >> 0) -
                           static_cast<float>((RGB_flow_color[color_index_] & 0x000000FF) >> 0);

        delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;

        for (int j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; ++j) {
            alpha += delta_alpha;
            red += delta_red;
            green += delta_green;
            blue += delta_blue;

            uint32_t aRGB = (static_cast<uint32_t>(alpha)) << 24 |
                            (static_cast<uint32_t>(red)) << 16 |
                            (static_cast<uint32_t>(green)) << 8 |
                            (static_cast<uint32_t>(blue)) << 0;

            led_->Display(aRGB);
            osDelay(TASK_DELAY_MS);
        }

        ++color_index_;
        color_index_ = color_index_ % 3;
    }

private:
    std::unique_ptr<display::RGB> led_;
    int color_index_ = 0;
};

//==================================================================================================
// Self Test Controller Class
//==================================================================================================

class SelfTestController {
public:
    static constexpr uint32_t TASK_DELAY_MS = 100;

    struct DeviceStatus {
        bool left_motor = false;
        bool right_motor = false;
        bool transform_motor = false;
        bool sbus = false;
        bool computer = false;
    };

    SelfTestController() = default;

    void Init(I2C_HandleTypeDef* hi2c, TIM_HandleTypeDef* htim,
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

    void Execute() {
        if (!startup_sequence_played_) {
            ShowStartupSequence();
            startup_sequence_played_ = true;
        }
        UpdateDeviceStatus();
        UpdateOLEDDisplay();
        osDelay(TASK_DELAY_MS);
    }

    void SetComputerStatus(bool connected) {
        device_status_.computer = connected;
    }

private:
    bool startup_sequence_played_ = false;

    VehicleController* vehicle_ctrl_ = nullptr;
    TransformController* transform_ctrl_ = nullptr;
    remote::SBUS* sbus_ = nullptr;
    SystemStateManager* state_manager_ = nullptr;
    IMUController* imu_ctrl_ = nullptr;

    std::unique_ptr<display::OLED> OLED_;
    std::unique_ptr<bsp::Buzzer> buzzer_;

    DeviceStatus device_status_;
    uint32_t start_time_ = 0;

    void ShowStartupSequence() {
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

    void UpdateDeviceStatus() {
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

    void UpdateOLEDDisplay() {
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
};

//==================================================================================================
// Communication Controller Class
//==================================================================================================

class CommunicationController {
public:
    static constexpr uint32_t TASK_DELAY_MS = 50;
    static constexpr uint32_t USB_RX_SIGNAL = (1 << 0);

    class CustomCommUSB : public bsp::VirtualUSB {
    protected:
        void RxCompleteCallback() override final {
            if (task_handle_) {
                osThreadFlagsSet(task_handle_, USB_RX_SIGNAL);
            }
        }

    public:
        void SetTaskHandle(osThreadId_t handle) {
            task_handle_ = handle;
        }

    private:
        osThreadId_t task_handle_ = nullptr;
    };

    CommunicationController() = default;

    void Init(SystemStateManager* state_manager,
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

    void SetTaskHandle(osThreadId_t handle) {
        if (comm_usb_) {
            comm_usb_->SetTaskHandle(handle);
        }
    }

    void Execute() {
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

private:
    static constexpr uint32_t PC_TIMEOUT_MS = 2000;
    static constexpr uint32_t IMU_SEND_PERIOD = 10;
    static constexpr uint32_t ODOM_SEND_PERIOD = 20;
    static constexpr uint32_t STATUS_SEND_PERIOD = 200;

    SystemStateManager* state_manager_ = nullptr;
    VehicleController* vehicle_ctrl_ = nullptr;
    IMUController* imu_ctrl_ = nullptr;
    SelfTestController* self_test_ = nullptr;

    std::unique_ptr<CustomCommUSB> comm_usb_;
    std::unique_ptr<communication::MinipcPort> minipc_session_;

    uint32_t last_pc_data_time_ = 0;
    uint32_t last_imu_send_ = 0;
    uint32_t last_odom_send_ = 0;
    uint32_t last_status_send_ = 0;

    // Selfcheck statistics
    struct {
        uint32_t tx_count = 0;
        uint32_t rx_count = 0;
        uint32_t error_count = 0;
        uint32_t last_ping_time = 0;
        uint8_t last_ping_seq = 0;
    } selfcheck_stats_;

    void HandleMotionCommand(const communication::status_data_t* status) {
        if (!status || state_manager_->GetCurrentControlMode() != ControlMode::COMPUTER_CONTROL) return;

        if (status->emergency_stop) {
            vehicle_ctrl_->EmergencyStop();
            return;
        }

        vehicle_ctrl_->SetComputerControl(status->target_linear_vel, status->target_angular_vel);
    }

    void HandleSelfcheckCommand(const communication::status_data_t* status) {
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

    uint8_t GetSelfcheckStatusValue(uint8_t query_type) {
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

    uint8_t ExecuteSelfcheckReset(uint8_t reset_type) {
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

    void SendSelfcheckResponse(const communication::selfcheck_data_t* response_data) {
        if (!response_data || !minipc_session_ || !comm_usb_) return;

        uint8_t packet[minipc_session_->MAX_PACKET_LENGTH];
        minipc_session_->Pack(packet, (void*)response_data, communication::SELFCHECK_CMD_ID);
        comm_usb_->Write(packet, minipc_session_->GetPacketLen(communication::SELFCHECK_CMD_ID));

        selfcheck_stats_.tx_count++;
    }

    void SendIMUData() {
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

    void SendOdometryData() {
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

    void SendSystemStatus() {
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
};

//==================================================================================================
// Global Instances
//==================================================================================================

static remote::SBUS* g_sbus = nullptr;
static bsp::CAN* g_can1 = nullptr;

// Controller instances
static std::unique_ptr<SystemStateManager> g_state_manager;
static std::unique_ptr<VehicleController> g_vehicle_controller;
static std::unique_ptr<TransformController> g_transform_controller;
static std::unique_ptr<SuctionController> g_suction_controller;
static std::unique_ptr<IMUController> g_imu_controller;
static std::unique_ptr<LEDController> g_led_controller;
static std::unique_ptr<SelfTestController> g_self_test_controller;
static std::unique_ptr<CommunicationController> g_comm_controller;

//==================================================================================================
// Task Functions
//==================================================================================================

extern "C" void vehicleTask(void* arg) {
    UNUSED(arg);
    while (true) {
        if (g_vehicle_controller) {
            g_vehicle_controller->Execute();
        }
    }
}

extern "C" void transformTask(void* arg) {
    UNUSED(arg);
    osDelay(1000); // Wait for system initialization
    while (true) {
        if (g_transform_controller) {
            g_transform_controller->Execute();
        }
    }
}

extern "C" void suctionTask(void* arg) {
    UNUSED(arg);
    while (true) {
        if (g_suction_controller) {
            g_suction_controller->Execute();
        }
    }
}

extern "C" void imuTask(void* arg) {
    UNUSED(arg);
    while (true) {
        if (g_imu_controller) {
            g_imu_controller->Execute();
        }
    }
}

extern "C" void selfTestTask(void* arg) {
    UNUSED(arg);
    while (true) {
        if (g_self_test_controller) {
            g_self_test_controller->Execute();
        }
    }
}

extern "C" void communicationTask(void* arg) {
    UNUSED(arg);
    while (true) {
        if (g_comm_controller) {
            g_comm_controller->Execute();
        }
    }
}

extern "C" void ledTask(void* arg) {
    UNUSED(arg);
    while (true) {
        if (g_led_controller) {
            g_led_controller->Execute();
        }
    }
}

//==================================================================================================
// Task Attributes
//==================================================================================================

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

// Task handles
osThreadId_t vehicleTaskHandle;
osThreadId_t transformTaskHandle;
osThreadId_t suctionTaskHandle;
osThreadId_t imuTaskHandle;
osThreadId_t selfTestTaskHandle;
osThreadId_t communicationTaskHandle;
osThreadId_t ledTaskHandle;

//==================================================================================================
// RTOS Initialization
//==================================================================================================

extern "C" void RTOS_Init() {
    // Initialize serial print
    print_use_uart(&huart1);

    // Initialize global hardware
    g_sbus = new remote::SBUS(&huart3);
    g_can1 = new bsp::CAN(&hcan1, true);

    // Initialize controllers
    g_state_manager = std::make_unique<SystemStateManager>();
    g_state_manager->Init(g_sbus);

    g_vehicle_controller = std::make_unique<VehicleController>();
    g_vehicle_controller->Init(g_can1, g_state_manager.get(), g_sbus);

    g_transform_controller = std::make_unique<TransformController>();
    g_transform_controller->Init(g_can1, g_state_manager.get());

    g_suction_controller = std::make_unique<SuctionController>();
    g_suction_controller->Init(&htim1, g_state_manager.get(), g_sbus);

    // Initialize IMU
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

    g_imu_controller = std::make_unique<IMUController>();
    g_imu_controller->Init(imu_init);

    g_led_controller = std::make_unique<LEDController>();
    g_led_controller->Init(&htim5);

    g_self_test_controller = std::make_unique<SelfTestController>();
    g_self_test_controller->Init(&hi2c2, &htim4,
                                 g_vehicle_controller.get(),
                                 g_transform_controller.get(),
                                 g_sbus,
                                 g_state_manager.get(),
                                 g_imu_controller.get());

    g_comm_controller = std::make_unique<CommunicationController>();
    g_comm_controller->Init(g_state_manager.get(),
                            g_vehicle_controller.get(),
                            g_imu_controller.get(),
                            g_self_test_controller.get());
}

//==================================================================================================
// RTOS Task Initialization
//==================================================================================================

extern "C" void RTOS_Threads_Init(void) {
    imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
    vehicleTaskHandle = osThreadNew(vehicleTask, nullptr, &vehicleTaskAttribute);
    transformTaskHandle = osThreadNew(transformTask, nullptr, &transformTaskAttribute);
    suctionTaskHandle = osThreadNew(suctionTask, nullptr, &suctionTaskAttribute);
    communicationTaskHandle = osThreadNew(communicationTask, nullptr, &communicationTaskAttribute);
    selfTestTaskHandle = osThreadNew(selfTestTask, nullptr, &selfTestTaskAttribute);
    ledTaskHandle = osThreadNew(ledTask, nullptr, &ledTaskAttribute);

    // Set task handles for callbacks
    if (g_imu_controller) {
        g_imu_controller->SetTaskHandle(imuTaskHandle);
    }

    if (g_comm_controller) {
        g_comm_controller->SetTaskHandle(communicationTaskHandle);
    }
}

//==================================================================================================
// RTOS Default Task
//==================================================================================================

extern "C" void RTOS_Default_Task(const void* args) {
    UNUSED(args);

    while (true) {
        if (g_state_manager) {
            g_state_manager->UpdateControlModes();

            if (g_state_manager->IsInEmergencyStop() && g_vehicle_controller) {
                g_vehicle_controller->EmergencyStop();
            }
        }

        osDelay(100);
    }
}