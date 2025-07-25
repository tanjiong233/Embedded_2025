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

#include "suction_controller.h"
#include "cmsis_os.h"

void SuctionController::Init(TIM_HandleTypeDef* htim, SystemStateManager* state_manager, remote::SBUS* sbus) {
    state_manager_ = state_manager;
    sbus_ = sbus;

    suction_fan_ = std::make_unique<bsp::PWM>(
            htim, 1, FAN_TIMER_CLOCK, FAN_PWM_FREQUENCY, MIN_FAN_PULSE_WIDTH);

    suction_fan_->Start();
    suction_fan_->SetPulseWidth(MIN_FAN_PULSE_WIDTH);
}

void SuctionController::Execute() {
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

int16_t SuctionController::ApplyDeadzone(int16_t input, int16_t deadzone) {
    return (abs(input) < deadzone) ? 0 : input;
}

float SuctionController::MapSuctionToPulseWidth(int16_t sbus_input) {
    if (sbus_input <= 0) {
        return MIN_FAN_PULSE_WIDTH;
    }

    float normalized_input = static_cast<float>(sbus_input) / SBUS_MAX_VALUE;
    if (normalized_input > 1.0f) normalized_input = 1.0f;

    return MIN_FAN_PULSE_WIDTH + (normalized_input * (MAX_FAN_PULSE_WIDTH - MIN_FAN_PULSE_WIDTH));
}