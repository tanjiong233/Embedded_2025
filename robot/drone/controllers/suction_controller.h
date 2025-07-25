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
#include "main.h"
#include "bsp_pwm.h"
#include "sbus.h"
#include "system_state_manager.h"
#include "../common/control_modes.h"
#include "../common/task_config.h"

//==================================================================================================
// Suction Control Class
//==================================================================================================

class SuctionController {
public:
    static constexpr uint32_t TASK_DELAY_MS = SUCTION_TASK_DELAY_MS;

    SuctionController() = default;

    void Init(TIM_HandleTypeDef* htim, SystemStateManager* state_manager, remote::SBUS* sbus);

    void Execute();

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

    static int16_t ApplyDeadzone(int16_t input, int16_t deadzone);
    static float MapSuctionToPulseWidth(int16_t sbus_input);
};