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
#include "main.h"
#include "rgb.h"
#include "../common/task_config.h"

//==================================================================================================
// LED Controller Class
//==================================================================================================

class LEDController {
public:
    static constexpr uint32_t TASK_DELAY_MS = LED_TASK_DELAY_MS;
    static constexpr int RGB_FLOW_COLOR_CHANGE_TIME = 300;

    LEDController() = default;

    void Init(TIM_HandleTypeDef* htim);

    void Execute();

private:
    std::unique_ptr<display::RGB> led_;
    int color_index_ = 0;
};