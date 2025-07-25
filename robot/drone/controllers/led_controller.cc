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

#include "led_controller.h"
#include "cmsis_os.h"

void LEDController::Init(TIM_HandleTypeDef* htim) {
    led_ = std::make_unique<display::RGB>(htim, 3, 2, 1, 1000000);
}

void LEDController::Execute() {
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