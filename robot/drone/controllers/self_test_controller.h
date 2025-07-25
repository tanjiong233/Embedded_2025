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
#include "i2c.h"
#include "oled.h"
#include "bsp_buzzer.h"
#include "sbus.h"
#include "vehicle_controller.h"
#include "transform_controller.h"
#include "imu_controller.h"
#include "system_state_manager.h"
#include "../common/task_config.h"

//==================================================================================================
// Self Test Controller Class
//==================================================================================================

class SelfTestController {
public:
    static constexpr uint32_t TASK_DELAY_MS = SELFTEST_TASK_DELAY_MS;

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
              IMUController* imu_ctrl);

    void Execute();

    void SetComputerStatus(bool connected);

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

    void ShowStartupSequence();
    void UpdateDeviceStatus();
    void UpdateOLEDDisplay();
};