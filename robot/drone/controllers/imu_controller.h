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
#include "cmsis_os.h"
#include "bsp_imu.h"
#include "../common/task_config.h"

//==================================================================================================
// IMU Controller Class
//==================================================================================================

class IMUController {
public:
    static constexpr uint32_t TASK_DELAY_MS = IMU_TASK_DELAY_MS;
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

    void Init(const bsp::IMU_typeC_init_t& imu_init);

    void SetTaskHandle(osThreadId_t handle);

    void Execute();

    CustomIMU* GetIMU() { return imu_.get(); }

private:
    std::unique_ptr<CustomIMU> imu_;
};