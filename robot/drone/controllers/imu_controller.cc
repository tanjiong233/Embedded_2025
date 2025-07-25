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

#include "imu_controller.h"
#include "cmsis_os.h"

void IMUController::Init(const bsp::IMU_typeC_init_t& imu_init) {
    imu_ = std::make_unique<CustomIMU>(imu_init, false);
    if (imu_) {
        imu_->Calibrate();
    }
}

void IMUController::SetTaskHandle(osThreadId_t handle) {
    if (imu_) {
        imu_->SetTaskHandle(handle);
    }
}

void IMUController::Execute() {
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL && imu_) {
        imu_->Update();
    }
    osDelay(TASK_DELAY_MS);
}