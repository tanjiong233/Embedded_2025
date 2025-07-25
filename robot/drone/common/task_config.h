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

#include <cstdint>
#include "cmsis_os.h"

//==================================================================================================
// Task Configuration Constants
//==================================================================================================

// Task delay times (milliseconds)
static constexpr uint32_t VEHICLE_TASK_DELAY_MS = 2;
static constexpr uint32_t TRANSFORM_TASK_DELAY_MS = 10;
static constexpr uint32_t SUCTION_TASK_DELAY_MS = 20;
static constexpr uint32_t IMU_TASK_DELAY_MS = 1;
static constexpr uint32_t SELFTEST_TASK_DELAY_MS = 100;
static constexpr uint32_t LED_TASK_DELAY_MS = 1;
static constexpr uint32_t COMMUNICATION_TASK_DELAY_MS = 50;
static constexpr uint32_t DEFAULT_TASK_DELAY_MS = 100;

// Task stack sizes
static constexpr uint32_t TASK_STACK_SIZE = 256 * 4;

// Task attributes template
#define DECLARE_TASK_ATTRIBUTES(name) \
    extern const osThreadAttr_t name##TaskAttribute; \
    extern osThreadId_t name##TaskHandle;

// Task function declarations
extern "C" {
    void vehicleTask(void* arg);
    void transformTask(void* arg);
    void suctionTask(void* arg);
    void imuTask(void* arg);
    void selfTestTask(void* arg);
    void communicationTask(void* arg);
    void ledTask(void* arg);
}

// Declare all task attributes and handles
DECLARE_TASK_ATTRIBUTES(vehicle)
DECLARE_TASK_ATTRIBUTES(transform)
DECLARE_TASK_ATTRIBUTES(suction)
DECLARE_TASK_ATTRIBUTES(imu)
DECLARE_TASK_ATTRIBUTES(selfTest)
DECLARE_TASK_ATTRIBUTES(communication)
DECLARE_TASK_ATTRIBUTES(led)