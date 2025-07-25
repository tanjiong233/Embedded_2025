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