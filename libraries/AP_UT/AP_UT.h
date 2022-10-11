/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_UT {
public:
    AP_UT();

    /* Do not allow copies */
    AP_UT(const AP_UT &other) = delete;
    AP_UT &operator=(const AP_UT&) = delete;

    //初始化串口
    void init(const AP_SerialManager& serial_manager); 
    //更新数据
    bool update();

    uint16_t get_distance() { return _distance;}

private:
    AP_HAL::UARTDriver *_port_UT;              // UART used to send data to receiver
    uint16_t _distance;
};
