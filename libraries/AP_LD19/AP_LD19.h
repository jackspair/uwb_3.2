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

#define ANGLE_PER_FRAME 12
#define HEADER 0x54
#define POINT_PER_PACK 12
#define LENGTH  0x2C 	

class AP_LD19 {

public:
    AP_LD19();

    /* Do not allow copies */
    AP_LD19(const AP_LD19 &other) = delete;
    AP_LD19 &operator=(const AP_LD19&) = delete;

    // init - perform required initialisation
    void init(const AP_SerialManager& serial_manager);

    bool update(void); 

    typedef struct __attribute__((packed)) Point_Data
    {
        uint16_t distance;//距离
        uint8_t confidence;//置信度
        
    }LidarPointStructDef;

    typedef struct __attribute__((packed)) Pack_Data
    {
        uint8_t header;
        uint8_t ver_len;
        uint16_t speed;
        uint16_t start_angle;
        LidarPointStructDef point[POINT_PER_PACK];
        uint16_t end_angle;
        uint16_t timestamp;
        uint8_t crc8;
    }LiDARFrameTypeDef;

    LiDARFrameTypeDef Pack_Data; //接收到的数据

    uint32_t last_frame_time_ms;

private:
    AP_HAL::UARTDriver *_port;

};
