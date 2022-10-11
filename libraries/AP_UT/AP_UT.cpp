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

/*
   UWB library
*/

#define AP_UT_BAUD 9600
#define AP_UT_BUFSIZE_RX 128
#define AP_UT_BUFSIZE_TX 128


#include "AP_UT.h"

extern const AP_HAL::HAL& hal;

// constructor
AP_UT::AP_UT(void) {
    _port_UT = NULL;
}

void AP_UT::init(const AP_SerialManager& serial_manager) {
    // check for DEVO_DPort
    if ((_port_UT = serial_manager.find_serial(
             AP_SerialManager::SerialProtocol_UT, 0))) {
        _port_UT->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
        _port_UT->begin(AP_UT_BAUD, AP_UT_BUFSIZE_RX, AP_UT_BUFSIZE_TX);
    }
}

bool AP_UT::update()
{
    if(!_port_UT)   return false;
    _port_UT->write("x");
    uint16_t cnt = _port_UT->available();
    if(!cnt)    return false;
    uint16_t i = 0, step = 0;
    uint8_t data_temp;
    uint8_t distance_H, distance_L, check;
    for ( i = 0; i < cnt; i++)
    {
        data_temp = _port_UT->read();
        switch (step)
        {
        case 0:
            if(data_temp == 0xff)
            {
                step = 1;
            } 
            break;
        case 1:
            distance_H = data_temp;
            step = 2;
            break;
        case 3:
            distance_L = data_temp;
            step = 3;
            break;
        case 4:
            check = data_temp;
            if(((0xff + distance_H + distance_L)&0xff) != check)
            {
                return false;
            }
            _distance = distance_H*256+distance_L;
            return true;
            break;
        default:
            break;
        }
    }
    return false;   
    
}
