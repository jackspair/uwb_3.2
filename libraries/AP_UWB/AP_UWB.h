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

class AP_UWB {
public:
    AP_UWB();

    /* Do not allow copies */
    AP_UWB(const AP_UWB &other) = delete;
    AP_UWB &operator=(const AP_UWB&) = delete;

    //��ʼ��UWB����
    void init(const AP_SerialManager& serial_manager); 
    //���ý��յ���վ�����
    void set_dis_EN() {_dis_EN = true;}
    //��ȡ���յ���վ����״̬
    bool get_dis_EN() {return _dis_EN;}
    //���õ�ǰλ��Ϊ��λ��
    void set_home_is_set() {_home_is_set = true; _home_uwb = _loc_NED;}
    //��ȡ��λ���Ƿ�����
    bool get_home_is_set() {return _home_is_set;}
    //��ȡ��վ�����
    uint16_t get_dis_BS1_BS2_cm() {return _dis_BS1_BS2_cm;}
    //����UWB����
    bool update(int32_t alt);      
    //��ȡλ������NED
    Vector3f get_location(void) {return _loc_NED;} 
    //���͸���ǩ����ָ��λ�ã��л���λ��վ
    void uwb_send2lable(bool lable);
    //���ͻ�վ�ѻ�ȡ��λ����
    void uwb_send2baseSta(uint16_t distance_cm);
    //��ȡ���λ��NE����
    bool get_relative_position_NE_origin(Vector2f &posNE) ;
    //��ȡ���λ��D����
    bool get_relative_position_D_origin(float &posD) ;
    //���͸���վ�����Ϣ
    void send_range_cmd();
    //��ʽ�����
    void printf(const char *format, ...);
    void print(const char* str) {_port->write(str);} 

    // uint32_t last_frame_ms;

    enum sterm {
        Lable2Flight = 0x66,
        Flight2Lable = 0x55,
        BaseStation2Flight = 0x44,
        Flight2BaseStation = 0x33,
        Flight2PC = 0x22,
    };
    

private:
    AP_HAL::UARTDriver *_port;              // UART used to send data to receiver
    Vector3f _loc_NED;
    Vector3f _home_uwb;
    bool _home_is_set;
    uint16_t _dis_BS1_BS2_cm;
    bool _dis_EN;

    //λ�ü��� 
    bool location_calculate(uint8_t* data , int32_t alt);
    //�������
    bool distance_calculate(uint8_t* data);
};
