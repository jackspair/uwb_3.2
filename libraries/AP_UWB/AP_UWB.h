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

#define UWB_Hz  20  //uwbˢ������Ƶ��
#define UWB_T  200  //uwb�߳��������ʱ��
#define BASESTA_B 1 //˫ģ��վb
#define BASESTA_C 2 //c

#include "UWB_PS.h"

class AP_UWB {
public:
    AP_UWB();
    /* Do not allow copies */
    AP_UWB(const AP_UWB &other) = delete;
    AP_UWB &operator=(const AP_UWB&) = delete;

    //��ʼ��UWB����
    void init(const AP_SerialManager& serial_manager); 
    //��ȡ����ϵ�Ƿ���
    bool get_dis_EN() {return _dis_EN;}
    //���õ�ǰλ��Ϊ��λ��
    void set_home_is_set() {_home_is_set = true; _home_uwb = _location;}
    //��ȡ��λ���Ƿ�����
    bool get_home_is_set() {return _home_is_set;}
    Vector3f get_home() {return _home_uwb;}
    //����UWB ��ǩ����
    bool update(int32_t alt); 
    //����UWB LoRa����
    bool update(); 
    //���͸���ǩ����ָ��λ�ã��л���λ��վ
    void uwb_send2lable(bool lable);
    //���ͻ�վ�ѻ�ȡ��λ����
    void uwb_send2baseSta(int num);
    //��ȡ���λ��NE����
    bool get_relative_position_NE_origin(Vector2f &posNE) ;
    //��ȡ���λ��D����
    bool get_relative_position_D_origin(float &posD) ;
    //���͸���վ�����Ϣ
    void send_range_cmd(int num);
    //��ʽ�����
    void printf(const char *format, ...);
    void print(const char* str) {_port_uwb->write(str);} 

    int get_dis_EN_step() { return _dis_EN_step; }

    // UWB_LOCATION::POINT_POS get_uwb_loc_pos() { return uwb_loc.get_n_pos();}

    uint32_t last_frame_ms;

    // UWB_LOCATION uwb_loc;
    UWB_PS uwb_PS;
    AP_HAL::UARTDriver *_port_Lora;              // UART used to send data to receiver
        uint32_t _dis_na_cm; //���˻����վ�ο���a�ľ���(��վ1)
    uint32_t _dis_nb_cm; //b(��վ2)
    uint32_t _dis_nc_cm; //c(��վ3)

    void reset_uwb_system();

private:
    AP_HAL::UARTDriver *_port_uwb;              // UART used to send data to receiver

    

    Vector3f _loc_NED;//��ǰλ�ñ���������

    Vector3f _location;//��ǰλ�þ�������

    Vector3f _home_uwb;//��λ������

    bool _home_is_set;//��λ���Ƿ�����

    uint32_t _dis_ab;//�ο���ab���
    uint32_t _dis_ac;//ac
    uint32_t _dis_bc;//bc

    bool _dis_EN;//����ϵ�ѽ���

    int _dis_EN_step;//����ϵ��������



    //�������
    bool distance_calculate(uint8_t* data, int baseSta);

    //λ�þ������
    bool location_calculate(uint8_t* data);

    // bool update_uwb_loc() { return  uwb_loc.loc_pos(_dis_na_cm, _dis_nb_cm, _dis_nc_cm); }
    void rst_uwb();

};
