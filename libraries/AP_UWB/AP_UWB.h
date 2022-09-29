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
#define UWB_Hz  20  //uwb刷新数据频率
#define UWB_T  200  //uwb线程运行最大时间
#define BASESTA_B 1 //双模基站b
#define BASESTA_C 2 //c

#include "UWB_PS.h"

class AP_UWB {
public:
    AP_UWB();
    /* Do not allow copies */
    AP_UWB(const AP_UWB &other) = delete;
    AP_UWB &operator=(const AP_UWB&) = delete;

    //初始化UWB串口
    void init(const AP_SerialManager& serial_manager); 
    //获取坐标系是否建立
    bool get_dis_EN() {return _dis_EN;}
    //设置当前位置为家位置
    void set_home_is_set() {_home_is_set = true; _home_uwb = _location;}
    //获取家位置是否设置
    bool get_home_is_set() {return _home_is_set;}
    Vector3f get_home() {return _home_uwb;}
    //更新UWB 标签数据
    bool update(int32_t alt); 
    //更新UWB LoRa数据
    bool update(); 
    //发送给标签到达指定位置，切换定位基站
    void uwb_send2lable(bool lable);
    //发送基站已获取定位数据
    void uwb_send2baseSta(int num);
    //获取相对位置NE坐标
    bool get_relative_position_NE_origin(Vector2f &posNE) ;
    //获取相对位置D坐标
    bool get_relative_position_D_origin(float &posD) ;
    //发送给基站测距信息
    void send_range_cmd(int num);
    //格式化输出
    void printf(const char *format, ...);
    void print(const char* str) {_port_uwb->write(str);} 

    int get_dis_EN_step() { return _dis_EN_step; }

    // UWB_LOCATION::POINT_POS get_uwb_loc_pos() { return uwb_loc.get_n_pos();}

    uint32_t last_frame_ms;

    // UWB_LOCATION uwb_loc;
    UWB_PS uwb_PS;
    AP_HAL::UARTDriver *_port_Lora;              // UART used to send data to receiver
        uint32_t _dis_na_cm; //无人机与基站参考点a的距离(基站1)
    uint32_t _dis_nb_cm; //b(基站2)
    uint32_t _dis_nc_cm; //c(基站3)

private:
    AP_HAL::UARTDriver *_port_uwb;              // UART used to send data to receiver

    

    Vector3f _loc_NED;//当前位置北东地坐标

    Vector3f _location;//当前位置绝对坐标

    Vector3f _home_uwb;//家位置坐标

    bool _home_is_set;//家位置是否设置

    uint32_t _dis_ab;//参考点ab间距
    uint32_t _dis_ac;//ac
    uint32_t _dis_bc;//bc

    bool _dis_EN;//坐标系已建立

    int _dis_EN_step;//坐标系建立步骤



    //距离计算
    bool distance_calculate(uint8_t* data, int baseSta);

    //位置距离计算
    bool location_calculate(uint8_t* data);

    // bool update_uwb_loc() { return  uwb_loc.loc_pos(_dis_na_cm, _dis_nb_cm, _dis_nc_cm); }

};
