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

#define AP_UWB_BAUD 115200
#define AP_UWB_BUFSIZE_RX 512
#define AP_UWB_BUFSIZE_TX 128

#define BASESTA_ALT_M 0.0
#define HOME_SET_COUNT 10

#include "AP_UWB.h"

#include "string.h"
#include "stdio.h"
#include "stdarg.h"

extern const AP_HAL::HAL& hal;

// constructor
AP_UWB::AP_UWB(void) {
    _port = NULL;
    _port_Lora = NULL;
    _loc_NED = {-1,-1,-1};
    _dis_EN = false;
    _home_is_set = false;
    _home_set_buff = NULL;
    last_frame_ms = 0;
}

void AP_UWB::init(const AP_SerialManager& serial_manager) {
    // check for DEVO_DPort
    if ((_port = serial_manager.find_serial(
             AP_SerialManager::SerialProtocol_UWB, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
        _port->begin(AP_UWB_BAUD, AP_UWB_BUFSIZE_RX, AP_UWB_BUFSIZE_TX);
    }
    if ((_port_Lora = serial_manager.find_serial(
             AP_SerialManager::SerialProtocol_LORA, 0))) {
        _port_Lora->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
        _port_Lora->begin(AP_UWB_BAUD, AP_UWB_BUFSIZE_RX, AP_UWB_BUFSIZE_TX);
    }
}

/**
 * @brief UWB数据更新
 * 
 * @param alt 高度数据来源
 * @return true 
 * @return false 
 */
bool AP_UWB::update(int32_t alt) { //高度来源气压计
    if (_port == NULL) return false;
    uint16_t num_cnt = _port->available();  //读取缓存区数据
    if (num_cnt == 0)  return false;
    uint8_t data_buff[num_cnt];//定义数据缓存区
    int i = 0;
    _port->read(data_buff, num_cnt); //读取所有数据到缓存区
    // _port_Lora->write(0xff);
    // _port_Lora->write(data_buff, num_cnt);
    // _port_Lora->write(0xff);
    for (i = 0; i < num_cnt; i++) {
        if (data_buff[i] == 0xf6 && data_buff[i+1] == 0x6f)  //如果接收到帧头为标签到飞控
        {
            if(location_calculate(&data_buff[i + 2], alt) ==  false)  //计算校验和失败,返回失败
                return false;
            // printf("\r\nrx:x:%.2f,y:%.2f,z:%.2f\r\n", _loc_NED.x,  _loc_NED.y,  _loc_NED.z);
            if(!_home_is_set && get_dis_EN() && last_frame_ms != 0) //第一次获取到距离位置设置为Home位置
            {
                static int n = 0;
                if((get_location().x-0.1) >= 0 && (get_location().y-0.1) >= 0)
                {
                    if(++n == 100)
                    {
                        n = 0;
                        set_home_is_set();
                    }
                }
            }
            // printf("x:%f,y:%f,z:%f", _loc_NED.x,_loc_NED.y,_loc_NED.z);
            return true;
            break;
        }
    }
    _port_Lora->write(data_buff, num_cnt);
    return false;
}

/**
 * @brief 更新lora协议数据
 * 
 * @return true 
 * @return false 
 */
bool AP_UWB::update_lora() 
{ 
    if (_port_Lora == NULL) return false;
    uint16_t num_cnt = _port_Lora->available();  //读取缓存区数据
    if (num_cnt == 0)  return false;
    uint8_t data_buff[num_cnt];//定义数据缓存区
    _port_Lora->read(data_buff, num_cnt); //读取所有数据到缓存区
    int i = 0;
    for(i = 0; i < num_cnt; i++) {
            if (data_buff[i] == 0xf4 && data_buff[i+1] == 0x4f)  //如果接收到帧头为基站到飞控
            {
                if(distance_calculate(&data_buff[i+2]) == false) return false;  //基站间数据帧尾是否异常
                uwb_send2baseSta(get_dis_BS1_BS2_cm());   //返回给基站已接收到距离数据
                return true;
            }
    }
    _port_Lora->write(data_buff, num_cnt);
    return true;
}
/**
 * @brief 位置计算
 * 
 * @param data 位置原始数据指针
 * @param alt 高度
 * @return true 
 * @return false 
 */
bool AP_UWB::location_calculate(uint8_t* data, int32_t alt) {
    if (*(data + 5) != sterm::Lable2Flight) return false;  //帧尾不对
    if (get_dis_EN() == false) return false;    //基站间距离未确认

    uint16_t d1 = (*data & 0xff) << 8 | (*(data + 1) & 0xff), //基站0距离
             d2 = (*(data + 2) & 0xff) << 8 | (*(data + 3) & 0xff); //基站1距离
    if(d1 == 0)  return false;  //双模基站处于测距状态
    uint8_t check = *(data + 4);  
    if (check != ((d1 + d2) & 0xff)) return false;  //验证校验和
    /**
     * @brief 海伦公式
     */
    double p, s_m2, h_m, x_m, y_m, z_m = (double)alt/100 + BASESTA_ALT_M; //三角形面积，三角形高，得出的x坐标，y坐标，z轴数据

    p = (get_dis_BS1_BS2_cm()+d1+d2)/200.0;
    s_m2=sqrt((double)(p*(p-get_dis_BS1_BS2_cm()/100.0)*(p-d1/100.0)*(p-d2/100.0)));

    h_m = s_m2*2/((double)get_dis_BS1_BS2_cm()/100.0);  //三角形高度
    if(z_m <= 0.5) //与原点没有高度差 
    {
        y_m = sqrt(d1*d1/10000.0 - h_m*h_m);
        x_m = h_m;
    }
    else{  //有高度差
        y_m = sqrt(d1*d1/10000.0  - h_m*h_m);
        x_m = sqrt(h_m*h_m - z_m*z_m);
    }
    //验证是钝角三角形还是锐角三角形,(基站0夹角钝角时需要考虑)直角和基站1钝角可忽略 
    if (d2 * d2 / 10000.0 >
        (d1 * d1 / 10000.0 +
         get_dis_BS1_BS2_cm() * get_dis_BS1_BS2_cm() / 10000.0))
        y_m = -y_m;
    _loc_NED.x = x_m;
    _loc_NED.y = -y_m;
    _loc_NED.z = z_m;
    last_frame_ms = AP_HAL::millis();
    return true;
}

/**
 * @brief 基站间距离验证
 * 
 * @param data 基站距离数据指针
 * @return true 
 * @return false 
 */
bool AP_UWB::distance_calculate(uint8_t* data) {
    if (*(data + 2) != sterm::BaseStation2Flight) return false;

    uint16_t d = (*data & 0xff) << 8 | (*(data + 1) & 0xff); //基站间距离数据
    _dis_BS1_BS2_cm = d;
    set_dis_EN();   //设置已获取基站间数据

    return true;
}

/**
 * @brief 发送选用哪一组基站给标签
 * 
 * @param lable 
 */
void AP_UWB::uwb_send2lable(bool lable)
{
    uint8_t tx_buff[4] = {0xf5,0x5f,0,0x55};
    if(lable == true)
    {
        tx_buff[2] = 0x02;
    }
    if(lable == false)
    {
        tx_buff[2] = 0x01;
    }
    _port->write(tx_buff, 4);
}

/**
 * @brief 发送基站间距离应答消息
 * 
 * @param distance_cm 
 */
void AP_UWB::uwb_send2baseSta(uint16_t distance_cm)
{
    uint8_t tx_buff[5] = {0xf3,0x3f,0,0,0x33};
    tx_buff[2] = distance_cm/256;
    tx_buff[3] = distance_cm%256;
    _port_Lora->write(tx_buff, 5);
}


//获取相对位置，定义x轴为北，y轴为-东，模拟成北东坐标
bool AP_UWB::get_relative_position_NE_origin(Vector2f &posNE)
{
    if(get_dis_EN() == false)
        return false;
    posNE.x = _loc_NED.x - _home_uwb.x;
    posNE.y = _loc_NED.y - _home_uwb.y; //默认前进为x，左边为y，北东是x正轴为北，
    return true;
}

//获取z轴
bool AP_UWB::get_relative_position_D_origin(float &posD)
{
    if (get_dis_EN() == false) return false;
    posD = _loc_NED.z;
    return true;
}

void AP_UWB::send_range_cmd()
{
    uint8_t data[5] = {0xf3,0x3f,0xff,0xff,0x33};
    _port_Lora->write(data, 5);
}

//格式化打印
void AP_UWB::printf(const char *format, ...)
{
va_list args;
  uint32_t length;
  uint8_t TxBuffer[1024];
  va_start(args, format);
  length = vsnprintf((char *)TxBuffer, 1024, (char *)format, args);
  va_end(args);
  _port_Lora->write(TxBuffer, length);
}
