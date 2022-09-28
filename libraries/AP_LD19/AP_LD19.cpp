/*

   Inspired by work done here
   https://github.com/PX4/Firmware/tree/master/src/drivers/frsky_telemetry from Stefan Rado <px4@sradonia.net>
   https://github.com/opentx/opentx/tree/2.3/radio/src/telemetry from the OpenTX team

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
   FRSKY Telemetry library
*/

#define AP_SERIALMANAGER_LD19_BAUD    230400
#define AP_SERIALMANAGER_LD19_BUFSIZE_RX    470   //十个帧数据大小
#define AP_SERIALMANAGER_LD19_BUFSIZE_TX    128

#include "AP_LD19.h"
#include <AP_SerialManager/AP_SerialManager.h>

static const uint8_t CrcTable[256] =
{
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
    0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
    0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
    0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
    0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
    0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
    0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
    0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
    0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
    0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
    0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
    0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
    0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
    0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
    0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
    0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
    0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
    0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
    0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
    0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
    0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
    0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};//用于crc校验的数组

extern const AP_HAL::HAL& hal;

//构造函数
AP_LD19::AP_LD19(void)
{
    _port =NULL;  //串口为空
}

/*
 * init - perform required initialisation
 */
void AP_LD19::init(const AP_SerialManager& serial_manager)
{
    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    //如果找到一个类别为ld19的串口且不为空，则赋给port
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_LD19, 0))) {
        //关掉串口流控
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        //初始化串口
        _port->begin(AP_SERIALMANAGER_LD19_BAUD,AP_SERIALMANAGER_LD19_BUFSIZE_RX,AP_SERIALMANAGER_LD19_BUFSIZE_TX);
    } 
}

bool AP_LD19::update()
{
    static uint8_t state = 0;//状态位
	static uint8_t crc = 0;//校验和
	static uint8_t cnt = 0;//用于一帧12个点的计数
	uint8_t temp_data;

    if(_port == NULL)  return false;
    int16_t numc = _port->available();
    uint8_t Data_Buff[numc];
    _port->read(Data_Buff, numc);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "NUM:%d",numc); 
    if((numc%47 == 0) && (numc != 0)) //一个完整帧的数据为47位
    {
        for(uint8_t j=0;j<numc/47;j++)
        {
            for(uint16_t i=0;i<47;i++)
            {
                temp_data=Data_Buff[i+j*47];
                if (state > 5)
                {
                    if(state < 42)
                    {
                        if(state%3 == 0)//6，9...表示数据里距离低八位的值
                        {
                            Pack_Data.point[cnt].distance = (uint16_t)temp_data;
                            state++;
                            crc = CrcTable[(crc^temp_data) & 0xff];
                        }
                        else if(state%3 == 1)//7，10...表示数据里距离高八位的值
                        {
                            Pack_Data.point[cnt].distance = ((uint16_t)temp_data<<8)+Pack_Data.point[cnt].distance;
                            state++;
                            crc = CrcTable[(crc^temp_data) & 0xff];
                        }
                        else//8，11...表示置信度
                        {
                            Pack_Data.point[cnt].confidence = temp_data;
                            cnt++;	
                            state++;
                            crc = CrcTable[(crc^temp_data) & 0xff];
                        }
                    }
                    else 
                    {
                        switch(state)
                        {
                            case 42:
                                Pack_Data.end_angle = (uint16_t)temp_data;//结束角度低八位
                                state++;
                                crc = CrcTable[(crc^temp_data) & 0xff];
                                break;
                            case 43:
                                Pack_Data.end_angle = ((uint16_t)temp_data<<8)+Pack_Data.end_angle;//结束角度高八位
                                state++;
                                crc = CrcTable[(crc^temp_data) & 0xff];
                                break;
                            case 44:
                                Pack_Data.timestamp = (uint16_t)temp_data;//时间戳低八位
                                state++;
                                crc = CrcTable[(crc^temp_data) & 0xff];
                                break;
                            case 45:
                                Pack_Data.timestamp = ((uint16_t)temp_data<<8)+Pack_Data.timestamp;//时间戳高八位
                                state++;
                                crc = CrcTable[(crc^temp_data) & 0xff];
                                break;
                            case 46:
                                Pack_Data.crc8 = temp_data;//雷达传来的校验和
                                if(Pack_Data.crc8 == crc)//和自己的校验比较正确
                                {
                                    LD19_Data_Process(SAFE_DISTANCE);
                                    //receive_cnt++;
                                    gcs().send_text(MAV_SEVERITY_CRITICAL, "success one date"); 
                                    last_frame_time_ms = AP_HAL::millis();
                                    crc = 0;
                                    state = 0;
                                    cnt = 0;
                                    if(j == numc/47-1)
                                    {
                                        return true;  //等所有数据都解析完再返回
                                    }
                                }
                                else
                                memset(&Pack_Data,0,sizeof(Pack_Data));//清零
                                crc = 0;
                                state = 0;
                                cnt = 0;//复位
                            default: break;
                        }
                    }
                }
                else 
                {
                    switch(state)
                    {
                        case 0:
                            if(temp_data == HEADER)//帧头
                            {
                                Pack_Data.header = temp_data;
                                state++;
                                crc = CrcTable[(crc^temp_data) & 0xff];//进行校验
                            } else state = 0,crc = 0;
                            break;
                        case 1:
                            if(temp_data == LENGTH)//测量的点数，固定12个
                            {
                                Pack_Data.ver_len = temp_data;
                                state++;
                                crc = CrcTable[(crc^temp_data) & 0xff];
                            } else state = 0,crc = 0;
                            break;
                        case 2:
                            Pack_Data.speed = (uint16_t)temp_data;//雷达转速低八位在，单位度每秒
                            state++;
                            crc = CrcTable[(crc^temp_data) & 0xff];
                            break;
                        case 3:
                            Pack_Data.speed = ((uint16_t)temp_data<<8)+Pack_Data.speed;//雷达转速高八位
                            state++;
                            crc = CrcTable[(crc^temp_data) & 0xff];
                            break;
                        case 4:
                            Pack_Data.start_angle = (uint16_t)temp_data;//开始角度低八位
                            state++;
                            crc = CrcTable[(crc^temp_data) & 0xff];
                            break;
                        case 5:
                            Pack_Data.start_angle = ((uint16_t)temp_data<<8)+Pack_Data.start_angle;
                            state++;
                            crc = CrcTable[(crc^temp_data) & 0xff];
                            break;
                        default: break;

                    }
                }
            }
        }
        return false;
    }
    else
    {
        return false;
    }
}


//功能：接收12个点的数据处理
//参数: 安全距离(mm)
void AP_LD19::LD19_Data_Process(uint16_t Safe_Distance)
{
    for(uint8_t i=0;i<12;i++)
    {
        if(Pack_Data.point[i].distance < Safe_Distance)
        {
            //判断此时角度值(单位是0.01度)
            if(Pack_Data.start_angle > Pack_Data.end_angle)
            {
                Pack_Data.end_angle+=36000;
            }
            res.Step = (Pack_Data.end_angle - Pack_Data.start_angle)/(12 - 1);
            res.Danger_Angle = (Pack_Data.start_angle + res.Step*i)/100.0; 
            res.Danger_Distance = (Pack_Data.point[i].distance)/10.0;
            //将角度值和危险距离值发出
            
        }
    }
}

