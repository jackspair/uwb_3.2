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
#define AP_UWB_LORA 115200
#define AP_UWB_BUFSIZE_RX 512
#define AP_UWB_BUFSIZE_TX 128

#define BASESTA_ALT_M 0.0
#define HOME_SET_COUNT 10

#include "AP_UWB.h"

#include "string.h"
#include "stdio.h"
#include "stdarg.h"


#define BC_DIS_CM 150

extern const AP_HAL::HAL& hal;

// constructor
AP_UWB::AP_UWB(void) {
    _port_uwb = NULL;
    _port_Lora = NULL;

    reset_uwb_system();
}

void AP_UWB::reset_uwb_system()
{
    _dis_ab = 0;
    _dis_ac = 0;
    _dis_bc = BC_DIS_CM;

    _dis_EN = false;
    _home_is_set = false;
    _dis_EN_step = 0;
    last_frame_ms = 0;
    uwb_PS.uwb_PS = {0};
}

void AP_UWB::rst_uwb() 
{
    uint8_t rst_buff[7] = {0x00, 0x01, 0x03, 0xf9, 0x9f, 0xff, 0x99};
    _port_uwb->write(rst_buff+3, 4);
    _port_Lora->write(rst_buff, 7);
    rst_buff[1] = 0x02;
    _port_Lora->write(rst_buff, 7);
    reset_uwb_system();
}

void AP_UWB::init(const AP_SerialManager& serial_manager) {
    // check for DEVO_DPort
    if ((_port_uwb = serial_manager.find_serial(
             AP_SerialManager::SerialProtocol_UWB, 0))) {
        _port_uwb->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
        _port_uwb->begin(AP_UWB_BAUD, AP_UWB_BUFSIZE_RX, AP_UWB_BUFSIZE_TX);
    }
    if ((_port_Lora = serial_manager.find_serial(
             AP_SerialManager::SerialProtocol_LORA, 0))) {
        _port_Lora->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
        _port_Lora->begin(AP_UWB_BAUD, AP_UWB_BUFSIZE_RX, AP_UWB_BUFSIZE_TX);
    }
}

/**
 * @brief UWB���ݸ���
 * 
 * @param alt �߶�������Դ
 * @return true 
 * @return false 
 */
bool AP_UWB::update(int32_t alt) { //�߶���Դ��ѹ��
    if (_port_uwb == NULL) return false;

    uint16_t num_cnt = _port_uwb->available();  //��ȡ����������
    if (num_cnt == 0)  return false;

    if (_dis_EN == false)   return false;
    uint8_t data_temp, step = 0;
    uint8_t data_buff[20];
    int i = 0;
    for ( i = 0; i < num_cnt; i++)
    {
        if(step == 0)   data_temp = _port_uwb->read();
        switch (step)
        {
        case 0:
            if(data_temp == 0xf6)
            {
                step = 1;
                _port_uwb->read(data_buff, 9);
            }
            break;
        case 1:
            if(location_calculate(data_buff) == true)
            {
                if(uwb_PS.loc_pos(_dis_na_cm, _dis_nb_cm, _dis_nc_cm) == 0)
                {
                    
                    static int x= 0;
                    // static int y = 0;
                    if(_home_is_set == false && ++x >= 10)
                    {
                        x= 0;
                        _home_uwb.x = uwb_PS.copter_uwb.loc_cm.x;
                        _home_uwb.y = -uwb_PS.copter_uwb.loc_cm.y;
                        _home_uwb.z = -uwb_PS.copter_uwb.loc_cm.z;
                        _home_is_set = true;
                        printf("-------home is set -----------\r\n\
-------home:x:%f, y:%f, z:%f-----------\r\n", _home_uwb.x, _home_uwb.y, _home_uwb.z);
                    }
                    // if(_home_is_set == true && ++y >= 10)
                    // {
                    //     y= 0;
                    //     UWB_PS::POINT_POS copter_uwb = uwb_PS.uwb_PS_get_copter();
                    //     // _loc_NED.x = copter_uwb.loc_cm.x - _home_uwb.x;
                    //     // _loc_NED.y = -copter_uwb.loc_cm.y - _home_uwb.y;
                    //     // _loc_NED.x = -copter_uwb.loc_cm.z - _home_uwb.z;
                    //     printf("������λ������:x:%f, y:%f, z:%f\r\n", copter_uwb.loc_cm.x - _home_uwb.x,-copter_uwb.loc_cm.y - _home_uwb.y, -copter_uwb.loc_cm.z - _home_uwb.z);
                    // }
                    return true;
                }
                else
                {
                    printf("-----------uwb_location failure  dis_a:%d, dis_b:%d, dis_c:%d---------\r\n",_dis_na_cm, _dis_nb_cm, _dis_nc_cm);
                }
                
                return false;
            }
            return false;
            break;       
        default:
            break;
        }
    }
    return false;
}

/**
 * @brief ����ϵ�������վabc�����˻����ڵ�ľ������ݴ���
 * 
 * @param data 
 * @return true 
 * @return false 
 */
bool AP_UWB::location_calculate(uint8_t* data)
{
    if(*data != 0x6f && *(data+8) != 0x66)  return false;
    uint8_t check = ((*(data+2))&0xff) + ((*(data+4))&0xff) + ((*(data+6))&0xff);
    if(check != *(data+7))  return false;
    _dis_na_cm = (*(data+1)&0xff)<<8 | (*(data+2)&0xff);
    _dis_nb_cm = (*(data+3)&0xff)<<8 | (*(data+4)&0xff);
    _dis_nc_cm = (*(data+5)&0xff)<<8 | (*(data+6)&0xff);
    // printf("check:%d,na:%d,nb:%d,nc:%d\r\n", check, _dis_na_cm, _dis_nb_cm, _dis_nc_cm);
    return true;
}

/**
 * @brief ����loraЭ������
 * 
 * @return true 
 * @return false 
 */
bool AP_UWB::update() 
{ 
    if (_port_Lora == NULL) return false;

    uint16_t num_cnt = _port_Lora->available();  //��ȡ����������
    if (num_cnt == 0)  return false;

    uint8_t data_temp;
    uint8_t data_buff[20] = {0};
    int i = 0, step = 0, baseSta = 0;
    for ( i = 0; i < num_cnt; i++)
    {
        if(step == 0) data_temp = _port_Lora->read();
        switch (step)
        {
        case 0 :
            if(data_temp == 0xf4) //˫ģ��վ1
            {
                step = 1;
                _port_Lora->read(data_buff, 4);
            }   
            if(data_temp == 0xf7) //˫ģ��վ2 
            {
                step = 2;
                _port_Lora->read(data_buff, 4);
            }
            if(data_temp == 0xf9)
            {
                _port_Lora->read(data_buff,3);
                step = 4;
            }
            break;
        case 1 :
            baseSta = BASESTA_B;
            step = 3;
            break;
        case 2 :
            baseSta = BASESTA_C;
            step = 3;
            break;
        case 3:
            if(distance_calculate(data_buff, baseSta) == false) return false;  //��վ������֡β�Ƿ��쳣
            uwb_send2baseSta(baseSta);   //���ظ���վ�ѽ��յ���������
            return true;
            break;
        case 4 :
            if(data_buff[0] == 0x9f && data_buff[1] == 0xff && data_buff[2] == 0x99)
            {
                rst_uwb();
            }
            return true;
            break;
        default:
            break;
        }
    }
    return false;
}

/**
 * @brief ����˫ģ��վ���׼���ź�
 * @param num ��վ��
 */
void AP_UWB::send_range_cmd(int num)
{
    if(num == BASESTA_B)
    {
        uint8_t data[8] = {0x00, 0x01, 0x03, 0xf3,0x3f,0xff,0xff,0x33};
        _port_Lora->write(data, 8);
    }
    if(num == BASESTA_C)
    {
        uint8_t data[8] = {0x00, 0x02, 0x03, 0xf8,0x8f,0xff,0xff,0x88};
        _port_Lora->write(data, 8);
    }
}

/**
 * @brief ��վ�������֤
 * 
 * @param data ��վ��������ָ��
 * @return true 
 * @return false 
 */
bool AP_UWB::distance_calculate(uint8_t* data, int baseSta) {
    uint8_t flag = 0;
    if ((baseSta == BASESTA_B && *data == 0x4f && *(data + 3) == 0x44) ||
        (baseSta == BASESTA_C && *data == 0x7f && *(data + 3) == 0x77))
        flag = baseSta;
    uint16_t d = (*(data+1) & 0xff) << 8 | (*(data + 2) & 0xff); //��վ���������
    if(flag == 1) 
    {
        _dis_ab = d;
        _dis_EN_step = 1;
    } 
    else if(flag == 2) 
    {
        _dis_ac = d;
        _dis_EN_step = 0;
        _dis_EN = true;
    } 
    else   return false;
    return true;
}

/**
 * @brief ���ͻ�վ�����Ӧ����Ϣ
 */
void AP_UWB::uwb_send2baseSta(int num)
{
    if(num == BASESTA_B)
    {
        uint8_t tx_buff[8] = {0x00, 0x01, 0x03, 0xf3,0x3f,0,0,0x33};
        tx_buff[5] = _dis_ab/256;
        tx_buff[6] = _dis_ab%256;
        _port_Lora->write(tx_buff, 8);
    }
    if(num == BASESTA_C)
    {
        uint8_t tx_buff[8] = {0x00, 0x02, 0x03, 0xf8,0x8f,0,0,0x88};
        tx_buff[5] = _dis_ac/256;
        tx_buff[6] = _dis_ac%256;
        _port_Lora->write(tx_buff, 8);
        if(uwb_PS.loc_init(_dis_ab, _dis_ac, _dis_bc) == 0)
        {
            uint8_t start_lable_buff[4] = {0xf5, 0x5f, 0xff, 0x55};
            _port_uwb->write(start_lable_buff, 4);
            printf("\r\n----------UWB_SYS_init succeed---------------\r\n");
            // uint8_t buff1[] = {0x00,0x10,0x03,0x33,0x32,0x31};
            // _port_Lora->write(buff1, 6);
            // printf("ab:%d,ac:%d,bc:%d\r\n", _dis_ab, _dis_ac, _dis_bc);
        }
        else
        {
            printf("\r\n--------------UWB_SYS_init failure, ab:%d, ac:%d, bc:%d\r\n", _dis_ab, _dis_ac, _dis_bc);
            rst_uwb();
        }
    }
}


// /**
//  * @brief λ�ü���
//  * 
//  * @param data λ��ԭʼ����ָ��
//  * @param alt �߶�
//  * @return true 
//  * @return false 
//  */
// bool AP_UWB::location_calculate(uint8_t* data, int32_t alt) {
//     _loc_NED.x = x_m;
//     _loc_NED.y = -y_m;
//     _loc_NED.z = z_m;
//     last_frame_ms = AP_HAL::millis();
//     return true;
// }



/**
 * @brief ����ѡ����һ���վ����ǩ
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
    _port_uwb->write(tx_buff, 4);
}

//��ȡ���λ�ã�����x��Ϊ����y��Ϊ-����ģ��ɱ�������
bool AP_UWB::get_relative_position_NE_origin(Vector2f &posNE)
{
    if(get_dis_EN() == false)
        return false;
    if(_home_is_set == false)
        return false;
    posNE.x = uwb_PS.copter_uwb.loc_cm.x - _home_uwb.x;
    posNE.x = posNE.x/100.0;
    posNE.y = -uwb_PS.copter_uwb.loc_cm.y - _home_uwb.y; //Ĭ��ǰ��Ϊx�����Ϊy��������x����Ϊ����
    posNE.y = posNE.y/100.0;
    return true;
}

//��ȡz��
bool AP_UWB::get_relative_position_D_origin(float &posD)
{
    if (get_dis_EN() == false) return false;
    if(_home_is_set == false)
    return false;
    posD = uwb_PS.copter_uwb.loc_cm.z - _home_uwb.z;
    posD = posD/100.0;
    return true;
}

//��ʽ����ӡ
void AP_UWB::printf(const char *format, ...)
{
  va_list args;
  uint32_t length;
//   uint8_t TxBuffer[1024];
  uint8_t TxBuffer[1024] = {0xff,0xff,0x03};
  va_start(args, format);
  length = vsnprintf((char *)TxBuffer+3, 1021, (char *)format, args);
  va_end(args);
//   uint8_t buff[3] = {0x00,0x10,0x03};
//   _port_Lora->write(buff, 3);
  _port_Lora->write(TxBuffer, length+3);
}
