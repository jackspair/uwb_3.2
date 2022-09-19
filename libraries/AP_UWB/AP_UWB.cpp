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
#define AP_UWB_BUFSIZE_RX 128
#define AP_UWB_BUFSIZE_TX 128

#define BASESTA_ALT_M 0.0

#include "AP_UWB.h"

#include "string.h"
#include "stdio.h"
#include "stdarg.h"

extern const AP_HAL::HAL& hal;

// constructor
AP_UWB::AP_UWB(void) {
    _port = NULL;
    _dis_EN = false;
    _home_is_set = false;
}

void AP_UWB::init(const AP_SerialManager& serial_manager) {
    // check for DEVO_DPort
    if ((_port = serial_manager.find_serial(
             AP_SerialManager::SerialProtocol_UWB, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
        _port->begin(AP_UWB_BAUD, AP_UWB_BUFSIZE_RX, AP_UWB_BUFSIZE_TX);
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
    if (_port == NULL) return false;
    uint16_t num_cnt = _port->available();  //��ȡ����������
    if (num_cnt == 0)  return false;
    uint8_t data_buff[num_cnt];//�������ݻ�����
    int i = 0;
    _port->read(data_buff, num_cnt); //��ȡ�������ݵ�������
    for (i = 0; i < num_cnt; i++) {
        if (data_buff[i] == 0xf6 && data_buff[i+1] == 0x6f)  //������յ�֡ͷΪ��ǩ���ɿ�
        {
            if (location_calculate(&data_buff[i + 2], alt) ==  false)  //����У���ʧ��,����ʧ��
                return false;
            // printf("x:%f,y:%f,z:%f", _loc_NED.x,_loc_NED.y,_loc_NED.z);
            return true;
            break;
        }
        if (data_buff[i] == 0xf4 && data_buff[i+1] == 0x4f)  //������յ�֡ͷΪ��վ���ɿ�
        {
            if(distance_calculate(&data_buff[i+2]) == false) return false;  //��վ������֡β�Ƿ��쳣
            uwb_send2baseSta(get_dis_BS1_BS2_cm());   //���ظ���վ�ѽ��յ���������
            return true;
        }
    }
    _port->write(data_buff, num_cnt);
    return false;
}

/**
 * @brief λ�ü���
 * 
 * @param data λ��ԭʼ����ָ��
 * @param alt �߶�
 * @return true 
 * @return false 
 */
bool AP_UWB::location_calculate(uint8_t* data, int32_t alt) {
    if (*(data + 5) != sterm::Lable2Flight) return false;  //֡β����
    if (get_dis_EN() == false) return false;    //��վ�����δȷ��

    uint16_t d1 = (*data & 0xff) << 8 | (*(data + 1) & 0xff), //��վ0����
             d2 = (*(data + 2) & 0xff) << 8 | (*(data + 3) & 0xff); //��վ1����
    uint8_t check = *(data + 4);  
    if (check != ((d1 + d2) & 0xff)) return false;  //��֤У���
    /**
     * @brief ���׹�ʽ
     */
    double p, s_m2, h_m, x_m, y_m, z_m =  BASESTA_ALT_M; //����������������θߣ��ó���x���꣬y���꣬z������

    p = (get_dis_BS1_BS2_cm()+d1+d2)/200.0;
    s_m2=sqrt((double)(p*(p-get_dis_BS1_BS2_cm()/100.0)*(p-d1/100.0)*(p-d2/100.0)));

    h_m = s_m2*2/((double)get_dis_BS1_BS2_cm()/100.0);  //�����θ߶�
    if(z_m <= 0.5) //��ԭ��û�и߶Ȳ� 
    {
        y_m = sqrt(d1*d1/10000.0 - h_m*h_m);
        x_m = h_m;
    }
    else{  //�и߶Ȳ�
        y_m = sqrt(d1*d1/10000.0  - h_m*h_m);
        x_m = sqrt(h_m*h_m - z_m*z_m);
    }
    //��֤�Ƕ۽������λ������������,(��վ0�нǶ۽�ʱ��Ҫ����)ֱ�Ǻͻ�վ1�۽ǿɺ��� 
    if (d2 * d2 / 10000.0 >
        (d1 * d1 / 10000.0 +
         get_dis_BS1_BS2_cm() * get_dis_BS1_BS2_cm() / 10000.0))
        y_m = -y_m;
    _loc_NED.x = x_m;
    _loc_NED.y = y_m;
    _loc_NED.z = z_m;
    return true;
}

/**
 * @brief ��վ�������֤
 * 
 * @param data ��վ��������ָ��
 * @return true 
 * @return false 
 */
bool AP_UWB::distance_calculate(uint8_t* data) {
    if (*(data + 2) != sterm::BaseStation2Flight) return false;

    uint16_t d = (*data & 0xff) << 8 | (*(data + 1) & 0xff); //��վ���������
    _dis_BS1_BS2_cm = d;
    set_dis_EN();   //�����ѻ�ȡ��վ������

    return true;
}

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
    _port->write(tx_buff, 4);
}

/**
 * @brief ���ͻ�վ�����Ӧ����Ϣ
 * 
 * @param distance_cm 
 */
void AP_UWB::uwb_send2baseSta(uint16_t distance_cm)
{
    uint8_t tx_buff[5] = {0xf3,0x3f,0,0,0x33};
    tx_buff[2] = distance_cm/256;
    tx_buff[3] = distance_cm%256;
    _port->write(tx_buff, 5);
}


//��ȡ���λ�ã�����x��Ϊ����y��Ϊ-����ģ��ɱ�������
bool AP_UWB::get_relative_position_NE_origin(Vector2f &posNE)
{
    if(get_dis_EN() == false)
        return false;
    posNE.x = _loc_NED.x;
    posNE.y = -_loc_NED.y; //Ĭ��ǰ��Ϊx�����Ϊy��������x����Ϊ����
    return true;
}

//��ȡz��
bool AP_UWB::get_relative_position_D_origin(float &posD)
{
    if (get_dis_EN() == false) return false;
    posD = -_loc_NED.z;
    return true;
}

void AP_UWB::send_range_cmd()
{
    uint8_t data[5] = {0xf3,0x3f,0xff,0xff,0x33};
    _port->write(data, 5);
}

//��ʽ����ӡ
void AP_UWB::printf(const char *format, ...)
{
va_list args;
  uint32_t length;
  uint8_t TxBuffer[1024];
  va_start(args, format);
  length = vsnprintf((char *)TxBuffer, 1024, (char *)format, args);
  va_end(args);
  _port->write(TxBuffer, length);
}
