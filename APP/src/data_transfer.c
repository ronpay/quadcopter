#include "data_transfer.h"
#include "USART.h"

#define BYTE0(dwTemp) (*((char*)(&dwTemp)))
#define BYTE1(dwTemp) (*((char*)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char*)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char*)(&dwTemp) + 3))

u8 data_to_send[50];

void ANO_DT_Send_Data(u8* dataToSend, u8 length)
{
    Usart_Transmit(USART6, dataToSend, length);
}

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, uint8_t sta)
{
    u8   _cnt = 0;
    vs16 _temp;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xFF;
    data_to_send[_cnt++] = 0x03;
    data_to_send[_cnt++] = 0x07;

    _temp                = (angle_rol * 100);
    data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);
    _temp                = (angle_pit * 100);
    data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);
    _temp                = (angle_yaw * 100);
    data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);

    data_to_send[_cnt++] = BYTE0(sta);

    u8 sumcheck = 0;
    u8 addcheck = 0;
    for (u8 i = 0; i < data_to_send[3] + 4; i++) {
        sumcheck += data_to_send[i];
        addcheck += sumcheck;
    }
    data_to_send[_cnt++] = sumcheck;
    data_to_send[_cnt++] = addcheck;
    ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Senser(float a_x, float a_y, float a_z, float g_x, float g_y, float g_z, uint8_t sta)
{
    u8   _cnt = 0;
    vs16 _temp;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xFF;
    data_to_send[_cnt++] = 0x01;
    data_to_send[_cnt++] = 0x0D;

    _temp                = a_x;
    data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);
    _temp                = a_y;
    data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);
    _temp                = a_z;
    data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);

    _temp                = g_x;
    data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);
    _temp                = g_y;
    data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);
    _temp                = g_z;
    data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);

    data_to_send[_cnt++] = BYTE0(sta);

    u8 sumcheck = 0;
    u8 addcheck = 0;
    for (u8 i = 0; i < _cnt; i++) {
        sumcheck += data_to_send[i];
        addcheck += sumcheck;
    }
    data_to_send[_cnt++] = sumcheck;
    data_to_send[_cnt++] = addcheck;
    ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Senser2(float m_x, float m_y, float m_z, int32_t alt, int16_t tmp, uint8_t bar_sta, uint8_t mag_sta)
{
    u8   _cnt = 0;
    vs16 _temp;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xFF;
    data_to_send[_cnt++] = 0x02;
    data_to_send[_cnt++] = 0x0E;

    _temp                = m_x;
    data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);
    _temp                = m_y;
    data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);
    _temp                = m_z;
    data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);

    _temp                = alt;
    data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);
    _temp                = tmp;
    data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);

    data_to_send[_cnt++] = BYTE0(bar_sta);

    data_to_send[_cnt++] = BYTE0(mag_sta);

    u8 sumcheck = 0;
    u8 addcheck = 0;
    for (u8 i = 0; i < _cnt; i++) {
        sumcheck += data_to_send[i];
        addcheck += sumcheck;
    }
    data_to_send[_cnt++] = sumcheck;
    data_to_send[_cnt++] = addcheck;
    ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_PWM(uint16_t PWM1, uint16_t PWM2, uint16_t PWM3, uint16_t PWM4)
{
    u8   _cnt = 0;
    vs16 _temp;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xFF;
    data_to_send[_cnt++] = 0x20;
    data_to_send[_cnt++] = 0x08;

    _temp                = PWM1;
    data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);
    _temp                = PWM2;
    data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);
    _temp                = PWM3;
    data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);
    _temp                = PWM4;
    data_to_send[_cnt++] = BYTE0(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);

    u8 sumcheck = 0;
    u8 addcheck = 0;
    for (u8 i = 0; i < _cnt; i++) {
        sumcheck += data_to_send[i];
        addcheck += sumcheck;
    }
    data_to_send[_cnt++] = sumcheck;
    data_to_send[_cnt++] = addcheck;
    ANO_DT_Send_Data(data_to_send, _cnt);
}
