#include "data_transfer.h"
#include "USART.h"

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

u8 data_to_send[50];

void ANO_DT_Send_Data(u8 *dataToSend,u8 length){
    Usart_Transmit(USART6,dataToSend,length);
}

void ANO_DT_Send_Status(s16 angle_rol, s16 angle_pit, s16 angle_yaw, u8 sta){
    u8 _cnt = 0;
    vs16 _temp;
    

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xFF;
    data_to_send[_cnt++] = 0x03;
    data_to_send[_cnt++] = 0x07;

    _temp=(s16)(angle_rol*100);
    data_to_send[_cnt++]=BYTE0(_temp);
    data_to_send[_cnt++]=BYTE1(_temp);
    _temp=(s16)(angle_pit*100);
    data_to_send[_cnt++]=BYTE0(_temp);
    data_to_send[_cnt++]=BYTE1(_temp);
    _temp=(s16)(angle_yaw*100);
    data_to_send[_cnt++]=BYTE0(_temp);
    data_to_send[_cnt++]=BYTE1(_temp);

    data_to_send[_cnt++]=BYTE0(sta);

    u8 sumcheck = 0;
    u8 addcheck = 0;
    for(u8 i=0;i<data_to_send[3]+4;i++){
        sumcheck += data_to_send[i];
        addcheck += sumcheck;
    }
    data_to_send[_cnt++] = sumcheck;
    data_to_send[_cnt++] = addcheck;
    ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Senser(s16 a_x, s16 a_y, s16 a_z, s16 g_x, s16 g_y, s16 g_z, u8 sta){
    u8 _cnt = 0;
    vs16 _temp;
    

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xFF;
    data_to_send[_cnt++] = 0x01;
    data_to_send[_cnt++] = 0x0D;

    _temp=(s16)(a_x*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp=(s16)(a_y*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp=(s16)(a_z*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    _temp=(s16)(g_x*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp=(s16)(g_y*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp=(s16)(g_z*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    data_to_send[_cnt++]=BYTE0(sta);

    u8 sumcheck = 0;
    u8 addcheck = 0;
    for(u8 i=0;i<_cnt;i++){
        sumcheck += data_to_send[i];
        addcheck += sumcheck;
    }
    data_to_send[_cnt++] = sumcheck;
    data_to_send[_cnt++] = addcheck;
    ANO_DT_Send_Data(data_to_send, _cnt);
}
