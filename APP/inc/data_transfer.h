#ifndef _DATA_TRANSFER_H_
#define _DATA_TRANSFER_H_

#include "stm32f4xx.h"

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float  angle_yaw, u8 sta);
void ANO_DT_Send_Senser(float a_x, float a_y, float a_z, float g_x, float g_y, float g_z, u8 sta);
void ANO_DT_Send_Senser2(float m_x, float m_y, float m_z, s32 alt, s16 tmp, u8 bar_sta, u8 mag_sta);
void ANO_DT_Send_Data(u8 *dataToSend,u8 length);

#endif  
