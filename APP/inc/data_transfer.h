#ifndef _DATA_TRANSFER_H_
#define _DATA_TRANSFER_H_

#include "stm32f4xx.h"

void ANO_DT_Send_Status(s16 angle_rol, s16 angle_pit, s16 angle_yaw, u8 sta);
void ANO_DT_Send_Senser(s16 a_x, s16 a_y, s16 a_z, s16 g_x, s16 g_y, s16 g_z, u8 sta);
void ANO_DT_Send_Data(u8 *dataToSend,u8 length);

#endif  