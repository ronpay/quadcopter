#ifndef _DATA_TRANSFER_H_
#define _DATA_TRANSFER_H_

#include "stm32f4xx.h"

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, uint8_t sta);
void ANO_DT_Send_Senser(float a_x, float a_y, float a_z, float g_x, float g_y, float g_z, uint8_t sta);
void ANO_DT_Send_Senser2(float m_x, float m_y, float m_z, int32_t alt, int16_t tmp, uint8_t bar_sta, uint8_t mag_sta);
void ANO_DT_Send_Data(uint8_t* dataToSend, uint8_t length);
void ANO_DT_Send_PWM(uint16_t PWM1, uint16_t PWM2, uint16_t PWM3, uint16_t PWM4);

#endif
