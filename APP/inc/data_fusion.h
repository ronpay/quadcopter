#ifndef _DATA_FUSION_H_
#define _DATA_FUSION_H_

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>

extern volatile float q0, q1, q2, q3;

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void Conversion_Quaternion_to_Euler(float q0, float q1, float q2, float q3);
void Attitude_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void Quat_Init(void);
#endif
