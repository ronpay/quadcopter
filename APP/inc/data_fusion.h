#ifndef _DATA_FUSION_H_
#define _DATA_FUSION_H_

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define Kp 10.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.008f                          // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.001f                   // half the sample period采样周期的一半

float exInt, eyInt, ezInt;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // roll,pitch,yaw 都为 0 时对应的四元数值。
// float roll, pitch, yaw; //has defined in main.c
extern float roll, pitch, yaw;

#endif
