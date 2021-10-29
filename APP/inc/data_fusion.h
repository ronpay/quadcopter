#ifndef _DATA_FUSION_H_
#define _DATA_FUSION_H_

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define Kp 10.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.008f                          // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.0005f                   // half the sample period采样周期的一半


void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void Conversion_Quaternion_to_Euler(float q0, float q1, float q2, float q3);
#endif
