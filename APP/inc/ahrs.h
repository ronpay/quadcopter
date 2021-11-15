//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef __AHRS_H
#    define __AHRS_H

#    include "data_fusion.h"
extern volatile float Pitch, Roll, Yaw;
//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float beta;  // algorithm gain
// extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
// extern volatile float pitch, roll, yaw;

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void MadgwickAHRSupdateIMUu(float gx, float gy, float gz, float ax, float ay, float az);
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

void gyroupdate(float gx, float gy, float gz);
void accupdate(float ax, float ay, float az);
void accmagupdate(float ax, float ay, float az, float mx, float my, float mz);
void whzupdate_euler(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void whzupdate_quaternion(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
