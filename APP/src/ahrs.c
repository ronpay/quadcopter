//---------------------------------------------------------------------------------------------------
// Header files

#include "ahrs.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq 100.0f  // sample frequency in Hz

//---------------------------------------------------------------------------------------------------
// Variable definitions

// extern volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame
// extern volatile float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

#define betaDef 0.041f          // 2 * proportional gain
volatile float beta = betaDef;  // 2 * proportional gain (Kp)

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3,
        q3q3;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0   = 2.0f * q0;
        _2q1   = 2.0f * q1;
        _2q2   = 2.0f * q2;
        _2q3   = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0   = q0 * q0;
        q0q1   = q0 * q1;
        q0q2   = q0 * q2;
        q0q3   = q0 * q3;
        q1q1   = q1 * q1;
        q1q2   = q1 * q2;
        q1q3   = q1 * q3;
        q2q2   = q2 * q2;
        q2q3   = q2 * q3;
        q3q3   = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx   = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy   = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
             _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
             (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
             (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    const float q0q0MinusHalf = q0 * q0 - 0.5f;
    roll                      = atan2f(q2 * q3 + q0 * q1, q0q0MinusHalf + q3 * q3) * 57.3f;
    pitch                     = -1.0f * asinf(2.0f * (q1 * q3 - q0 * q2)) * 57.3f;
    yaw                       = atan2f(q1 * q2 + q0 * q3, q0q0MinusHalf + q1 * q1) * 57.3f;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0        = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1        = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2        = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3        = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    const float q0q0MinusHalf = q0 * q0 - 0.5f;
    roll                      = atan2f(q2 * q3 + q0 * q1, q0q0MinusHalf + q3 * q3) * 57.3f;
    pitch                     = -1.0f * asinf(2.0f * (q1 * q3 - q0 * q2)) * 57.3f;
    yaw                       = atan2f(q1 * q2 + q0 * q3, q0q0MinusHalf + q1 * q1) * 57.3f;
}

#define a 1
void MadgwickAHRSupdateIMUu(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    float u = a * sqrtf(qDot1 * qDot1 + qDot2 * qDot2 + qDot3 * qDot3 + qDot4 * qDot4) * (1.0f / sampleFreq);

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0        = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1        = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2        = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3        = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    float y  = 1.f / (1.f + u / (beta * (1.0f / sampleFreq)));
    float y1 = 1.f - y;

    // Apply feedback step
    q0 -= y * u * s0 - qDot1 * y1 * (1.0f / sampleFreq);
    q1 -= y * u * s1 - qDot2 * y1 * (1.0f / sampleFreq);
    q2 -= y * u * s2 - qDot3 * y1 * (1.0f / sampleFreq);
    q3 -= y * u * s3 - qDot4 * y1 * (1.0f / sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    const float q0q0MinusHalf = q0 * q0 - 0.5f;
    roll                      = atan2f(q2 * q3 + q0 * q1, q0q0MinusHalf + q3 * q3) * 57.3f;
    pitch                     = -1.0f * asinf(2.0f * (q1 * q3 - q0 * q2)) * 57.3f;
    yaw                       = atan2f(q1 * q2 + q0 * q3, q0q0MinusHalf + q1 * q1) * 57.3f;
}

#define twoKpDef (2.0f * 5.f)     // 2 * proportional gain
#define twoKiDef (2.0f * 0.004f)  // 2 * integral gain

volatile float twoKp       = twoKpDef;                                      // 2 * proportional gain (Kp)
volatile float twoKi       = twoKiDef;                                      // 2 * integral gain (Ki)
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;  // integral error terms scaled by Ki

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx;  // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f;  // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq));  // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    const float q0q0MinusHalf = q0 * q0 - 0.5f;
    roll                      = atan2f(q2 * q3 + q0 * q1, q0q0MinusHalf + q3 * q3) * 57.3f;
    pitch                     = -1.0f * asinf(2.0f * (q1 * q3 - q0 * q2)) * 57.3f;
    yaw                       = atan2f(q1 * q2 + q0 * q3, q0q0MinusHalf + q1 * q1) * 57.3f;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx;  // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f;  // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq));  // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    const float q0q0MinusHalf = q0 * q0 - 0.5f;
    roll                      = atan2f(q2 * q3 + q0 * q1, q0q0MinusHalf + q3 * q3) * 57.3f;
    pitch                     = -1.0f * asinf(2.0f * (q1 * q3 - q0 * q2)) * 57.3f;
    yaw                       = atan2f(q1 * q2 + q0 * q3, q0q0MinusHalf + q1 * q1) * 57.3f;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y     = x;
    long  i     = *(long*)&y;
    i           = 0x5f3759df - (i >> 1);
    y           = *(float*)&i;
    y           = y * (1.5f - (halfx * y * y));
    return y;
}

void gyroupdate(float gx, float gy, float gz)
{
    //	float recipNorm;
    //	float qDot1, qDot2, qDot3, qDot4;

    //	// Rate of change of quaternion from gyroscope
    //	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    //	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    //	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    //	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    //	// Integrate rate of change of quaternion to yield quaternion
    //	q0 += qDot1 * (1.0f / sampleFreq);
    //	q1 += qDot2 * (1.0f / sampleFreq);
    //	q2 += qDot3 * (1.0f / sampleFreq);
    //	q3 += qDot4 * (1.0f / sampleFreq);

    //	// Normalise quaternion
    //	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    //	q0 *= recipNorm;
    //	q1 *= recipNorm;
    //	q2 *= recipNorm;
    //	q3 *= recipNorm;

    //	const float q0q0MinusHalf = q0 * q0 - 0.5f;
    //	roll = atan2f(q2 * q3 + q0 * q1, q0q0MinusHalf + q3 * q3) * 57.3f;
    //	pitch = -1.0f * asinf(2.0f * (q1 * q3 - q0 * q2)) * 57.3f;
    //	yaw = atan2f(q1 * q2 + q0 * q3, q0q0MinusHalf + q1 * q1) * 57.3f;

    //	r *= 0.5f;
    //	p *= 0.5f;
    //	y *= 0.5f;

    //	q0 = cosf(r) * cosf(p) * cosf(y) - sinf(r) * sinf(p) * sinf(y);
    //	q1 = cosf(r) * sinf(p) * cosf(y) - cosf(r) * sinf(p) * sinf(y);
    //	q2 = cosf(r) * sinf(p) * cosf(y) - sinf(r) * cosf(p) * sinf(y);
    //	q3 = cosf(r) * cosf(p) * sinf(y) - sinf(r) * sinf(p) * cosf(y);

    float dr, dp, dy;
    float sinr  = sinf(roll / 57.3f);
    float cosr  = cosf(roll / 57.3f);
    float sinp  = sinf(pitch / 57.3f);
    float cosp  = cosf(pitch / 57.3f);
    float cosp1 = 1.0f / cosp;

    dr = gx + sinp * sinr * cosp1 * gy + cosr * sinp * cosp1 * gz;
    dp = cosr * gy - sinr * gz;
    dy = sinr * gy * cosp1 + cosr * gz * cosp1;

    roll += dr * (1.0f / sampleFreq) * 57.3f;
    pitch += dp * (1.0f / sampleFreq) * 57.3f;
    yaw += dy * (1.0f / sampleFreq) * 57.3f;
}

float eps;
int   c;

void accupdate(float ax, float ay, float az)
{
    //	float recipNorm;
    //	float f1, f2, f3;
    //	float s0, s1, s2, s3;
    //	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    //	// Normalise accelerometer measurement
    //	recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    //	ax *= recipNorm;
    //	ay *= recipNorm;
    //	az *= recipNorm;

    //	do
    //	{
    //		// Auxiliary variables to avoid repeated arithmetic
    //		_2q0 = 2.0f * q0;
    //		_2q1 = 2.0f * q1;
    //		_2q2 = 2.0f * q2;
    //		_2q3 = 2.0f * q3;
    //		_4q0 = 4.0f * q0;
    //		_4q1 = 4.0f * q1;
    //		_4q2 = 4.0f * q2;
    //		_8q1 = 8.0f * q1;
    //		_8q2 = 8.0f * q2;
    //		q0q0 = q0 * q0;
    //		q1q1 = q1 * q1;
    //		q2q2 = q2 * q2;
    //		q3q3 = q3 * q3;

    //		// Gradient decent algorithm corrective step
    //		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    //		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    //		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    //		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    //		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    //		s0 *= recipNorm;
    //		s1 *= recipNorm;
    //		s2 *= recipNorm;
    //		s3 *= recipNorm;

    //		// Apply feedback step
    //		q0 -= beta * s0;
    //		q1 -= beta * s1;
    //		q2 -= beta * s2;
    //		q3 -= beta * s3;

    //		// Normalise quaternion
    //		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    //		q0 *= recipNorm;
    //		q1 *= recipNorm;
    //		q2 *= recipNorm;
    //		q3 *= recipNorm;

    //		f1 = 2 * (q1 * q3 - q0 * q2) - ax;
    //		f2 = 2 * (q0 * q1 + q2 * q3) - ay;
    //		f3 = 1 - 2 * (q1 * q1 + q2 * q2) - az;

    //		eps = sqrt(f1 * f1 + f2 * f2 + f3 * f3);
    //	} while (eps > 0.1f);

    //	const float q0q0MinusHalf = q0 * q0 - 0.5f;
    //	roll = atan2f(q2 * q3 + q0 * q1, q0q0MinusHalf + q3 * q3) * 57.3f;
    //	pitch = -1.0f * asinf(2.0f * (q1 * q3 - q0 * q2)) * 57.3f;
    //	yaw = atan2f(q1 * q2 + q0 * q3, q0q0MinusHalf + q1 * q1) * 57.3f;

    roll  = atan2f(ay, az) * 57.3f;
    pitch = -1.0f * atan2f(ax, sqrt(ay * ay + az * az)) * 57.3f;
    yaw   = 0.f;
}

#define ut 0.01f

float hx, hy;
void  accmagupdate(float ax, float ay, float az, float mx, float my, float mz)
{
    // Normalise magnetometer measurement
    float recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    float r = atan2f(ay, az);
    float p = -1.0f * atan2f(ax, sqrt(ay * ay + az * az));
    float y = atan2f(my * cosf(r) - mz * sinf(r), mx * cosf(p) + my * sinf(r) * sinf(p) + mz * cosf(r) * sinf(p));

    hx = mx * cosf(p) + my * sinf(r) * sinf(p) + mz * cosf(r) * sinf(p);
    hy = my * cosf(r) - mz * sinf(r);

    roll  = r * 57.3f;
    pitch = p * 57.3f;
    yaw   = -y * 57.3f;
    // yaw = 0.f;

    //	float recipNorm;
    //	float f1, f2, f3;
    //	float s0, s1, s2, s3;
    //	float hx, hy;
    //	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2,
    // q2q3, q3q3;

    //	// Normalise accelerometer measurement
    //	recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    //	ax *= recipNorm;
    //	ay *= recipNorm;
    //	az *= recipNorm;

    //	// Normalise magnetometer measurement
    //	recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    //	mx *= recipNorm;
    //	my *= recipNorm;
    //	mz *= recipNorm;

    //	int count = 0;
    //	do
    //	{
    //		count++;
    //		// Auxiliary variables to avoid repeated arithmetic
    //		_2q0mx = 2.0f * q0 * mx;
    //		_2q0my = 2.0f * q0 * my;
    //		_2q0mz = 2.0f * q0 * mz;
    //		_2q1mx = 2.0f * q1 * mx;
    //		_2q0 = 2.0f * q0;
    //		_2q1 = 2.0f * q1;
    //		_2q2 = 2.0f * q2;
    //		_2q3 = 2.0f * q3;
    //		_2q0q2 = 2.0f * q0 * q2;
    //		_2q2q3 = 2.0f * q2 * q3;
    //		q0q0 = q0 * q0;
    //		q0q1 = q0 * q1;
    //		q0q2 = q0 * q2;
    //		q0q3 = q0 * q3;
    //		q1q1 = q1 * q1;
    //		q1q2 = q1 * q2;
    //		q1q3 = q1 * q3;
    //		q2q2 = q2 * q2;
    //		q2q3 = q2 * q3;
    //		q3q3 = q3 * q3;

    //		// Reference direction of Earth's magnetic field
    //		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    //		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    //		_2bx = sqrt(hx * hx + hy * hy);
    //		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    //		_4bx = 2.0f * _2bx;
    //		_4bz = 2.0f * _2bz;

    //		// Gradient decent algorithm corrective step
    //		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) -
    //mx)
    //+ (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    //		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx
    //* (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz *
    //q1)
    //* (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz); 		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2
    //* (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) *
    //(_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz); 		s3 = _2q1
    //* (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
    //+
    //(-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    //		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    //		s0 *= recipNorm;
    //		s1 *= recipNorm;
    //		s2 *= recipNorm;
    //		s3 *= recipNorm;

    //		// Apply feedback step
    //		q0 -= ut * s0;
    //		q1 -= ut * s1;
    //		q2 -= ut * s2;
    //		q3 -= ut * s3;

    //		// Normalise quaternion
    //		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    //		q0 *= recipNorm;
    //		q1 *= recipNorm;
    //		q2 *= recipNorm;
    //		q3 *= recipNorm;

    //		f1 = 2 * (q1 * q3 - q0 * q2) - ax;
    //		f2 = 2 * (q0 * q1 + q2 * q3) - ay;
    //		f3 = 1 - 2 * (q1 * q1 + q2 * q2) - az;

    //		eps = sqrt(f1 * f1 + f2 * f2 + f3 * f3);
    //	} while (eps > 0.08f);

    //	const float q0q0MinusHalf = q0 * q0 - 0.5f; // calculate common terms to avoid repeated operations
    //	roll = atan2f(q2 * q3 + q0 * q1, q0q0MinusHalf + q3 * q3) * 57.3f;
    //	pitch = -1.0f * asinf(2.0f * (q1 * q3 - q0 * q2)) * 57.3f;
    //	yaw = atan2f(q1 * q2 + q0 * q3, q0q0MinusHalf + q1 * q1) * 57.3f;
}

#define k 0.8f

void whzupdate_euler(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float dr, dp, dy;

    float sinr  = sinf(roll / 57.3f);
    float cosr  = cosf(roll / 57.3f);
    float sinp  = sinf(pitch / 57.3f);
    float cosp  = cosf(pitch / 57.3f);
    float cosp1 = 1.0f / cosp;

    dr = gx + sinp * sinr * cosp1 * gy + cosr * sinp * cosp1 * gz;
    dp = cosr * gy - sinr * gz;
    dy = sinr * gy * cosp1 + cosr * gz * cosp1;

    float rw = dr * (1.0f / sampleFreq) * 57.3f + roll;
    float pw = dp * (1.0f / sampleFreq) * 57.3f + pitch;
    float yw = dy * (1.0f / sampleFreq) * 57.3f + yaw;

    float rg = atan2f(ay, az);
    float pg = -1.0f * atan2f(ax, sqrtf(ay * ay + az * az));
    // float yg = atan2f(my * cosf(rg) - mz * sinf(rg), mx * cosf(pg) + my * sinf(rg) * sinf(pg) + mz * cosf(rg) * sinf(pg));
    float yg = atan2f(my * cosf(roll / 57.3f) - mz * sinf(roll / 57.3f),
                      mx * cosf(pitch / 57.3f) + my * sinf(roll / 57.3f) * sinf(pitch / 57.3f) + mz * cosf(roll / 57.3f) * sinf(pitch / 57.3f));

    rg *= 57.3f;
    pg *= 57.3f;
    yg *= 57.3f;

    float w, t, t1;
    w  = sqrtf(gx * gx + gy * gy + gz * gz);
    t  = k / (w + k);
    t1 = 1 - t;

    roll  = t1 * rw + t * rg;
    pitch = t1 * pw + t * pg;
    //	yaw = t1 * yw + t * yg;
    yaw = yw;
}

void whzupdate_quaternion(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float qDot1, qDot2, qDot3, qDot4;
    float qw0, qw1, qw2, qw3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Integrate rate of change of quaternion to yield quaternion
    qw0 = qDot1 * (1.0f / sampleFreq) + q0;
    qw1 = qDot2 * (1.0f / sampleFreq) + q1;
    qw2 = qDot3 * (1.0f / sampleFreq) + q2;
    qw3 = qDot4 * (1.0f / sampleFreq) + q3;

    // Normalise quaternion
    recipNorm = invSqrt(qw0 * qw0 + qw1 * qw1 + qw2 * qw2 + qw3 * qw3);
    qw0 *= recipNorm;
    qw1 *= recipNorm;
    qw2 *= recipNorm;
    qw3 *= recipNorm;

    float f1, f2, f3;
    float s0, s1, s2, s3;
    float qd0, qd1, qd2, qd3;
    float _2qd0, _2qd1, _2qd2, _2qd3, _4qd0, _4qd1, _4qd2, _8qd1, _8qd2, qd0qd0, qd1qd1, qd2qd2, qd3qd3;

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    qd0 = q0;
    qd1 = q1;
    qd2 = q2;
    qd3 = q3;

    // Auxiliary variables to avoid repeated arithmetic
    _2qd0  = 2.0f * qd0;
    _2qd1  = 2.0f * qd1;
    _2qd2  = 2.0f * qd2;
    _2qd3  = 2.0f * qd3;
    _4qd0  = 4.0f * qd0;
    _4qd1  = 4.0f * qd1;
    _4qd2  = 4.0f * qd2;
    _8qd1  = 8.0f * qd1;
    _8qd2  = 8.0f * qd2;
    qd0qd0 = qd0 * qd0;
    qd1qd1 = qd1 * qd1;
    qd2qd2 = qd2 * qd2;
    qd3qd3 = qd3 * qd3;

    // Gradient decent algorithm corrective step
    s0        = _4qd0 * qd2qd2 + _2qd2 * ax + _4qd0 * qd1qd1 - _2qd1 * ay;
    s1        = _4qd1 * qd3qd3 - _2qd3 * ax + 4.0f * qd0qd0 * qd1 - _2qd0 * ay - _4qd1 + _8qd1 * qd1qd1 + _8qd1 * qd2qd2 + _4qd1 * az;
    s2        = 4.0f * qd0qd0 * qd2 + _2qd0 * ax + _4qd2 * qd3qd3 - _2qd3 * ay - _4qd2 + _8qd2 * qd1qd1 + _8qd2 * qd2qd2 + _4qd2 * az;
    s3        = 4.0f * qd1qd1 * qd3 - _2qd1 * ax + 4.0f * qd2qd2 * qd3 - _2qd2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qd0 = qd0 - ut * s0;
    qd1 = qd1 - ut * s1;
    qd2 = qd2 - ut * s2;
    qd3 = qd3 - ut * s3;

    // Normalise quaternion
    recipNorm = invSqrt(qd0 * qd0 + qd1 * qd1 + qd2 * qd2 + qd3 * qd3);
    qd0 *= recipNorm;
    qd1 *= recipNorm;
    qd2 *= recipNorm;
    qd3 *= recipNorm;

    f1 = 2 * (qd1 * qd3 - qd0 * qd2) - ax;
    f2 = 2 * (qd0 * qd1 + qd2 * qd3) - ay;
    f3 = 1 - 2 * (qd1 * qd1 + qd2 * qd2) - az;

    eps = sqrt(f1 * f1 + f2 * f2 + f3 * f3);

    float w, t, t1;
    w  = sqrtf(gx * gx + gy * gy + gz * gz);
    t  = k / (w + k);
    t1 = 1 - t;

    q0 = t1 * qw0 + t * qd0;
    q1 = t1 * qw1 + t * qd1;
    q2 = t1 * qw2 + t * qd2;
    q3 = t1 * qw3 + t * qd3;

    const float q0q0MinusHalf = q0 * q0 - 0.5f;
    roll                      = atan2f(q2 * q3 + q0 * q1, q0q0MinusHalf + q3 * q3) * 57.3f;
    pitch                     = -1.0f * asinf(2.0f * (q1 * q3 - q0 * q2)) * 57.3f;
    yaw                       = atan2f(q1 * q2 + q0 * q3, q0q0MinusHalf + q1 * q1) * 57.3f;
}
//====================================================================================================
// END OF CODE
//====================================================================================================
