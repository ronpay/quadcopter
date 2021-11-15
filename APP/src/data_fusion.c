#include "data_fusion.h"

float          exInt, eyInt, ezInt;
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // roll,pitch,yaw 都为 0 时对应的四元数值。
volatile float Pitch, Roll, Yaw;

#define Kp 10.0f      // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.008f     // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.003f  // half the sample period采样周期的一半

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float norm;
    float hx, hy, hz, bx, bz;
    float wx, wy, wz;
    float vx, vy, vz;
    float ex, ey, ez;

    // 先把这些用得到的值算好
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    if (ax * ay * az == 0)
        return;

    if (mx * my * mz == 0)
        return;

    norm = sqrt(ax * ax + ay * ay + az * az);  // acc数据归一化
    ax   = ax / norm;
    ay   = ay / norm;
    az   = az / norm;

    norm = sqrt(mx * mx + my * my + mz * mz);  // mag数据归一化
    mx   = mx / norm;
    my   = my / norm;
    mz   = mz / norm;

    //  mx = 0;
    //  my = 0;
    //  mz = 0;

    hx = 2 * mx * (0.5 - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2);
    hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5 - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1);
    hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5 - q1q1 - q2q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = hz;

    // estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
    vx = 2 * (q1q3 - q0q2);  //四元素中xyz的表示
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    wx = 2 * bx * (0.5 - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
    wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
    wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5 - q1q1 - q2q2);

    // error is sum of cross product between reference direction of fields and direction measured by sensors
    //  ex = (ay*vz - az*vy) ;                                               //向量外积在相减得到差分就是误差
    //  ey = (az*vx - ax*vz) ;
    //  ez = (ax*vy - ay*vx) ;

    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    exInt = exInt + ex * Ki;  //对误差进行积分
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    // adjusted gyroscope measurements
    gx = gx + Kp * ex + exInt;  //将误差PI后补偿到陀螺仪，即补偿零点漂移
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;  //这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

    // integrate quaternion rate and normalise                           //四元素的微分方程
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

    // normalise quaternion
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0   = q0 / norm;
    q1   = q1 / norm;
    q2   = q2 / norm;
    q3   = q3 / norm;

    Yaw   = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3;  // yaw
    Pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;                                  // pitch
    Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;  // roll
}

void Conversion_Quaternion_to_Euler(float q0, float q1, float q2, float q3)
{
    Pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
    Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;
    Yaw   = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;
}
