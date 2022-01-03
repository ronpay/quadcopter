#include "data_fusion.h"
#include "mpu6050.h"

float          exInt, eyInt, ezInt;
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // roll,pitch,yaw 都为 0 时对应的四元数值。
volatile float Pitch, Roll, Yaw;

#define Kp 2.0f       // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f     // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.0025f  // half the sample period采样周期的一半

extern int16_t Mag_gs[3];

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
//        return;

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

void Attitude_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float norm;
    float hx, hy, hz, bz, by;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float q0_last, q1_last, q2_last, q3_last;

    //空间换时间，提高效率
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

    //加速度计归一化，这就是为什么之前只要陀螺仪数据进行单位转换
    norm = invSqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    //计算上一时刻机体坐标系下加速度坐标
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    //磁力计数据归一化
//    norm = invSqrt(mx * mx + my * my + mz * mz);
//    mx = mx * norm;
//    my = my * norm;
//    mz = mz * norm;
//    //计算地理坐标系下磁力坐标（地理南北极）
//    hx = 2 * mx * (0.5f - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2);
//    hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5f - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1);
//    hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5f - q1q1 - q2q2);
//    //（地磁南北极）因为地磁南北极与地理南北极有偏差
//    //bx=0;
//    by = sqrtf((hx * hx) + (hy * hy));
//    bz = hz;
//    //磁力转换回机体坐标系坐标
//    wx = 2 * by * (q1q2 + q0q3) + 2 * bz * (q1q3 - q0q2);
//    wy = 2 * by * (0.5f - q1q1 - q3q3) + 2 * bz * (q0q1 + q2q3);
//    wz = 2 * by * (q2q3 - q0q1) + 2 * bz * (0.5f - q1q1 - q2q2);

    //误差计算（加速度和磁场强度一起）
//    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
//    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
//    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    ex = (ay * vz - az * vy) ;
    ey = (az * vx - ax * vz) ;
    ez = (ax * vy - ay * vx) ;
    //由定时器获取采样周期的一半

    //pi运算
    if (ex != 0.0f && ey != 0.0f && ez != 0.0f) {
        // 误差积分
        exInt += ex * Ki * halfT;
        eyInt += ey * Ki * halfT;
        ezInt += ez * Ki * halfT;

        // 角速度补偿
        gx = gx + Kp * ex + exInt;
        gy = gy + Kp * ey + eyInt;
        gz = gz + Kp * ez + ezInt;
    }

    // 保存四元数
    q0_last = q0;
    q1_last = q1;
    q2_last = q2;
    q3_last = q3;
    // 积分增量运算
    q0 = q0_last + (-q1_last * gx - q2_last * gy - q3_last * gz) * halfT;
    q1 = q1_last + (q0_last * gx + q2_last * gz - q3_last * gy) * halfT;
    q2 = q2_last + (q0_last * gy - q1_last * gz + q3_last * gx) * halfT;
    q3 = q3_last + (q0_last * gz + q1_last * gy - q2_last * gx) * halfT;

    // 归一化四元数
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 * norm; //w
    q1 = q1 * norm; //x
    q2 = q2 * norm; //y
    q3 = q3 * norm; //z

    //四元数转欧拉角
    Roll = asin(2.0f * (q0 * q2 - q1 * q3)) * 57.3f;
	 //Roll=0;
//		Pitch = 0;
    Pitch = -atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 57.3f;
    Yaw = atan2(2.0f * (q0 * q3 + q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3f;
		//Yaw = 0;
}

void Quat_Init(void)
{
    int16_t initMx, initMy, initMz;
    float   initYaw, initPitch, initRoll;

    Read_Mag_Gs();

    initMx = Mag_gs[0];
    initMx = Mag_gs[1];
    initMx = Mag_gs[2];

    //求出初始欧拉角，初始状态水平，所以roll、pitch为0
    initRoll  = 0.0f;
    initPitch = 0.0f;
		initYaw = 0.0f;
//    initYaw   = atan2(initMx * cos(initRoll) + initMy * sin(initRoll) * sin(initPitch) + initMz * sin(initRoll) * cos(initPitch),
//                      initMy * cos(initPitch) - initMz * sin(initPitch));

    // 四元数计算
    q0 = cos(0.5f * initRoll) * cos(0.5f * initPitch) * cos(0.5f * initYaw) + sin(0.5f * initRoll) * sin(0.5f * initPitch) * sin(0.5f * initYaw);  // w
    q1 = cos(0.5f * initRoll) * sin(0.5f * initPitch) * cos(0.5f * initYaw) - sin(0.5f * initRoll) * cos(0.5f * initPitch) * sin(0.5f * initYaw);  // x Pitch
    q2 = sin(0.5f * initRoll) * cos(0.5f * initPitch) * cos(0.5f * initYaw) + cos(0.5f * initRoll) * sin(0.5f * initPitch) * sin(0.5f * initYaw);  // y Roll
    q3 = cos(0.5f * initRoll) * cos(0.5f * initPitch) * sin(0.5f * initYaw) - sin(0.5f * initRoll) * sin(0.5f * initPitch) * cos(0.5f * initYaw);  // z Yaw
}
