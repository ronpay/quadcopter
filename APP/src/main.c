#include "stm32f4xx.h"
#include "ucos_ii.h"

#include "ahrs.h"
#include "data_fusion.h"
#include "data_transfer.h"
#include "hm10.h"
#include "hmc5883l.h"
#include "inv_mpu.h"
#include "motor.h"
#include "mpu6050.h"
#include "oled.h"
#include "pid.h"
#include "quad_pid.h"
#include "receiver.h"
#include "si2c.h"
#include "sysTick.h"

OS_EVENT* PIDSem;
OS_EVENT* SensorSem;
OS_EVENT* I2CSem;
// OS_EVENT* ReceiverSem;
extern u8 hm_flag;

// GY86 所需要的数组
extern float             Acel_mps[3];
extern int16_t           Acel_raw[3];
extern float             Gyro_dps[3];
extern int16_t           Gyro_raw[3];
float                    Gyro_int[3] = {0, 0, 0};
extern int16_t           Mag_raw[3];
extern int16_t           Mag_gs[3];
extern float             Temp;
extern volatile float    Pitch, Roll, Yaw;
extern volatile uint16_t Duty[6];
// for pid
extern PID_TYPE       Roll_w_PID, Pitch_w_PID, Yaw_w_PID;
extern PID_TYPE       Roll_PID, Pitch_PID, Yaw_PID;
extern volatile float Pitch_T, Roll_T, Yaw_T;

volatile uint16_t Base_CCR;          // 遥控器提供的基准油门
uint16_t          Fly_Thre_CCR = 0;  // 无人机起飞的最低油门
uint16_t          Margin_CCR   = 0;  // 做姿态需要的油门余量, 防止掉下来
#define BASE_MIN 400
#define BASE_MAX 1250

uint16_t Servo_PWM[4] = {0};

#define CONTROL_TASK_PRIO 10
#define DATA_TRANSFER_TASK_PRIO 20

#define CONTROL_STK_SIZE 512
#define DATA_TRANSFER_STK_SIZE 128

static OS_STK CONTROL_TASK_STK[CONTROL_STK_SIZE];
static OS_STK DATA_TRANSFER_TASK_STK[DATA_TRANSFER_STK_SIZE];

#define DMP 0
#define MOTOR 1
#define RECEIVER 1
#define PID 1

int   timeCnt = 0;
float T       = 0.005;

void INIT_TASK(void* pdata)
{
    HM10_Config();

    Delay_s(1);

#if MOTOR
    Motor_Config();
    Motor_Unlock();
#endif
#if RECEIVER
    Receiver_Config();
#endif

#if DMP
    // with dmp
    MPU6050_Config();
    Delay_s(2);
    int ret = mpu_dmp_init();
    if (ret != 0) {
        printf("ret:%d\n", ret);
    }
    Delay_s(1);
    // Only when dmp
    MPU_HMC_Init();
    // 记得打开 GY86_TASK 中的 mpu_dmp_get_data
#else

    Delay_s(1);
    // without dmp
    GY86_Init();
    //		OLED_CLS();
    Delay_s(1);
    IIC_Init();
    GY86_SelfTest();
#endif
#if PID
    Gesture_PID_Init();
#endif
    Quat_Init();
}

void DATA_TRANSFER_TASK(void* pdata)
{
    //    INT8U err;
    while (1) {
        ANO_DT_Send_Status(Roll, Pitch, Yaw, timeCnt);

        ANO_DT_Send_Senser(Acel_mps[0] * 100, Acel_mps[1] * 100, Acel_mps[2] * 100, Gyro_dps[0] * 100, Gyro_dps[1] * 100, Gyro_dps[2] * 100, 0);
        ANO_DT_Send_Senser2(Mag_raw[0], Mag_raw[1], Mag_raw[2], 0, 0, 0, 0);
        /* 遥控器四通道的量 */
        ANO_DT_Send_PWM(Duty[0], Duty[1], Duty[2], Duty[3]);
/* 目标姿态，通过遥控器的数据直接算出 */
#if PID
        ANO_DT_Send_Target_Status(Roll_T, Pitch_T, Yaw_T);

        /* PID 计算出来的三个角度的PWM控制量 */
        ANO_DT_Send_Control_Status(Roll_w_PID.Output, Pitch_w_PID.Output, Base_CCR, Yaw_w_PID.Output);
#endif

        OSTimeDly(20);
    }
}

void CONTROL_TASK(void* pdata)
{
    int cnt = 0;

    while (1) {
        Read_Accel_MPS();
        Read_Gyro_DPS();
        Read_Mag_Gs();

        for (int i = 0; i < 3; i++) {
            Gyro_int[i] += Gyro_dps[i] * 57.2957795f * T;
        }

        Attitude_Update(Gyro_dps[0], Gyro_dps[1], Gyro_dps[2], Acel_mps[0], Acel_mps[1], Acel_mps[2], Mag_gs[0], Mag_gs[1], Mag_gs[2]);

        if (cnt == 0) {
            /* 外环任务 */
            PID_Cycle(&Roll_PID);
            PID_Cycle(&Pitch_PID);
            PID_Cycle(&Yaw_PID);
        }

        /* 内环任务 */
        PID_Cycle(&Roll_w_PID);
        PID_Cycle(&Pitch_w_PID);
        PID_Cycle(&Yaw_w_PID);

        /* 给电机分配占空比 */
        // 电机输出足够起飞且有一定余量做动作时, 设置PWM
        // 飞行模式
        if (Base_CCR > (Fly_Thre_CCR + Margin_CCR)) {
            // 3201
            //
            Servo_PWM[3] = Limit(Base_CCR + Roll_w_PID.Output + Pitch_w_PID.Output - Yaw_w_PID.Output, BASE_MIN, BASE_MAX);  // x+  y+
            Servo_PWM[0] = Limit(Base_CCR - Roll_w_PID.Output + Pitch_w_PID.Output + Yaw_w_PID.Output, BASE_MIN, BASE_MAX);  // x-  y+
            Servo_PWM[2] = Limit(Base_CCR + Roll_w_PID.Output - Pitch_w_PID.Output + Yaw_w_PID.Output, BASE_MIN, BASE_MAX);  // x+  y-
            Servo_PWM[1] = Limit(Base_CCR - Roll_w_PID.Output - Pitch_w_PID.Output - Yaw_w_PID.Output, BASE_MIN, BASE_MAX);  // x-  y-
        }
        // 电机输出不够, 不允许起飞
        else {
            for (int i = 0; i < 4; i++)
                Servo_PWM[i] = 0;
        }

        if (Roll > 65 || Roll < -65 || Pitch > 65 || Pitch < -65) {
            for (int i = 0; i < 4; i++)
                Servo_PWM[i] = BASE_MIN;
        }
        if (Base_CCR <= 550) {
            for (int i = 0; i < 4; i++) {
                Servo_PWM[i] = 400;
            }
        }
        for (int i = 0; i < 4; i++) {
            Motor_Set(Servo_PWM[i], i + 1);
        }

        cnt++;
        timeCnt++;
        cnt %= 5;

        OSTimeDly(5);
    }
}

//使能systick中断为1ms
void SystemClock_Config()
{
    SysTick_Config(SystemCoreClock / 1000);
}

int main()
{
    SystemClock_Config();
    RCC_ClocksTypeDef get_rcc_clock;
    RCC_GetClocksFreq(&get_rcc_clock);
    //	SysTick_Init();

    INIT_TASK(NULL);
    Delay_s(1);
    OSInit();

    SensorSem = OSSemCreate(0);
    //	ReceiverSem=OSSemCreate(1);
    PIDSem = OSSemCreate(0);
    I2CSem = OSSemCreate(1);
    OSTaskCreate(DATA_TRANSFER_TASK, (void*)0, (void*)&DATA_TRANSFER_TASK_STK[DATA_TRANSFER_STK_SIZE - 1], DATA_TRANSFER_TASK_PRIO);
    OSTaskCreate(CONTROL_TASK, (void*)0, (void*)&CONTROL_TASK_STK[CONTROL_STK_SIZE - 1], CONTROL_TASK_PRIO);

    // OSTaskCreate(Quadcopter_Imple_Task, (void*)0, (void*)&PID_OUTER_TASK_STK[PID_OUTER_STK_SIZE - 1], PID_OUTER_TASK_PRIO);
    //    OSTaskCreate(Quadcopter_Control_Task, (void*)0, (void*)&PID_INNER_TASK_STK[PID_INNER_STK_SIZE - 1], PID_INNER_TASK_PRIO);
    //    OSTaskCreate(GY86_TASK, (void*)0, (void*)&GY86_TASK_STK[GY86_STK_SIZE - 1], GY86_TASK_PRIO);
    // OSTaskCreate(DATA_FUSION_TASK, (void*)0, (void*)&DATA_FUSION_TASK_STK[DATA_FUSION_STK_SIZE - 1], DATA_FUSION_TASK_PRIO);

    OSStart();
}

#if 0

// 内环任务, 运行频率可以和姿态更新频率保持一致。
void Quadcopter_Imple_Task(void* pdata)
{
    while (1) {
        //        OSSemPend(PIDSem, 100, NULL);
        PID_Cycle(&Roll_w_PID);
        PID_Cycle(&Pitch_w_PID);
        PID_Cycle(&Yaw_w_PID);

        /* 给电机分配占空比 */
        // 电机输出足够起飞且有一定余量做动作时, 设置PWM
        // TODO: 电机输出分配要根据飞行模式(十字或X), 正反桨位置来调整, 本代码适用飞行模式是X飞行模式, 仅供参考
        if (Base_CCR > (Fly_Thre_CCR + Margin_CCR)) {
            // 3201
            //
            Servo_PWM[3] = Limit(Base_CCR + Roll_w_PID.Output + Pitch_w_PID.Output - Yaw_w_PID.Output, BASE_MIN, BASE_MAX);  // x+  y+
            Servo_PWM[0] = Limit(Base_CCR - Roll_w_PID.Output + Pitch_w_PID.Output + Yaw_w_PID.Output, BASE_MIN, BASE_MAX);  // x-  y+
            Servo_PWM[2] = Limit(Base_CCR + Roll_w_PID.Output - Pitch_w_PID.Output + Yaw_w_PID.Output, BASE_MIN, BASE_MAX);  // x+  y-
            Servo_PWM[1] = Limit(Base_CCR - Roll_w_PID.Output - Pitch_w_PID.Output - Yaw_w_PID.Output, BASE_MIN, BASE_MAX);  // x-  y-
        }
        // 电机输出不够, 不允许起飞
        else {
            for (int i = 0; i < 4; i++)
                Servo_PWM[i] = 0;
        }

        if (Roll > 65 || Roll < -65 || Pitch > 65 || Pitch < -65) {
            for (int i = 0; i < 4; i++)
                Servo_PWM[i] = BASE_MIN;
        }
        // TODO: 将Servo_PWM中的值写入控制电机的TIM相应通道的CCR寄存器
        // 未实现, 仅作为逻辑示意
        if (Base_CCR <= 550) {
            for (int i = 0; i < 4; i++) {
                Servo_PWM[i] = 400;
            }
        }
        for (int i = 0; i < 4; i++) {
            Motor_Set(Servo_PWM[i], i + 1);
        }

        OSTimeDly(5);
    }
}

// 外环任务, 运行频率要小于内环频率, 运行频率小于内环的一半计算才有意义, 本人设定 外环频率 : 内环频率 = 1 : 5
void Quadcopter_Control_Task(void* pdata)
{
    INT8U err;
    while (1) {
        PID_Cycle(&Roll_PID);
        PID_Cycle(&Pitch_PID);
        PID_Cycle(&Yaw_PID);
        for (int i = 0; i < 5; i++) {
            OSSemPost(PIDSem);
        }
        OSTimeDly(25);
    }
}

void GY86_TASK(void* pdata)
{
    while (1) {
        Read_Accel_MPS();
        Read_Gyro_DPS();
        Read_Mag_Gs();

        OSSemPost(SensorSem);
        OSTimeDly(5);
    }
}
void DATA_FUSION_TASK(void* pdata)
{
    INT8U err;
    while (1) {
        //			OSSemPend(I2CSem,2,&err);
        Read_Accel_MPS();
        Read_Gyro_DPS();
        Read_Mag_Gs();
        //#if OS_CRITICAL_METHOD == 3u /* Allocate storage for CPU status register */
        //    OS_CPU_SR cpu_sr = 0u;
        //#endif
        //		OS_ENTER_CRITICAL();
#    if DMP
// mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
#    else

        //        IMUupdate(Gyro_dps[0], Gyro_dps[1], Gyro_dps[2], Acel_mps[0], Acel_mps[1], Acel_mps[2], Mag_gs[0], Mag_gs[1], Mag_gs[2]);
        Attitude_Update(Gyro_dps[0], Gyro_dps[1], Gyro_dps[2], Acel_mps[0], Acel_mps[1], Acel_mps[2], Mag_gs[0], Mag_gs[1], Mag_gs[2]);
// origin        IMUupdate(Gyro_dps[0], Gyro_dps[1], Gyro_dps[2], Acel_mps[0], Acel_mps[1], Acel_mps[2], Mag_gs[0], Mag_gs[2], Mag_gs[1]);
//  MadgwickAHRSupdate(Gyro_dps[1],Gyro_dps[0],-Gyro_dps[2],-Acel_mps[1],-Acel_mps[0],Acel_mps[2],-Mag_gs[2],-Mag_gs[0],Mag_gs[1]);
#    endif
        //			OSSemPost(I2CSem);
        //			OS_EXIT_CRITICAL();
        OSTimeDly(5);
    }
}
#endif
