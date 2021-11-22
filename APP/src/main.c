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
#include "receiver.h"
#include "sysTick.h"

OS_EVENT* PIDSem;
OS_EVENT* SensorSem;
// OS_EVENT* ReceiverSem;
extern u8 hm_flag;

// GY86 所需要的数组
extern float          Acel_mps[3];
extern short          Acel[3];
extern float          Gyro_dps[3];
extern short          Gyro[3];
extern short          Mag[3];
extern float          Mag_gs[3];
extern float          Temp;
extern volatile float Pitch, Roll, Yaw;
extern uint16_t       Duty[6];
// for pid
extern PID_TYPE       Roll_w_PID, Pitch_w_PID, Yaw_w_PID;
extern PID_TYPE       Roll_PID, Pitch_PID, Yaw_PID;
extern volatile float Pitch_T, Roll_T, Yaw_T;

volatile uint16_t Base_CCR;          // 遥控器提供的基准油门
uint16_t          Fly_Thre_CCR = 0;  // 无人机起飞的最低油门
uint16_t          Margin_CCR   = 0;  // 做姿态需要的油门余量, 防止掉下来

uint16_t Servo_PWM[4] = {0};

#define PID_INNER_TASK_PRIO 3
#define PID_OUTER_TASK_PRIO 4
#define RECEIVER_TASK_PRIO 15
#define GY86_TASK_PRIO 7
#define DATA_TRANSFER_TASK_PRIO 62
#define DATA_FUSION_TASK_PRIO 61

#define PID_INNER_STK_SIZE 128
#define PID_OUTER_STK_SIZE 128
#define GY86_STK_SIZE 128
#define DATA_TRANSFER_STK_SIZE 128
#define DATA_FUSION_STK_SIZE 128

OS_STK PID_INNER_TASK_STK[PID_INNER_STK_SIZE];
OS_STK PID_OUTER_TASK_STK[PID_OUTER_STK_SIZE];
OS_STK GY86_TASK_STK[GY86_STK_SIZE];
OS_STK DATA_TRANSFER_TASK_STK[DATA_TRANSFER_STK_SIZE];
OS_STK DATA_FUSION_TASK_STK[DATA_FUSION_STK_SIZE];

#define DMP 0
#define MOTOR 0
#define RECEIVER 0
void INIT_TASK(void* pdata)
{
    HM10_Config();

    OLED_Config();
    OLED_Init();
    OLED_CLS();
    OLED_ShowStr(0, 4, (unsigned char*)"Loding........", 2);
    Delay_s(1);

    Motor_Config();
    Motor_Unlock();
    Receiver_Config();

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

    Delay_s(2);
    // without dmp
    GY86_Init();
    GY86_SelfTest();

#endif
    OLED_CLS();

    void Gesture_PID_Init();
}

void DATA_FUSION_TASK(void* pdata)
{
    INT8U err;
    while (1) {
        OSSemPend(SensorSem, 500, &err);
        IMUupdate(Gyro_dps[0], Gyro_dps[1], Gyro_dps[2], Acel_mps[0], Acel_mps[1], Acel_mps[2], Mag_gs[0], Mag_gs[2], Mag_gs[1]);
        // MadgwickAHRSupdate(Gyro_dps[1],Gyro_dps[0],-Gyro_dps[2],-Acel_mps[1],-Acel_mps[0],Acel_mps[2],-Mag_gs[2],-Mag_gs[0],Mag_gs[1]);
        OSTimeDly(5);
    }
}

void DATA_TRANSFER_TASK(void* pdata)
{
    //    INT8U err;
    while (1) {
        ANO_DT_Send_Status(Roll, Pitch, Yaw, 0);
        /* 目标姿态，通过遥控器的数据直接算出 */
        ANO_DT_Send_Target_Status(Roll_T, Pitch_T, Yaw_T);
        ANO_DT_Send_Senser(Acel_mps[0], Acel_mps[1], Acel_mps[2], Gyro_dps[0], Gyro_dps[1], Gyro_dps[2], 0);
        ANO_DT_Send_Senser2(Mag_gs[0], Mag_gs[2], Mag_gs[1], 0, 0, 0, 0);
        /* 遥控器四通道的量 */
        ANO_DT_Send_PWM(Duty[0], Duty[1], Duty[2], Duty[3]);
        /* PID 计算出来的三个角度的PWM控制量 */
        ANO_DT_Send_Control_Status(Roll_w_PID.Output, Pitch_w_PID.Output, Base_CCR, Yaw_w_PID.Output);

        OSTimeDly(20);
    }
}

void GY86_TASK(void* pdata)
{
    //    INT8U err;
    while (1) {
        Read_Accel_MPS();
        Read_Gyro_DPS();
        Read_Mag_Gs();

        OSSemPost(SensorSem);

//		MPU6050_ReturnTemp(&Temp);
#if DMP
// mpu_dmp_get_data(&pitch,&roll,&yaw);
#endif
        // OSSemPost(SensorSem);

        OSTimeDly(10);
    }
}

// 内环任务, 运行频率可以和姿态更新频率保持一致。
void Quadcopter_Imple_Task(void* pdata)
{
    while (1) {
        PID_Cycle(&Roll_w_PID);
        PID_Cycle(&Pitch_w_PID);
        PID_Cycle(&Yaw_w_PID);

        /* 给电机分配占空比 */
        // 电机输出足够起飞且有一定余量做动作时, 设置PWM
        // TODO: 电机输出分配要根据飞行模式(十字或X), 正反桨位置来调整, 本代码适用飞行模式是X飞行模式, 仅供参考
        if (Base_CCR > (Fly_Thre_CCR + Margin_CCR)) {
            Servo_PWM[0] = Base_CCR + Roll_w_PID.Output + Pitch_w_PID.Output - Yaw_w_PID.Output;  // x负方向 y负方向
            Servo_PWM[1] = Base_CCR - Roll_w_PID.Output + Pitch_w_PID.Output + Yaw_w_PID.Output;  // x负方向 y正方向
            Servo_PWM[2] = Base_CCR + Roll_w_PID.Output - Pitch_w_PID.Output + Yaw_w_PID.Output;  // x正方向 y负方向
            Servo_PWM[3] = Base_CCR - Roll_w_PID.Output - Pitch_w_PID.Output - Yaw_w_PID.Output;  // x正方向 y正方向
        }
        // 电机输出不够, 不允许起飞
        else {
            for (int i = 0; i < 4; i++)
                Servo_PWM[i] = 0;
        }

        // TODO: 将Servo_PWM中的值写入控制电机的TIM相应通道的CCR寄存器
        // 未实现, 仅作为逻辑示意
        for (int i = 0; i < 4; i++) {
            Motor_Set(Servo_PWM[i], i + 1);
        }
        OSSemPost(PIDSem);
        OSTimeDly(3);
    }
}

// 外环任务, 运行频率要小于内环频率, 运行频率小于内环的一半计算才有意义, 本人设定 外环频率 : 内环频率 = 1 : 5
void Quadcopter_Control_Task(void* pdata)
{
    INT8U err;
    while (1) {
        for (int i = 0; i < 5; i++) {
            OSSemPend(PIDSem, 1000, &err);
        }
        PID_Cycle(&Roll_PID);
        PID_Cycle(&Pitch_PID);
        PID_Cycle(&Yaw_PID);
        OSTimeDly(3);
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
    OSTaskCreate(Quadcopter_Imple_Task, (void*)0, (void*)&PID_OUTER_TASK_STK[PID_OUTER_STK_SIZE - 1], PID_OUTER_TASK_PRIO);
    OSTaskCreate(Quadcopter_Control_Task, (void*)0, (void*)&PID_INNER_TASK_STK[PID_INNER_STK_SIZE - 1], PID_INNER_TASK_PRIO);
    //    OSTaskCreate(GY86_TASK, (void*)0, (void*)&GY86_TASK_STK[GY86_STK_SIZE - 1], GY86_TASK_PRIO);
    OSTaskCreate(DATA_TRANSFER_TASK, (void*)0, (void*)&DATA_TRANSFER_TASK_STK[DATA_TRANSFER_STK_SIZE - 1], DATA_TRANSFER_TASK_PRIO);
    OSTaskCreate(DATA_FUSION_TASK, (void*)0, (void*)&DATA_FUSION_TASK_STK[DATA_FUSION_STK_SIZE - 1], DATA_FUSION_TASK_PRIO);

    OSStart();
}
