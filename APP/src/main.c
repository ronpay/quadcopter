#include "data_fusion.h"
#include "data_transfer.h"
#include "filter.h"
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
#include "stm32f4xx.h"
#include "sysTick.h"
#include "ucos_ii.h"

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

volatile uint16_t Base_CCR;            // 遥控器提供的基准油门
uint16_t          Fly_Thre_CCR = 550;  // 无人机起飞的最低油门
uint16_t          Margin_CCR   = 0;    // 做姿态需要的油门余量, 防止掉下来
#define BASE_MIN 400
#define BASE_MAX 1250

uint16_t Servo_PWM[4] = {0};

#define CONTROL_TASK_PRIO 10
#define DATA_TRANSFER_TASK_PRIO 20

#define CONTROL_STK_SIZE 512
#define DATA_TRANSFER_STK_SIZE 128

static OS_STK CONTROL_TASK_STK[CONTROL_STK_SIZE];
static OS_STK DATA_TRANSFER_TASK_STK[DATA_TRANSFER_STK_SIZE];

int   timeCnt = 0;
float T       = 0.005;

/* for filter */
#define GYRO_LPF_CUTOFF_FREQ 30.0f
biquadFilter_t gyroFilterLPF[3];

void INIT_TASK(void* pdata)
{
    HM10_Config();

    Delay_s(1);

#if RECEIVER
    // Receiver_Config();
#endif

#if MOTOR
    Motor_Config();
    Motor_Unlock();
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

    Quat_Init();

#if PID
    Gesture_PID_Init();
#endif

    for (int axis = 0; axis < 3; axis++) {
        biquadFilterInitLPF(&gyroFilterLPF[axis], 200, GYRO_LPF_CUTOFF_FREQ);
    }
}

void DATA_TRANSFER_TASK(void* pdata)
{
    //    INT8U err;
    while (1) {
        ANO_DT_Send_Status(Roll, Pitch, Yaw, 0);

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

        OSTimeDly(25);
    }
}

void CONTROL_TASK(void* pdata)
{
    while (1) {
        Read_Accel_MPS();
        Read_Gyro_DPS();
        Read_Mag_Gs();

        for (int axis = 0; axis < 3; axis++) {
            Gyro_dps[axis] = biquadFilterApply(&gyroFilterLPF[axis], Gyro_dps[axis]);
        }

        Attitude_Update(Gyro_dps[0], Gyro_dps[1], Gyro_dps[2], Acel_mps[0], Acel_mps[1], Acel_mps[2], Mag_gs[0], Mag_gs[1], Mag_gs[2]);

        /* 外环任务 */
        PID_Cycle(&Roll_PID);
        PID_Cycle(&Pitch_PID);
        PID_Cycle(&Yaw_PID);

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

        for (int i = 0; i < 4; i++) {
            Motor_Set(Servo_PWM[i], i + 1);
        }

        OSTimeDly(4);
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
    /* 测试时钟频率 */
    // RCC_ClocksTypeDef get_rcc_clock;
    // RCC_GetClocksFreq(&get_rcc_clock);

    INIT_TASK(NULL);
    Delay_s(1);
    OSInit();

    OSTaskCreate(DATA_TRANSFER_TASK, (void*)0, (void*)&DATA_TRANSFER_TASK_STK[DATA_TRANSFER_STK_SIZE - 1], DATA_TRANSFER_TASK_PRIO);
    OSTaskCreate(CONTROL_TASK, (void*)0, (void*)&CONTROL_TASK_STK[CONTROL_STK_SIZE - 1], CONTROL_TASK_PRIO);

    OSStart();
}
