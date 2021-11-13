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

OS_EVENT* SensorSem;
OS_EVENT* ReceiverSem;
extern u8 hm_flag;

// GY86 所需要的数组
extern float          Acel_mps[3];
extern short          Acel[3];
extern float          Gyro_dps[3];
extern short          Gyro[3];
extern short          Mag[3];
extern volatile float Mag_gs[3];
extern volatile float Temp;
extern volatile float pitch, roll, yaw;
extern uint16_t       Duty[6];
// for pid
extern desired_t angledesired;
extern desired_t ratedesired;
extern set_t     setpoint;
extern sensor_t  sensordata;
extern Attitude  state;
extern control_t control;
extern PID_type  PID_angle_roll;
extern PID_type  PID_angle_pitch;
extern PID_type  PID_angle_yaw;
extern PID_type  PID_rate_roll;
extern PID_type  PID_rate_pitch;
extern PID_type  PID_rate_yaw;

#define PID_TASK_PRIO 3
#define RECEIVER_TASK_PRIO 15
#define GY86_TASK_PRIO 7
#define DATA_TRANSFER_TASK_PRIO 62
#define DATA_FUSION_TASK_PRIO 61

#define PID_STK_SIZE 128
#define GY86_STK_SIZE 128
#define DATA_TRANSFER_STK_SIZE 128
#define DATA_FUSION_STK_SIZE 128

OS_STK PID_TASK_STK[PID_STK_SIZE];
OS_STK GY86_TASK_STK[GY86_STK_SIZE];
OS_STK DATA_TRANSFER_TASK_STK[DATA_TRANSFER_STK_SIZE];
OS_STK DATA_FUSION_TASK_STK[DATA_FUSION_STK_SIZE];

#define DMP 0

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
}

void DATA_FUSION_TASK(void* pdata)
{
    INT8U err;
    while (1) {
        IMUupdate(Gyro_dps[0], Gyro_dps[1], Gyro_dps[2], Acel_mps[0], Acel_mps[1], Acel_mps[2], Mag_gs[0], Mag_gs[2], Mag_gs[1]);
        // MadgwickAHRSupdate(Gyro_dps[1],Gyro_dps[0],-Gyro_dps[2],-Acel_mps[1],-Acel_mps[0],Acel_mps[2],-Mag_gs[2],-Mag_gs[0],Mag_gs[1]);
        OSTimeDly(5);
    }
}

void DATA_TRANSFER_TASK(void* pdata)
{
    INT8U err;
    while (1) {
        ANO_DT_Send_Status(roll, pitch, yaw, 0);
        ANO_DT_Send_Senser(Acel_mps[0], Acel_mps[1], Acel_mps[2], Gyro_dps[0], Gyro_dps[1], Gyro_dps[2], 0);
        ANO_DT_Send_Senser2(Mag_gs[0], Mag_gs[2], Mag_gs[1], 0, 0, 0, 0);
        ANO_DT_Send_PWM(Duty[0], Duty[1], Duty[2], Duty[3]);

        OSTimeDly(20);
    }
}

void GY86_TASK(void* pdata)
{
    INT8U err;
    while (1) {
        OSSemPend(SensorSem, 1000, &err);

        Read_Accel_MPS();
        Read_Gyro_DPS();
        Read_Mag_Gs();

//		MPU6050_ReturnTemp(&Temp);
#if DMP
// mpu_dmp_get_data(&pitch,&roll,&yaw);
#endif
        OSSemPost(SensorSem);

        OSTimeDly(10);
    }
}

void MOTOR_TASK(void* pdata)
{
    INT8U err;
    while (1) {
        OSSemPend(ReceiverSem, 1000, &err);
        for (int i = 0; i < 4; i++) {
            Motor_Set(Duty[i], i + 1);
        }
        OSSemPost(ReceiverSem);
        OSTimeDly(50);
    }
}

void PID_TASK(void* pdata)
{
    int time;  //暂无
    int motor[4];
    controlinit(&control);
    while (1) {
        //获取遥控器信息
        setpoint.throttle = Duty[0] - 1500;
        setpoint.yaw      = Duty[1] - 1500;
        setpoint.pitch    = Duty[2] - 1500;
        setpoint.roll     = Duty[3] - 1500;

        //获取传感器数据
        Read_Accel_MPS();
        Read_Gyro_DPS();
        Read_Mag_Gs();

        sensordata.acc.x = Acel_mps[0];
        sensordata.acc.y = Acel_mps[1];
        sensordata.acc.z = Acel_mps[2];

        sensordata.gyro.x = Gyro_dps[0];
        sensordata.gyro.y = Gyro_dps[1];
        sensordata.gyro.z = Gyro_dps[2];

        sensordata.mag.x = Mag_gs[0];
        sensordata.mag.y = Mag_gs[1];
        sensordata.mag.z = Mag_gs[2];

        //进行姿态解算

        state.yaw   = yaw;
        state.roll  = roll;
        state.pitch = pitch;

        IMUupdate(Gyro_dps[0], Gyro_dps[1], Gyro_dps[2], Acel_mps[0], Acel_mps[1], Acel_mps[2], Mag_gs[0], Mag_gs[2], Mag_gs[1]);

        //			if(TIME_TODO(ATTITUDE_ESTIMAT_RATE,time)){
        //				imuUpdate(sensordata.acc,sensordata.gyro,&state,ATTITUDE_ESTIMAT_DT);
        //				float x=state.pitch;
        //				state.pitch=state.roll;
        //				state.roll=x;
        //			}

        //计算点击控制
        attitudecontrol(&control, &setpoint, &state, time, &sensordata);
        //设置电机转速
        // MOTOR_control(&control);
        //
        motor[0] = LIM(control.throttle - control.roll - control.pitch + control.yaw);
        motor[1] = LIM(control.throttle - control.roll + control.pitch - control.yaw);
        motor[2] = LIM(control.throttle + control.roll + control.pitch + control.yaw);
        motor[3] = LIM(control.throttle + control.roll - control.pitch - control.yaw);

        for (int i = 0; i < 4; i++) {
            Motor_Set(motor[i], i + 1);
        }

        //			OSQPost(mm,&remotor[ptr]);
        //			ptr=(ptr+1)%128;
        //			else {
        //			motor[0]=motor[1]=motor[2]=motor[3]=1000;
        //					MOTOR_lock();
        //			}
        OSTimeDly(1);  //任务周期1ms
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

    //    SensorSem=OSSemCreate(1);
    //	ReceiverSem=OSSemCreate(1);
    OSTaskCreate(PID_TASK, (void*)0, (void*)&PID_TASK_STK[PID_STK_SIZE - 1], PID_TASK_PRIO);
    //    OSTaskCreate(GY86_TASK, (void*)0, (void*)&GY86_TASK_STK[GY86_STK_SIZE - 1], GY86_TASK_PRIO);
    OSTaskCreate(DATA_TRANSFER_TASK, (void*)0, (void*)&DATA_TRANSFER_TASK_STK[DATA_TRANSFER_STK_SIZE - 1], DATA_TRANSFER_TASK_PRIO);
    //    OSTaskCreate(DATA_FUSION_TASK, (void*)0, (void*)&DATA_FUSION_TASK_STK[DATA_FUSION_STK_SIZE - 1], DATA_FUSION_TASK_PRIO);

    OSStart();
}
