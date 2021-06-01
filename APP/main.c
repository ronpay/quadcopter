#include "os_cpu.h"
#include "ucos_ii.h"
#include "stm32f4xx.h"

#include "LED.h"
#include "hm10.h"
#include "hmc5883l.h"
#include "motor.h"
#include "mpu6050.h"
#include "oled.h"
#include "receiver.h"
#include "stm32f4xx.h"
#include "sysTick.h"

uint32_t Task_Delay[NumOfTask] = {0};

extern u8 hm_flag;
short Duty[6];

//GY86 所需要的数组
short Acel[3];
short Gyro[3];
int Temp;
short Me[3];

#define HM10_TASK_PRIO 6
#define RECEIVER_TASK_PRIO 3
#define GY86_TASK_PRIO 4
#define MOTOR_TASK_PRIO 2
#define OLED_TASK_PRIO 5
#define LED_TASK_PRIO 7

#define HM10_STK_SIZE 128
#define RECEIVER_STK_SIZE 128
#define GY86_STK_SIZE 1024
#define MOTOR_STK_SIZE 128
#define OLED_STK_SIZE 128
#define LED_STK_SIZE 128

OS_STK HM10_TASK_STK[HM10_STK_SIZE];
__align(8) static OS_STK RECEIVER_TASK_STK[RECEIVER_STK_SIZE];
__align(8) static OS_STK GY86_TASK_STK[GY86_STK_SIZE];
OS_STK MOTOR_TASK_STK[MOTOR_STK_SIZE];
__align(8) static OS_STK OLED_TASK_STK[OLED_STK_SIZE];
OS_STK LED_TASK_STK[LED_STK_SIZE];

void HM10_TASK(void *pdata){
}

void RECEIVER_TASK(void *pdata){
	while(1){
		if (hm_flag == '1') {
			for (int i = 0; i < 6; i++) {
//				if (Duty[i] > 0.01) {
//					printf("CH%i:%.2f %% \n", i + 1, 100 * Duty[i]/10000);
					printf("CH%i:%i %% \n", i + 1, Duty[i]/100);
//				}
			}
		}
		OSTimeDly(500);
	}
}

void GY86_TASK(void *pdata){

	while(1){
		MPU6050ReadAcc(Acel);
		MPU6050ReadGyro(Gyro);
//		MPU6050_ReturnTemp(&Temp);
		HMC5884LReadMe(Me);
		if(hm_flag == '0'){
			printf("Acceleration: %8d%8d%8d\n", Acel[0], Acel[1], Acel[2]);
			printf("Gyroscope:    %8d%8d%8d\n", Gyro[0], Gyro[1], Gyro[2]);
//			printf("Temperature:  %8.2f\n", Temp);
			printf("MagneticField:%8d%8d%8d\n", Me[0], Me[1], Me[2]);
		}
		OSTimeDly(200);
	}
}

void MOTOR_TASK(void *pdata){
	while(1){
		for (int i = 0; i < 4; i++) {
			Motor_Set(Duty[i], i + 1);
		}
		OSTimeDly(200);
	}
}

void OLED_TASK(void *pdata){
	while(1){
		OLED_Show_3num(Acel[0], Acel[1], Acel[2], 1);
		OLED_Show_3num(Gyro[0], Gyro[1], Gyro[2], 0);
//		OLED_ShowNum(24, 7, Temp, 2, 12);
		OLED_Show_3num(Me[0], Me[1], Me[2], 2);
//		OLED_Show_3num(100 * Duty[0], 100 * Duty[1], 100 * Duty[2], 3);
//		OLED_Show_3num(100 * Duty[3],100 * Duty[4], 100 * Duty[5], 4);
		OSTimeDly(500);
	}
}

void LED_TASK(void *pdata){
	int x;
	while(1){
		x++;
		printf("%i\n",x);
		printf("begin\n");
		printf("x\n");
		printf("x\n");
		printf("x\n");
		printf("end\n");
		OSTimeDly(1000);
	}
}


////设置任务优先级
//#define LED_ON_TASK_PRIO 7
//#define LED_OFF_TASK_PRIO 8
////设置任务堆栈大小
//#define LED_ON_STK_SIZE 128
//#define LED_OFF_STK_SIZE 128
////任务堆栈
//OS_STK LED_ON_TASK_STK[LED_ON_STK_SIZE];
//OS_STK LED_OFF_TASK_STK[LED_OFF_STK_SIZE];

////亮灯
//void LED_ON_TASK(void *pdata)
//{
//    while(1)
//	{
//		*(unsigned int *)0x40020014 |= 1<<5;
//		OSTimeDly(100);
//	}
//}

////灭灯
//void LED_OFF_TASK(void *pdata)
//{
//    while(1)
//	{
//		*(unsigned int *)0x40020014 &= ~(1<<5);
//		OSTimeDly(200);
//	}
//}

//使能systick中断为1ms
void SystemClock_Config()
{	
	 SysTick_Config(SystemCoreClock/1000);
}

void LED_Init(void)
{    	 
	// 配置RCC寄存器，使能GPIOA时钟
	*(unsigned int *)0x40023830 |= 1;
	
	// 配置MODER寄存器，配置为通用输出
	*(unsigned int *)0x40020000 |= (1<<(5*2));
}

int main()
{	
	SystemClock_Config();
	RCC_ClocksTypeDef get_rcc_clock;
	RCC_GetClocksFreq(&get_rcc_clock);
//	SysTick_Init();
	
    HM10_Config();
	
	OLED_Config();
    OLED_Init();
	
	OLED_CLS();
    OLED_ShowStr(0, 4, (unsigned char*)"Hello World!", 2);  //测试8*16字符
    Delay_s(1);
    OLED_CLS();  //清屏
	
	Motor_Config();
    Motor_Unlock();
	
	Receiver_Config();
	
	GY86_Init();

//	LED_Init();
	OSInit();
//	OSTaskCreate(LED_ON_TASK, (void *)0, (void *)&LED_ON_TASK_STK[LED_ON_STK_SIZE - 1], LED_ON_TASK_PRIO);
//  OSTaskCreate(LED_OFF_TASK, (void *)0, (void *)&LED_OFF_TASK_STK[LED_OFF_STK_SIZE - 1], LED_OFF_TASK_PRIO);

//	OSTaskCreate(HM10_TASK, (void *)0, (void *)&HM10_TASK_STK[HM10_STK_SIZE - 1], HM10_TASK_PRIO);
	OSTaskCreate(RECEIVER_TASK, (void *)0, (void *)&RECEIVER_TASK_STK[RECEIVER_STK_SIZE - 1], RECEIVER_TASK_PRIO);
	OSTaskCreate(GY86_TASK, (void *)0, (void *)&GY86_TASK_STK[GY86_STK_SIZE - 1], GY86_TASK_PRIO);
	OSTaskCreate(MOTOR_TASK, (void *)0, (void *)&MOTOR_TASK_STK[MOTOR_STK_SIZE - 1], MOTOR_TASK_PRIO);
	OSTaskCreate(OLED_TASK, (void *)0, (void *)&OLED_TASK_STK[OLED_STK_SIZE - 1], OLED_TASK_PRIO);
//	OSTaskCreate(LED_TASK, (void *)0, (void *)&LED_TASK_STK[LED_STK_SIZE - 1], LED_TASK_PRIO);
	
	
	OSStart();
	
}
