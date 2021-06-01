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
extern double Duty[6];

//设置任务优先级
#define LED_ON_TASK_PRIO 7
#define LED_OFF_TASK_PRIO 8
//设置任务堆栈大小
#define LED_ON_STK_SIZE 128
#define LED_OFF_STK_SIZE 128
//任务堆栈
OS_STK LED_ON_TASK_STK[LED_ON_STK_SIZE];
OS_STK LED_OFF_TASK_STK[LED_OFF_STK_SIZE];

//亮灯
void LED_ON_TASK(void *pdata)
{
    while(1)
	{
		*(unsigned int *)0x40020014 |= 1<<5;
		OSTimeDly(100);
	}
}

//灭灯
void LED_OFF_TASK(void *pdata)
{
    while(1)
	{
		*(unsigned int *)0x40020014 &= ~(1<<5);
		OSTimeDly(200);
	}
}

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
    LED_Init();
	RCC_ClocksTypeDef get_rcc_clock;
    RCC_GetClocksFreq(&get_rcc_clock);

	OSInit();
	OSTaskCreate(LED_ON_TASK, (void *)0, (void *)&LED_ON_TASK_STK[LED_ON_STK_SIZE - 1], LED_ON_TASK_PRIO);
    OSTaskCreate(LED_OFF_TASK, (void *)0, (void *)&LED_OFF_TASK_STK[LED_OFF_STK_SIZE - 1], LED_OFF_TASK_PRIO);
	OSStart();
}
