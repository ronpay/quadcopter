#include "motor.h"
#include "receiver.h"
#include "stm32f4xx.h"
#include "sysTick.h"
#include "tim.h"
#include "ucos_ii.h"

#define ReceiverDelayReady 0
#define NumOfReciver 6

volatile uint32_t CapVal[6] = {0};      //第一次下降沿计数值
volatile uint8_t captureFlag[6] = {0};  //捕获状态
extern short Duty[6] ;
int Receiver_Delay[6] = {0};
u8 hm_flag = '0';
const float Cycle = 20000;
extern uint32_t Task_Delay[];
extern int close_systick_flag;

void USART6_IRQHandler(void) {
    if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) {
        hm_flag = USART_ReceiveData(USART6);
        if (hm_flag == '0') {
            printf("Ble is printing the data of mpu6050\n");
        } else if (hm_flag == '1') {
            printf("Ble is printing the data of reciver\n");
        } else if (hm_flag == '2') {
            
        } else {
            printf("Wrong input, please enter 0 or 1, without CR/LF\n");
        }
    }
}

//void SysTick_Handler(void) {
//    TimingDelay_Decrement();

//    uint8_t i;
//    for (i = 0; i < NumOfTask; i++) {
//        if (Task_Delay[i]) {
//            Task_Delay[i]--;
//        }
//    }
//}

void TIM3_IRQHandler(void) {
    int receiverNum;
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET) {
        receiverNum = 0;

        switch (captureFlag[receiverNum]) {
            case 0:
                TIM_Cmd(TIM3, DISABLE);
                TIM_SetCounter(TIM3, 0);
                TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Falling);
                TIM_Cmd(TIM3, ENABLE);
                captureFlag[receiverNum]=1;
                break;
            case 1:
                CapVal[receiverNum] = TIM_GetCapture1(TIM3);
                Duty[receiverNum] = (float)((int)CapVal[receiverNum] % (int)Cycle);

                //Motor_Set(Duty[receiverNum], receiverNum + 1);
                TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Rising);
                captureFlag[receiverNum] = 0;

                break;
        }
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1 | TIM_IT_Update);
    }

    else if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET) {
        receiverNum = 1;

        switch (captureFlag[receiverNum]) {
            case 0:
                TIM_Cmd(TIM3, DISABLE);
                TIM_SetCounter(TIM3, 0);
                TIM_OC2PolarityConfig(TIM3, TIM_ICPolarity_Falling);
                TIM_Cmd(TIM3, ENABLE);
                captureFlag[receiverNum]=1;
                break;
            case 1:
                CapVal[receiverNum] = TIM_GetCapture2(TIM3);
                Duty[receiverNum] = (float)((int)CapVal[receiverNum] % (int)Cycle);
                //Motor_Set(Duty[receiverNum], receiverNum + 1);
                TIM_OC2PolarityConfig(TIM3, TIM_ICPolarity_Rising);
                captureFlag[receiverNum] = 0;

                break;
        }
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC2 | TIM_IT_Update);
    }

    else if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET) {
        receiverNum = 2;

        switch (captureFlag[receiverNum]) {
            case 0:
                TIM_Cmd(TIM3, DISABLE);
                TIM_SetCounter(TIM3, 0);
                TIM_OC3PolarityConfig(TIM3, TIM_ICPolarity_Falling);
                TIM_Cmd(TIM3, ENABLE);
                captureFlag[receiverNum]=1;
                break;
            case 1:
                CapVal[receiverNum] = TIM_GetCapture3(TIM3);
                Duty[receiverNum] = (float)((int)CapVal[receiverNum] % (int)Cycle);
                //Motor_Set(Duty[receiverNum], receiverNum + 1);
                TIM_OC3PolarityConfig(TIM3, TIM_ICPolarity_Rising);
                captureFlag[receiverNum] = 0;

                break;
        }
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC3 | TIM_IT_Update);
    }

    else if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET) {
        receiverNum = 3;

        switch (captureFlag[receiverNum]) {
            case 0:
                TIM_Cmd(TIM3, DISABLE);
                TIM_SetCounter(TIM3, 0);
                TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Falling);
                TIM_Cmd(TIM3, ENABLE);
                captureFlag[receiverNum]=1;
                break;
            case 1:
                CapVal[receiverNum] = TIM_GetCapture4(TIM3);
                Duty[receiverNum] = (float)((int)CapVal[receiverNum] % (int)Cycle);
                //Motor_Set(Duty[receiverNum], receiverNum + 1);
                TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Rising);
                captureFlag[receiverNum] = 0;
        }
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC4 | TIM_IT_Update);
    }
}

void HardFault_Handler(void) {
    while (1)
        ;
}

void OS_CPU_SysTickHandler(void)
{	
	if (OSRunning == 1) //当os开始运行才跑这个
	{
		OSIntEnter(); //进入中断
		OSTimeTick(); //调用ucos的时钟服务器
		OSIntExit();  //触发任务切换软中断
	}else{
		TimingDelay_Decrement();
	}
}
