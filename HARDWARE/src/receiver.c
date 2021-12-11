#include "receiver.h"

volatile uint8_t         captureFlag[6] = {0};  //捕获状态
volatile uint32_t        CapVal[6]      = {0};  //第一次下降沿计数值
volatile uint16_t        Duty[6];               // 1000-2000
const int                Cycle = 20000;
extern volatile float    Pitch_T, Roll_T, Yaw_T;
const int                Pitch_Range = 30, Roll_Range = 30, Yaw_Range = 15;
extern volatile uint16_t Base_CCR;

void Receiver_Config(void)
{
    TIM3_Cap_Init(0xFFFF, 84 - 1);
}

void Receiver_IRQ_Handler(void)
{
    int receiverNum;
    // 油门
		//new roll 横滚角
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET) {
        receiverNum = 0;

        switch (captureFlag[receiverNum]) {
            case 0:
                TIM_Cmd(TIM3, DISABLE);
                TIM_SetCounter(TIM3, 0);
                TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Falling);
                TIM_Cmd(TIM3, ENABLE);
                captureFlag[receiverNum] = 1;
                break;
            case 1:
                CapVal[receiverNum] = TIM_GetCapture1(TIM3);
                Duty[receiverNum]   = CapVal[receiverNum] % Cycle;
//								Roll_T              = (Duty[receiverNum] - 1500) / 500.0f * Roll_Range;
						Yaw_T               = (Duty[receiverNum] - 1500) / 500.0f * Yaw_Range;
                TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Rising);
                captureFlag[receiverNum] = 0;

                break;
        }
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1 | TIM_IT_Update);
    }
    /* Yaw 偏航角 */
		// new pitch 俯仰角
    else if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET) {
        receiverNum = 1;

        switch (captureFlag[receiverNum]) {
            case 0:
                TIM_Cmd(TIM3, DISABLE);
                TIM_SetCounter(TIM3, 0);
                TIM_OC2PolarityConfig(TIM3, TIM_ICPolarity_Falling);
                TIM_Cmd(TIM3, ENABLE);
                captureFlag[receiverNum] = 1;
                break;
            case 1:
                CapVal[receiverNum] = TIM_GetCapture2(TIM3);
                Duty[receiverNum]   = CapVal[receiverNum] % Cycle;
								Pitch_T             = (Duty[receiverNum] - 1500) / 500.0f * Pitch_Range;
                TIM_OC2PolarityConfig(TIM3, TIM_ICPolarity_Rising);
                captureFlag[receiverNum] = 0;

                break;
        }
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC2 | TIM_IT_Update);
    }
    /* Pitch 俯仰角 */
		// new 油门
    else if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET) {
        receiverNum = 2;

        switch (captureFlag[receiverNum]) {
            case 0:
                TIM_Cmd(TIM3, DISABLE);
                TIM_SetCounter(TIM3, 0);
                TIM_OC3PolarityConfig(TIM3, TIM_ICPolarity_Falling);
                TIM_Cmd(TIM3, ENABLE);
                captureFlag[receiverNum] = 1;
                break;
            case 1:
                CapVal[receiverNum] = TIM_GetCapture3(TIM3);
                Duty[receiverNum]   = CapVal[receiverNum] % Cycle;
								Base_CCR            = 400+(Duty[receiverNum]-1000)*3/4;
                TIM_OC3PolarityConfig(TIM3, TIM_ICPolarity_Rising);
                captureFlag[receiverNum] = 0;

                break;
        }
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC3 | TIM_IT_Update);
    }
    /* Roll 翻滚角 */
		//new  yaw 偏航角
    else if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET) {
        receiverNum = 3;

        switch (captureFlag[receiverNum]) {
            case 0:
                TIM_Cmd(TIM3, DISABLE);
                TIM_SetCounter(TIM3, 0);
                TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Falling);
                TIM_Cmd(TIM3, ENABLE);
                captureFlag[receiverNum] = 1;
                break;
            case 1:
                CapVal[receiverNum] = TIM_GetCapture4(TIM3);
                Duty[receiverNum]   = CapVal[receiverNum] % Cycle;
//								Yaw_T               = (Duty[receiverNum] - 1500) / 500.0f * Yaw_Range;
						Roll_T              = (Duty[receiverNum] - 1500) / 500.0f * Roll_Range;
                TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Rising);
                captureFlag[receiverNum] = 0;
        }
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC4 | TIM_IT_Update);
    }
}
