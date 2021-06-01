#include "hm10.h"

void HM10_Config(void) { USART6_Config(); }

void HM10_Test(void) { USARTTest(USART6); }

///重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f) {
    /* 发送一个字节数据到串口 */
    USART_SendData(USART6, (uint8_t)ch);

    /* 等待发送完毕 */
    while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET)
        ;

    return (ch);
}
