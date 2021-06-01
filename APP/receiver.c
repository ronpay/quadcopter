#include "receiver.h"

void Receiver_Config(void) { TIM3_Cap_Init(0xFFFF, 84 - 1); }
