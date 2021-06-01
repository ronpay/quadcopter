#include "includes.h"

OS_STK *OSTaskStkInit (void (*task)(void *p_arg), void *p_arg, OS_STK *ptos, INT16U opt)
{
	OS_STK *p_stk;	
	(void)opt;		   /* opt为保留选项，通常不使用。进行类型转化避免报错*/
	p_stk = ptos;       /*传值调用，避免改变原指针*/
                                                       /*设定通用寄存器的初始值为寄存器号，便于调试时观察出栈时对应值是否匹配*/
    *(p_stk) = (OS_STK)0x01000000uL;                   /* xPSR  thumb状态                                      */
    *(--p_stk) = (OS_STK)task;                         /* Entry Point                                          */
    *(--p_stk) = (OS_STK)OS_TaskReturn;                /* R14 (LR)                                             */
    *(--p_stk) = (OS_STK)0x12uL;                       /* R12                                                  */
    *(--p_stk) = (OS_STK)0x3uL;                        /* R3                                                   */
    *(--p_stk) = (OS_STK)0x2uL;                        /* R2                                                   */
    *(--p_stk) = (OS_STK)0x1uL;                        /* R1                                                   */
    *(--p_stk) = (OS_STK)p_arg;                        /* R0 : argument                                        */

    *(--p_stk) = (OS_STK)0x11uL;                       /* R11                                                  */
    *(--p_stk) = (OS_STK)0x10uL;                       /* R10                                                  */
    *(--p_stk) = (OS_STK)0x9uL;                        /* R9                                                   */
    *(--p_stk) = (OS_STK)0x8uL;                        /* R8                                                   */
    *(--p_stk) = (OS_STK)0x7uL;                        /* R7                                                   */
    *(--p_stk) = (OS_STK)0x6uL;                        /* R6                                                   */
    *(--p_stk) = (OS_STK)0x5uL;                        /* R5                                                   */
    *(--p_stk) = (OS_STK)0x4uL;                        /* R4                                                   */

	return (p_stk);
}

void  OSInitHookBegin (void)
{

}

void  OSInitHookEnd (void)
{

}

void  OSTaskCreateHook (OS_TCB *ptcb)
{

}

void  OSTaskDelHook (OS_TCB *ptcb)
{

}

void  OSTaskIdleHook (void)
{

}

void  OSTaskStatHook (void)
{

}

void  OSTaskSwHook (void)
{

}

void  OSTCBInitHook (OS_TCB *ptcb)
{

}

void  OSTimeTickHook (void)
{

}

void OSTaskReturnHook(OS_TCB *ptcb)
{

}
