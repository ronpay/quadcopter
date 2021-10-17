#include "includes.h"

OS_STK *OSTaskStkInit (void (*task)(void *p_arg), void *p_arg, OS_STK *ptos, INT16U opt)
{
	OS_STK *p_stk;	
	(void)opt;		   /* opt为保留选项，通常不使用。进行类型转化避免报错*/
	p_stk      = ptos + 1u;        /*传值调用，避免改变原指针*/
	
//	p_stk      = (OS_STK *)((OS_STK)(p_stk) & 0xFFFFFFF8u);
                                                       /* Registers stacked as if auto-saved on exception      */
                                                       /*设定通用寄存器的初始值为寄存器号，便于调试时观察出栈时对应值是否匹配*/
    *(--p_stk) = (OS_STK)0x01000000uL;                          /* xPSR                                                 */
    *(--p_stk) = (OS_STK)task;                                  /* Entry Point                                          */
    *(--p_stk) = (OS_STK)OS_TaskReturn;                         /* R14 (LR)                                             */
    *(--p_stk) = (OS_STK)0x12121212uL;                          /* R12                                                  */
    *(--p_stk) = (OS_STK)0x03030303uL;                          /* R3                                                   */
    *(--p_stk) = (OS_STK)0x02020202uL;                          /* R2                                                   */
    *(--p_stk) = (OS_STK)0x01010101uL;                          /* R1                                                   */
    *(--p_stk) = (OS_STK)p_arg;                                 /* R0 : argument                                        */

                                                                /* Remaining registers saved on process stack           */
    *(--p_stk) = (OS_STK)0x11111111uL;                          /* R11                                                  */
    *(--p_stk) = (OS_STK)0x10101010uL;                          /* R10                                                  */
    *(--p_stk) = (OS_STK)0x09090909uL;                          /* R9                                                   */
    *(--p_stk) = (OS_STK)0x08080808uL;                          /* R8                                                   */
    *(--p_stk) = (OS_STK)0x07070707uL;                          /* R7                                                   */
    *(--p_stk) = (OS_STK)0x06060606uL;                          /* R6                                                   */
    *(--p_stk) = (OS_STK)0x05050505uL;                          /* R5                                                   */
    *(--p_stk) = (OS_STK)0x04040404uL;                          /* R4                                                   */

	#if (OS_CPU_ARM_FP_EN == DEF_ENABLED)
    if ((opt & OS_TASK_OPT_SAVE_FP) != (INT16U)0) {
        *--p_stk = (CPU_STK)0x02000000u;                        /* FPSCR                                                  */
                                                                /* Initialize S0-S31 floating point registers             */
        *--p_stk = (CPU_STK)0x41F80000u;                        /* S31                                                    */
        *--p_stk = (CPU_STK)0x41F00000u;                        /* S30                                                    */
        *--p_stk = (CPU_STK)0x41E80000u;                        /* S29                                                    */
        *--p_stk = (CPU_STK)0x41E00000u;                        /* S28                                                    */
        *--p_stk = (CPU_STK)0x41D80000u;                        /* S27                                                    */
        *--p_stk = (CPU_STK)0x41D00000u;                        /* S26                                                    */
        *--p_stk = (CPU_STK)0x41C80000u;                        /* S25                                                    */
        *--p_stk = (CPU_STK)0x41C00000u;                        /* S24                                                    */
        *--p_stk = (CPU_STK)0x41B80000u;                        /* S23                                                    */
        *--p_stk = (CPU_STK)0x41B00000u;                        /* S22                                                    */
        *--p_stk = (CPU_STK)0x41A80000u;                        /* S21                                                    */
        *--p_stk = (CPU_STK)0x41A00000u;                        /* S20                                                    */
        *--p_stk = (CPU_STK)0x41980000u;                        /* S19                                                    */
        *--p_stk = (CPU_STK)0x41900000u;                        /* S18                                                    */
        *--p_stk = (CPU_STK)0x41880000u;                        /* S17                                                    */
        *--p_stk = (CPU_STK)0x41800000u;                        /* S16                                                    */
        *--p_stk = (CPU_STK)0x41700000u;                        /* S15                                                    */
        *--p_stk = (CPU_STK)0x41600000u;                        /* S14                                                    */
        *--p_stk = (CPU_STK)0x41500000u;                        /* S13                                                    */
        *--p_stk = (CPU_STK)0x41400000u;                        /* S12                                                    */
        *--p_stk = (CPU_STK)0x41300000u;                        /* S11                                                    */
        *--p_stk = (CPU_STK)0x41200000u;                        /* S10                                                    */
        *--p_stk = (CPU_STK)0x41100000u;                        /* S9                                                     */
        *--p_stk = (CPU_STK)0x41000000u;                        /* S8                                                     */
        *--p_stk = (CPU_STK)0x40E00000u;                        /* S7                                                     */
        *--p_stk = (CPU_STK)0x40C00000u;                        /* S6                                                     */
        *--p_stk = (CPU_STK)0x40A00000u;                        /* S5                                                     */
        *--p_stk = (CPU_STK)0x40800000u;                        /* S4                                                     */
        *--p_stk = (CPU_STK)0x40400000u;                        /* S3                                                     */
        *--p_stk = (CPU_STK)0x40000000u;                        /* S2                                                     */
        *--p_stk = (CPU_STK)0x3F800000u;                        /* S1                                                     */
        *--p_stk = (CPU_STK)0x00000000u;                        /* S0                                                     */
    }
#endif

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
