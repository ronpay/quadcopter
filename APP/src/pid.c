/**
 * @file  		pid.c
 * @brief 		PID封装库
 * @author   	Haozhe Tang
 * @date     	2021-7-22
 * @version   	A001
 * @copyright 	Haozhe Tang
 */

#include "pid.h"

void PID_Init(p_PID_TYPE PID)
{
    PID->Set_PID_Arg_Handler = Set_PID_Arg;
    PID->Update_Err_Handler  = Update_Err;
}

void Set_PID_Arg(p_PID_TYPE PID, Gain_Type* K_pid)
{
    PID->Kp = K_pid[0];
    PID->Ki = K_pid[1];
    PID->Kd = K_pid[2];
}

// 计算位置式PID
void Calculate_Position_PID_Output(p_PID_TYPE PID)
{
    PID->Output = PID->Kp * PID->Err + PID->Ki * PID->ErrAccu + PID->Kd * PID->ErrDiff;
    //	printf("PID->Output %d\n", PID->Output);

    if (PID->Output > PID->Output_Max)
        PID->Output = PID->Output_Max;
    else if (PID->Output < (-1) * PID->Output_Max)
        PID->Output = (-1) * PID->Output_Max;
}

// 计算增量式PID
void Calculate_Delta_PID_Output(p_PID_TYPE PID)
{
    PID->Delta = PID->Kp * PID->ErrDiff + PID->Ki * PID->Err + PID->Kd * (PID->Err - 2 * PID->LastErr + PID->PrevErr);
    //	printf("PID->Delta %d\n", PID->Delta);

    PID->Output = PID->Output + PID->Delta;
    //	printf("PID->Output %d\n", PID->Output);
}

void PID_Cycle(p_PID_TYPE PID)
{
    // 更新反馈值
    PID->Update_Feedback_Handler(PID);
    //	printf("PID->Feedback %f\n", PID->Feedback);

    // 更新目标值
    PID->Update_Target_Handler(PID);
    //	printf("PID->Target %f\n", PID->Target);

    // 更新误差值
    PID->Update_Err_Handler(PID);
    //	printf("PID->Err %f\n", PID->Err);

    // 计算输出值
    PID->Calculate_Output_Handler(PID);
    //	printf("PID->Output %d\n", PID->Output);
}

void Update_Err(p_PID_TYPE PID)
{
    PID->PrevErr = PID->LastErr;
    //	printf("PID->PrevErr %f\n", PID->PrevErr);

    PID->LastErr = PID->Err;
    //	printf("PID->LastErr %f\n", PID->LastErr);

    PID->Err = PID->Target - PID->Feedback;
    //	printf("PID->Err %f\n", PID->Err);

    PID->ErrDiff = PID->Err - PID->LastErr;
    //	printf("PID->ErrDiff %f\n", PID->ErrDiff);

    if (PID->Err > PID->Err_Max)
        PID->ErrAccu += PID->Err_Max;
    else if (PID->Err < -1 * PID->Err_Max)
        PID->ErrAccu += -1 * PID->Err_Max;
    else
        PID->ErrAccu += PID->Err;

    // 积分限幅
    if (PID->Err > PID->Accu_Err_Max)
        PID->ErrAccu = PID->Accu_Err_Max;
    else if (PID->Err < -1 * PID->Accu_Err_Max)
        PID->ErrAccu = -1 * PID->Accu_Err_Max;

    //	printf("PID->ErrAccu %f\n", PID->ErrAccu);
}

int Limit(int target,int min,int max){
	return target<min?min:(target>max?max:target);
}
/**********Ϊ������������λ����Э�鶨��ı���****************************/
// cupΪС��ģʽ�洢��Ҳ�����ڴ洢��ʱ�򣬵�λ������0�ֽڣ���λ��1�ֽ�
//  #define BYTE0(dwTemp)       (*(char *)(&dwTemp))	 //ȡ��int�ͱ����ĵ��ֽ�
//  #define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))	 //	ȡ�洢�ڴ˱�����һ�ڴ��ֽڵ����ݣ����ֽ�
//  #define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
//  #define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

// void PID_DEBUG_ANO_Send(Target_Type target, Feedback_Type* real)
// {
// 	unsigned char data_to_send[23] = {0};
// 	unsigned char i = 0;
// 	unsigned char cnt = 0;
// 	unsigned char sum = 0;

// 	data_to_send[cnt++]=0x88;		 //֡ͷ��88
// 	data_to_send[cnt++]=0xA2;	 	 //�����֣�OXFnֻ�������ݣ�����ʾͼ��0x0n��ʾ���ݺ�ͼ��
// 	data_to_send[cnt++]=0;	     //��Ҫ�������ݵ��ֽ�������ʱ��0�������ڸ�ֵ��

// 	data_to_send[cnt++] = BYTE3(target);	//���ֽ�
// 	data_to_send[cnt++] = BYTE2(target);	//���ֽ�
// 	data_to_send[cnt++] = BYTE1(target);	//���ֽ�
// 	data_to_send[cnt++] = BYTE0(target);	//���ֽ�
// 	data_to_send[cnt++] = BYTE3(*real);
// 	data_to_send[cnt++] = BYTE2(*real);
// 	data_to_send[cnt++] = BYTE1(*real);
// 	data_to_send[cnt++] = BYTE0(*real);

// 	data_to_send[cnt++] = BYTE3(*(real + 1));
// 	data_to_send[cnt++] = BYTE2(*(real + 1));
// 	data_to_send[cnt++] = BYTE1(*(real + 1));
// 	data_to_send[cnt++] = BYTE0(*(real + 1));
// 	data_to_send[cnt++] = 0;
// 	data_to_send[cnt++] = 0;

// 	data_to_send[cnt++] = 0;
// 	data_to_send[cnt++] = 0;

// 	data_to_send[2] = cnt - 3;//���������ݵ��ֽ�����

// 	for(i=0;i<cnt;i++)
// 		sum+=data_to_send[i];

// 	data_to_send[cnt++] = sum;	//����У��λ

// 	HAL_UART_Transmit(&huart1, data_to_send, cnt, 0xffff);
// }
