//
// Created by 24158 on 2026/6/15.
//

#include "foot_control.h"
#include "Motor_DM.h"

void Class_SingleDogFoot_DM::SingleFoot_Init(FDCAN_HandleTypeDef *m_back_hfdcan, uint16_t m_back_CAN_Rx_ID,uint16_t  m_back_CAN_Tx_ID,
	FDCAN_HandleTypeDef *m_front_hfdcan,uint16_t m_front_CAN_Rx_ID, uint16_t m_front_CAN_Tx_ID,uint8_t L_R_sidex,
	float kp, float kd, float Angle,float Omega,float Torque)
{
	L_R_side = L_R_sidex;

	M_back_hfdcan = m_back_hfdcan;//已完成此前成员变量名和函数传入参数名冲突的修改
	M_back_CAN_Rx_ID = m_back_CAN_Rx_ID;
	M_back_CAN_Tx_ID = m_back_CAN_Tx_ID;

	M_front_hfdcan = m_front_hfdcan;
	M_front_CAN_Rx_ID = m_front_CAN_Rx_ID;
	M_front_CAN_Tx_ID = m_front_CAN_Tx_ID;

	M_back.Init(M_back_hfdcan, M_back_CAN_Rx_ID, M_back_CAN_Tx_ID);
	M_front.Init(M_front_hfdcan, M_front_CAN_Rx_ID, M_front_CAN_Tx_ID);

//HACK：需要重构，目前上电就回0位是不好的做法（直接改用VMC）

	M_back.Set_K_P(kp);
	M_back.Set_K_D(kd);
	M_back.Set_Control_Angle(Angle);
	M_back.Set_Control_Omega(Omega);
	M_back.Set_Control_Torque(Torque);

	M_front.Set_K_P(kp);
	M_front.Set_K_D(kd);
	M_front.Set_Control_Angle(Angle);
	M_front.Set_Control_Omega(Omega);
	M_front.Set_Control_Torque(Torque);

	// mo.CAN_Send_Enter();
	// mi.CAN_Send_Enter();//为什么在这里调用会导致整个回传及发送的帧停滞发送？//因为发送之后一直会进接收中断，导致与之后的HAL_delay(100)反复中断卡死，进而导致init_finished一直不能被置true
//但是为什么会导致总线上一样收不到电机的回传数据？？？


}

int8_t  Class_SingleDogFoot_DM::FootXY_map_MotorAngle()//直角坐标系下的运动学逆解
{

	d_AD = sqrt(X*X + Y*Y);

		if (d_AD > Long_L + Short_L || d_AD < fabs(Long_L - Short_L))
	{
    return 0;
	}

	//提前，防止除0错误

	double a = (Short_L*Short_L + d_AD*d_AD - Long_L*Long_L)/(2.0*Short_L*d_AD);
	if (a > 1.0) a = 1.0;
	if (a < -1.0) a = -1.0;

	//适用于atan2恒为负值时
	motor_angle_raw_front = acos(a) + atan2(Y,X);
	motor_angle_raw_back = -acos(a) + atan2(Y,X);
	//计算角度原数据,均以水平x轴正向为基准,后面要考虑越轴-只有外侧电机要考虑？



	motor_angle_DM_front =  -(motor_angle_raw_front + 0.16802f);//目前串联腿构型的,0.16802f为水平轴到0点的偏移补偿
	motor_angle_DM_back = (-Pi - motor_angle_raw_back) - 0.16802f;


	//限位，不应该直接大跳，保持上一次可行的发送
	if(motor_angle_DM_front > High_limit || motor_angle_DM_front < Low_limit || motor_angle_DM_back > High_limit || motor_angle_DM_back < Low_limit)
	{

		if (motor_angle_DM_front>High_limit)
		{
			motor_angle_DM_front = High_limit;
		}
		if (motor_angle_DM_front<Low_limit)
		{
			motor_angle_DM_front = Low_limit;
		}

		if (motor_angle_DM_back>High_limit)
		{
			motor_angle_DM_back = High_limit;
		}
		if (motor_angle_DM_back<Low_limit)
		{
			motor_angle_DM_back = Low_limit;
		}
		if(L_R_side == Left)
		{
			M_front.Set_Control_Angle(motor_angle_DM_front);
			M_back.Set_Control_Angle(motor_angle_DM_back);

		}else if(L_R_side == Right)
		{
			M_front.Set_Control_Angle(-motor_angle_DM_front);
			M_back.Set_Control_Angle(-motor_angle_DM_back);

		}else
		{
			return 2;
		}
		return 0;
	}else{
		if(L_R_side == Left)
		{
			M_front.Set_Control_Angle(motor_angle_DM_front);
			M_back.Set_Control_Angle(motor_angle_DM_back);
			pre_motor_angle_DM_front = motor_angle_DM_front;
			pre_motor_angle_DM_back = motor_angle_DM_back;
		}else if(L_R_side == Right)
		{
			M_front.Set_Control_Angle(-motor_angle_DM_front);
			M_back.Set_Control_Angle(-motor_angle_DM_back);
			pre_motor_angle_DM_front = motor_angle_DM_front;
			pre_motor_angle_DM_back = motor_angle_DM_back;
		}else
		{
			return 0;
		}
		return 1;
	}




}





HAL_StatusTypeDef Class_SingleDogFoot_DM::Foot_control()
{


	FootXY_map_MotorAngle();

	M_front.RTOS_Send_Callback();
	M_back.RTOS_Send_Callback();
	return HAL_OK;
}

/*
HAL_StatusTypeDef Bluetooth_control(UART_HandleTypeDef *huartx)
{

if (uart1_cmd_ready)
{
    uart1_cmd_ready = 0;

    // 将接收到的int值转换为short（如果需要）
    speed = (int16_t)uart1_rx_value;
    Motor_SetTargetSpeed(&motor1, speed);

    // 构造发送数据包（11字节）：帧头 + 原数据(4+4字节) + 校验和 + 帧尾
    uint8_t tx_buffer[11];
    tx_buffer[0] = 0xA5;                          // 帧头
    // 原数据（小端模式，与接收解析一致）
    tx_buffer[1] = (uint8_t)(uart1_rx_value & 0xFF);？？？
    tx_buffer[2] = (uint8_t)((uart1_rx_value >> 8) & 0xFF);
    tx_buffer[3] = (uint8_t)((uart1_rx_value >> 16) & 0xFF);
    tx_buffer[4] = (uint8_t)((uart1_rx_value >> 24) & 0xFF);
    // 校验和：原数据4字节之和的低8位
    tx_buffer[5] = tx_buffer[1] + tx_buffer[2] + tx_buffer[3] + tx_buffer[4];
    tx_buffer[6] = 0x5A;                          // 帧尾

    HAL_UART_Transmit(huartx, tx_buffer, 7, 100);
}
}
*/

// void DM_Motor_FootData_SendVOFA_Firewater(Foot *footx,UART_HandleTypeDef *huart)
// {
// 		uint32_t current_time = HAL_GetTick();
// 		//设置UART发送缓冲
// 		char buffer[100];
// 		int len;
// 		//将电机回传数据使用串口发送到VOFA，使用firewater协议
// 		if((current_time - foot_uart_last_time) > 100)
// 		{
//
//
// 		len = sprintf(buffer, "%lf,%lf,%lf,%lf,%lf,%lf\n",
// 			motor_angle_DM_inside,motor_angle_DM_outside,
// 			motor_angle_raw_inside,motor_angle_raw_outside,
// 			X,Y);
// 		HAL_UART_Transmit(huart, (uint8_t *)buffer, len, 1000);
// 		foot_uart_last_time = HAL_GetTick();
// 		}
//
//
// }
