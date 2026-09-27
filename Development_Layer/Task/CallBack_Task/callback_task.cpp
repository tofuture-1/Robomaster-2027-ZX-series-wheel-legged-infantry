//
// Created by 谭恩泽 on 2025/10/30.
//

#include "callback_task.h"

#include "drv_uart.h"
#include "imu.h"
#include "rbt_class.h"
#include <cstdint>
#include <cstring>

/**--- 无敌·哨兵！！！ ---**/

CLASS_ROBOT robot;

uint32_t flag = 0;

/**
 * @brief CAN1回调函数
 *
 * @param FDCAN_RxMessage CAN1收到的消息
 */
void Device_FDCAN1_Callback(Struct_FDCAN_Rx_Buffer *FDCAN_RxMessage) {
	
	switch (FDCAN_RxMessage->Header.Identifier)
	{
	case(0x12):
		{
			robot.Foot1_Left.M_back.CAN_RxCpltCallback(FDCAN_RxMessage->Data);
			break;
		}
	case(0x11):
		{
			robot.Foot1_Left.M_front.CAN_RxCpltCallback(FDCAN_RxMessage->Data);
			break;
		}
	case(0x14):
		{
			robot.Foot2_Right.M_back.CAN_RxCpltCallback(FDCAN_RxMessage->Data);
			break;
		}
	case(0x13):
		{
			robot.Foot2_Right.M_front.CAN_RxCpltCallback(FDCAN_RxMessage->Data);
			break;
		}
	default:
		// 所有case都不匹配时执行
		break;
	}
//   switch (FDCAN_RxMessage->Header.Identifier & 0x7FF) {
// 		case (0x202):
// 		{
// 			robot.Booster.Motor_Friction_Right.CAN_RxCpltCallback(FDCAN_RxMessage->Data);
// 			break;
// 		}
// 		case (0x201):
// 		{
// 			robot.Booster.Motor_Friction_Left.CAN_RxCpltCallback(FDCAN_RxMessage->Data);
// 			break;
// 		}
// 		case (0x203):
// 		{
// 			robot.Booster.Motor_Driver.CAN_RxCpltCallback(FDCAN_RxMessage->Data);
// 			break;
// 		}
//         case (0x204):
// 		{
// 			robot.Lift.Motor_Lift.CAN_RxCpltCallback(FDCAN_RxMessage->Data);
// 			break;
// 		}
// //		case (0x204):
// //		{
// //			static uint8_t cnt= 0;
// //				cnt++;
// //			break;
// //		}
//   }
}

/**
 * @brief CAN2回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
void Device_FDCAN2_Callback(Struct_FDCAN_Rx_Buffer *FDCAN_RxMessage) {
  switch (FDCAN_RxMessage->Header.Identifier & 0x7FF) {
		case (0x201):
		{
			robot.Chassis.Motor_Chassis_Right.CAN_RxCpltCallback(FDCAN_RxMessage->Data);
      break;
		}
		case (0x202):
		{
				robot.Chassis.Motor_Chassis_Left.CAN_RxCpltCallback(FDCAN_RxMessage->Data);
      break;
		}
  }
}

/**
 * @brief CAN3回调函数
 *
 * @param FDCAN_RxMessage CAN3收到的消息
 */
void Device_FDCAN3_Callback(Struct_FDCAN_Rx_Buffer *FDCAN_RxMessage) {
  // switch (FDCAN_RxMessage->Header.Identifier & 0x7FF) {
  //   case (0x100):
  //   {
  //       robot.The_Chassis.CAN_RxCpltCallback(FDCAN_RxMessage);
  //       break;
  //   }
  //   case (0x101):
  //   {
  //       robot.The_Chassis.CAN_RxCpltCallback(FDCAN_RxMessage);
  //       break;
  //   }
  // }
}

/**
 * @brief UART1遥控器回调函数
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */
void DR16_UART5_Callback(uint8_t *Buffer, uint16_t Length) {
  //robot.DR16.UART_RxCpltCallback(Buffer, Length);
}

void USART1_RX_Callback(uint8_t *Buffer, uint16_t Length) {
//   // robot.Referee.UART_RxCpltCallback(Buffer, Length);
//   float temp;
//   memcpy(&temp, Buffer + 4, 4);
//   if (Buffer[0] == 0xAA && Buffer[1] == 0xFF && Buffer[2] == 0x00) {
//     switch (Buffer[3]) {
//     case (0x10): {
//       //robot.Gimbal.Motor_Small_Yaw.PID_Omega.Set_K_P(temp);
//       //robot.Gimbal.Motor_Pitch.PID_Omega.Set_K_P(temp);
//       //robot.Booster.Motor_Driver.PID_Omega.Set_K_P(temp);
//       //robot.Booster.Motor_Friction_Left.PID_Omega.Set_K_P(temp);
//         robot.Lift.Motor_Lift.PID_Angle.Set_K_P(temp);
//         break;
//     }
//     case (0x11): {
//       //robot.Gimbal.Motor_Small_Yaw.PID_Omega.Set_K_I(temp);
//       //robot.Gimbal.Motor_Pitch.PID_Omega.Set_K_I(temp);
//       //robot.Booster.Motor_Driver.PID_Omega.Set_K_I(temp);
// 	  //robot.Booster.Motor_Friction_Left.PID_Omega.Set_K_I(temp);
//         robot.Lift.Motor_Lift.PID_Angle.Set_K_I(temp);
//         break;
//     }
//     case (0x12): {
//       //robot.Gimbal.Motor_Small_Yaw.PID_Omega.Set_K_D(temp);
//       //robot.Gimbal.Motor_Pitch.PID_Omega.Set_K_D(temp);
//       //robot.Booster.Motor_Driver.PID_Omega.Set_K_D(temp);
//       //robot.Booster.Motor_Friction_Left.PID_Omega.Set_K_D(temp);
//         robot.Lift.Motor_Lift.PID_Angle.Set_K_D(temp);
//         break;
//     }
// 	case (0x13):{
// 			//robot.Gimbal.Motor_Small_Yaw.PID_Omega.Set_K_FF(temp);
//       robot.Booster.Motor_Friction_Left.PID_Omega.Set_K_FF(temp);
//       //robot.Booster.Motor_Driver.PID_Omega.Set_K_FF(temp);
// 			break;
// 		}
//     case (0x14): {
//       //robot.Gimbal.Motor_Small_Yaw.PID_Omega.Set_K_F(temp);
//       robot.Booster.Motor_Friction_Left.Set_Target_Omega(temp);
//      // robot.Booster.Motor_Driver.PID_Omega.Set_K_F(temp);
//       break;
//     }
// 		case (0x15): {
// 			robot.Gimbal.Motor_Pitch.Set_K_D(temp);
// 		//	robot.Gimbal.Motor_Small_Yaw.Set_Target_Omega(temp);
// 			break;
//     }
// 		case (0x16):
// 		{
// 			if(temp != 0.0f)
// 			{
// 			//robot.Gimbal.Motor_Big_Yaw.CAN_Send_Save_Zero();
// 			}
// 			break;
// 		}
//   }
// }
}

void VOFA_UART6_Callback(uint8_t *Buffer, uint16_t Length) {
  // robot.Vofa_SerialPlot.UART_RxCpltCallback(Buffer,Length);
}

void BMI088_SPI2_Callback(uint16_t Length) {}

void TIM6_Robot_1ms_Callback() {
  robot_1ms_cycle_flag = true;

  // 1ms_检测
  static uint8_t cnt_100ms = 0;
  static uint16_t cnt_500ms = 0;
  static uint16_t cnt_1000ms = 0;

  cnt_100ms++;
  cnt_500ms++;
  cnt_1000ms++;

  if (cnt_100ms == 100) {
    robot_100ms_cycle_flag = true;
    cnt_100ms = 0;
  }
  if (cnt_1000ms == 1000) {
    robot_1000ms_cycle_flag = true;
    cnt_1000ms = 0;
  }
  // 1ms通信任务，保证控制频率
}


void FSIA6B_RX_Callback(uint8_t *Rx_Data, uint16_t Length) {

	robot.FS_I6X.UART_RxCpltCallback(Rx_Data, Length);
}