#include "gimbal.h"

#include "User_debug_config.h"
#include "motor_dji.h"
#include <cmath>


/**
 * @brief 初始化云台对象和各执行、感知模块。
 * @param __PC_Data 上位机接收数据指针。
 */
void Class_Gimbal::Init(PC_ReceivePacket *__PC_Data) {
  PC_Data = __PC_Data;

  Motor_Yaw.Init(&hfdcan2, 0x06, 0x03, The_Yaw,
                     Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 30.0f, 10.0f,
                     10.261194f);
    
  Motor_Yaw.PID_Angle.Init(12.5f, 0.0f, 0.005f, 0.0f, 0.2f, 0.0f, 4*PI,
                               0.001f);
  Motor_Yaw.PID_Omega.Init(1.6f, 8.0f, 0.0f, 0.0f, 0.2f, 2.0f, 10.0f,
                               0.001f);
    

  Motor_Yaw.Set_K_D(2.0f);

  //Motor_Yaw.CAN_Send_Save_Zero();
	//Motor_Pitch.CAN_Send_Save_Zero();


  Motor_Pitch.Init(&hfdcan2, 0x07, 0x04, The_Pitch,
                   Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 30.0f,
                   10.0f, 10.261194f);
  Motor_Pitch.PID_Angle.Init(12.0f, 0.0f, 0.002f, 0.0f, 0.0f, 0.0f, 5.0f,
                             0.001f);
  Motor_Pitch.PID_Omega.Init(1.5f, 5.0f, 0.0f, 0.0f, 0.0f, 1.5f, 10.0f,
                             0.001f);
  Motor_Pitch.Set_K_D(2.0f); 

  Gimbal_Work_Status = Gimbal_Mode_ENABLE;

}



/**
 * @brief 将大 yaw 目标映射到最近等效角，减少跨圈目标突变。
 * @param 无。
 */
void Class_Gimbal::_Motor_Nearest_Transposition(float *target_angle_raw ,float now_angle) {
  float tmp_delta_angle =
      fmodf(*target_angle_raw - now_angle, 2.0f * PI);
  if (tmp_delta_angle > PI) {
    tmp_delta_angle -= 2.0f * PI;
  } else if (tmp_delta_angle < -PI) {
    tmp_delta_angle += 2.0f * PI;
  }

  *target_angle_raw = now_angle + tmp_delta_angle;
}

/**
 * @brief 姿态解算，绝对小 yaw 等于大 yaw 与相对小 yaw 之和。
 * @param 无。
 */
void Class_Gimbal::Self_Resolution() {
  Now_Yaw_Angle = Motor_Yaw.Get_Now_Angle();
  Now_Pitch_Angle = Motor_Pitch.Get_Now_Angle();
}


/**
 * @brief 将最终目标统一下发给电机控制器，并在失能时清积分。
 * @param 无。
 */
void Class_Gimbal::Output_To_Motor() {
	
  Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
  Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
	Motor_Yaw.RTOS_1ms_Calculate_Callback();
  Motor_Pitch.RTOS_1ms_Calculate_Callback();  
}


/**
 * @brief 100ms 保活回调，检测在线状态并处理失能恢复后的目标同步。
 * @param 无。
 */
void Class_Gimbal::RTOS_100ms_Alive_Callback() {
  //每个电机的存活检测
  Motor_Yaw.RTOS_100ms_Alive_Callback();
  Motor_Pitch.RTOS_100ms_Alive_Callback();

  if (Motor_Pitch.Get_Status() == Motor_DM_Status_DISABLE ||
      Motor_Yaw.Get_Status() == Motor_DM_Status_DISABLE) {
        Gimbal_Work_Status = Gimbal_Mode_DISABLE;
        return;
  }

  this->Gimbal_Work_Status = Gimbal_Mode_ENABLE;
}

/**
 * @brief 2ms 计算回调，完成与底盘数据的对齐。
 * @param 无。
 */
void Class_Gimbal::RTOS_2ms_Calculate_Callback() {

}

/**
 * @brief 云台 1ms 主控制流程，完成采样滤波、姿态解算、FSM 控制与电机输出。
 * @param 无。
 */
void Class_Gimbal::Encoder_Lock_Control() {
  Motor_Yaw.Follow_Chassis_Mode = true;

	Motor_Yaw.Set_Target_Angle(0.0f);
  Motor_Pitch.Set_Target_Angle(0.0f);
	Motor_Yaw.RTOS_1ms_Calculate_Callback();
  Motor_Pitch.RTOS_1ms_Calculate_Callback();  
}

void Class_Gimbal::RTOS_1ms_Calculate_Callback() {

  if (Yaw_Encoder_Lock) {
    Encoder_Lock_Control();
    Yaw_Encoder_Lock_Prev = true;
    return;
  }

	//这里更新IMU的yaw轴
  Motor_Yaw.Follow_Chassis_Mode = false;
  if (Yaw_Encoder_Lock_Prev) {
    Target_Yaw_Angle = Motor_Yaw.Get_Now_Angle();
		Target_Pitch_Angle = 0.0f;
    Yaw_Encoder_Lock_Prev = false;
  }

  Self_Resolution();
  Output_To_Motor();
  static uint8_t cnt = 0;
  cnt++;
  if(cnt >= 2)
  {
    cnt = 0;
    RTOS_2ms_Calculate_Callback();
  }

}

