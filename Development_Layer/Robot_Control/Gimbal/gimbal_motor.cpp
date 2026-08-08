#include "gimbal_motor.h"
#include "imu.h"
#include "motor_dji.h"

/**
 * @brief 初始化云台电机（复写基类Init以做子类额外初始化）
 */
void Class_4310_Joint_With_IMU::Init(
    FDCAN_HandleTypeDef *hfdcan, uint16_t __CAN_Rx_ID, uint16_t __CAN_Tx_ID,
    Gimbal_Axis_Type __axis_type,
    Enum_Motor_DM_Control_Method __Motor_DM_Control_Method, float __Angle_Max,
    float __Omega_Max, float __Torque_Max, float __Current_Max) {
  // 调用基类初始化
  Class_Motor_DM_Normal::Init(hfdcan, __CAN_Rx_ID, __CAN_Tx_ID,
                              __Motor_DM_Control_Method, __Angle_Max,
                              __Omega_Max, __Torque_Max, __Current_Max);

  // 设定滤波器：默认低通，采样频率与系统1kHz一致
  Filter_Omega_DM_4310.Init(0.01f, 2.0f, 0.1f, 0.0f);

  // 设置绑定轴类型
  axis_type = __axis_type;

  // 清零内部滤波量
  after_filter_Omega = 0.0f;

  if(__Motor_DM_Control_Method == Motor_DM_Control_Method_NORMAL_MIT)
  {
    Set_K_P(0.0f);
  }
}

void Class_4310_Joint_With_IMU::Bind_IMU(Class_Board_IMU *__IMU) {
  Motor_Absolute_Angle = __IMU;
}

void Class_4310_Joint_With_IMU::RTOS_1ms_Calculate_Callback() {
  // 进行滤波计算（4310电机使用IMU的角速度而不是电机反馈速度）
  float now_omega ;

  now_omega = Rx_Data.Now_Omega;

  if(Motor_DM_Control_Method == Motor_DM_Control_Method_NORMAL_MIT)
  {
  Filter_Omega_DM_4310.Set_Measurement(now_omega);
  Filter_Omega_DM_4310.Filter_Calculate();
  after_filter_Omega = Filter_Omega_DM_4310.Get_State_Estimate();
  }


  PID_Calculate();
  // 电机在线, 正常控制
  if (Rx_Data.Control_Status == Motor_DM_Status_ENABLE) {
    Math_Constrain(&Control_Omega, -Omega_Max, Omega_Max);
    Math_Constrain(&Control_Torque, -Torque_Max, Torque_Max);
    Math_Constrain(&Control_Current, -Current_Max, Current_Max);
    Math_Constrain(&K_D, 0.0f, 5.0f);

    Output();
   } 
	  else if (Rx_Data.Control_Status == Motor_DM_Status_DISABLE)
    {
        // 电机可能掉线, 使能电机
        CAN_Send_Enter();
    }
    else
    {
        // 电机错误, 发送清除错误帧
        CAN_Send_Clear_Error();
    }
}

/**
 * @brief pid计算
 *
 */
void Class_4310_Joint_With_IMU::PID_Calculate() {

  switch (Motor_DM_Control_Method) {
  case (Motor_DM_Control_Method_NORMAL_MIT): {
      PID_Angle.Set_Target(Target_Angle);
      PID_Angle.Set_Now(Get_Now_Angle());
      PID_Angle.PID_Process();

      Control_Omega = -(PID_Angle.Get_Out() + Control_FeedForward_Omega);

      PID_Omega.Set_Target(Control_Omega);
      PID_Omega.Set_Now(Get_Now_Omega());
      PID_Omega.PID_Process();

      // 重力补偿：根据 pitch 角度和运动方向
      float gravity_compensation = 0.0f;
		  const float gravity_torque = 0.4f;
      if (axis_type == The_Pitch) {
        float pitch_angle = Get_Now_Angle();
        // 当抬头时需要更大的速度指令来克服重力
          gravity_compensation = gravity_torque * arm_cos_f32(fabsf(pitch_angle));
      }

      if(The_Yaw == axis_type)
      {
          Control_Torque = PID_Omega.Get_Out();
      }
      else
      {
          Control_Torque = PID_Omega.Get_Out() + gravity_compensation;
      }

    break;
  }
  case (Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA): {
   //直接发送无需计算
    break;
  }
  case (Motor_DM_Control_Method_NORMAL_OMEGA): {
    // 速度环 -> 扭矩(输出)
      PID_Omega.Set_Target(Target_Omega);
      PID_Omega.Set_Now(Get_Now_Omega());
      PID_Omega.PID_Process();

      Control_Torque = PID_Omega.Get_Out();
      Control_Omega = Target_Omega;
    break;
  }
  case (Motor_DM_Control_Method_NORMAL_EMIT): {
    // EMIT 模式通常是限幅/发送限幅值：不通过 PID 计算扭矩，保留
    // Control_Current/Control_Omega
    Control_Torque = 0.0f;
    break;
  }
  }
}

/**
 * @brief 100ms心跳回调，复写以在电机离线时清零PID积分
 */
void Class_4310_Joint_With_IMU::RTOS_100ms_Alive_Callback() {
  // 先执行基类的在线检测逻辑
  Class_Motor_DM_Normal::RTOS_100ms_Alive_Callback();

  // Motor_Absolute_Angle->RTOS_100ms_IMU_Alive_Callback();
}

void Class_4310_Joint_With_IMU::Output() {
  // 电机控制
  switch (Motor_DM_Control_Method) {
  case (Motor_DM_Control_Method_NORMAL_MIT): {
    Struct_Motor_DM_CAN_Tx_Data_Normal_MIT *tmp_buffer =
        (Struct_Motor_DM_CAN_Tx_Data_Normal_MIT *)Tx_Data;

    uint16_t tmp_angle, tmp_omega, tmp_torque, tmp_k_p, tmp_k_d;
    // 修改了目标角度
    tmp_angle = 0x0000;
    tmp_omega =
        Math_Float_To_Int(Control_Omega, 0, Omega_Max, 0x7ff, (1 << 12) - 1);
    tmp_torque =
        Math_Float_To_Int(Control_Torque, 0, Torque_Max, 0x7ff, (1 << 12) - 1);
    tmp_k_p = Math_Float_To_Int(0.0f, 0, 500.0f, 0, (1 << 12) - 1);
    tmp_k_d = Math_Float_To_Int(K_D, 0, 5.0f, 0, (1 << 12) - 1);

    tmp_buffer->Control_Angle_Reverse =
        Math_Endian_Reverse_16(&tmp_angle, nullptr);
    tmp_buffer->Control_Omega_11_4 = tmp_omega >> 4;
    tmp_buffer->Control_Omega_3_0_K_P_11_8 =
        ((tmp_omega & 0x0f) << 4) | (tmp_k_p >> 8);
    tmp_buffer->K_P_7_0 = tmp_k_p & 0xff;
    tmp_buffer->K_D_11_4 = tmp_k_d >> 4;
    tmp_buffer->K_D_3_0_Control_Torque_11_8 =
        ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
    tmp_buffer->Control_Torque_7_0 = tmp_torque & 0xff;

    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, CAN_Tx_ID, Tx_Data, 8);


    break;
  }
  case (Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA): {
    Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega *tmp_buffer =
        (Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega *)Tx_Data;

    tmp_buffer->Control_Angle = Control_Angle;
    tmp_buffer->Control_Omega = Control_Omega;

    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, CAN_Tx_ID, Tx_Data, 8);

    break;
  }
  case (Motor_DM_Control_Method_NORMAL_OMEGA): {
    Struct_Motor_DM_CAN_Tx_Data_Normal_Omega *tmp_buffer =
        (Struct_Motor_DM_CAN_Tx_Data_Normal_Omega *)Tx_Data;

    tmp_buffer->Control_Omega = Control_Omega;

    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, CAN_Tx_ID, Tx_Data, 4);

    break;
  }
  case (Motor_DM_Control_Method_NORMAL_EMIT): {
    Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT *tmp_buffer =
        (Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT *)Tx_Data;

    tmp_buffer->Control_Angle = Control_Angle;
    tmp_buffer->Control_Omega = (uint16_t)(Control_Omega * 100.0f);
    tmp_buffer->Control_Current =
        (uint16_t)(Control_Current / Current_Max * 10000.0f);

    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, CAN_Tx_ID, Tx_Data, 8);

    break;
  }
  }
}



