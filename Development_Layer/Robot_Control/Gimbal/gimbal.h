#ifndef C_BOARD_GIMBAL_H
#define C_BOARD_GIMBAL_H

#include "fsm_class.h"
#include "imu.h"
#include "motor_dji.h"
#include "motor_dm.h"
#include "pc_packet.h"
#include "gimbal_motor.h"


typedef enum {
  Gimbal_Mode_DISABLE = 0,
  Gimbal_Mode_ENABLE = 1
} Enum_Gimbal_Work_Status;


class Class_Gimbal {
public: 
  //云台电机实体
  Class_4310_Joint_With_IMU  Motor_Yaw;
  Class_4310_Joint_With_IMU  Motor_Pitch;



  /**
   * @brief 初始化云台对象，完成执行器、IMU、状态机与目标量的基础初始化。
   * @param __PC_Data 上位机接收数据指针，用于读取视觉检测状态与目标数据。
   */
  void Init(PC_ReceivePacket *__PC_Data);

  inline float Get_Yaw_Angle();
  inline float Get_Pitch_Angle();

  inline void Set_Target_Velocity_X(float X);
  inline void Set_Target_Velocity_Y(float Y);
  inline void Set_Target_Omega(float Omega);

  inline void Set_Target_Yaw_Angle(float __Yaw_Angle);
  inline void Set_Target_Pitch_Angle(float __Pitch_Angle);

  inline Enum_Gimbal_Work_Status Get_Gimbal_Work_Status();
  inline void Set_Gimbal_Work_Status(Enum_Gimbal_Work_Status __Work_Status);

  //需要绑定上位机指针
  PC_ReceivePacket *PC_Data = nullptr;

  bool Yaw_Encoder_Lock = false;

  float Target_Yaw_Angle = 0.0f;
  float Target_Pitch_Angle = 0.0f;

  float Target_Velocity_X = 0.0f;
  float Target_Velocity_Y = 0.0f;
  float Target_Omega = 0.0f;


  void RTOS_100ms_Alive_Callback();

  void RTOS_2ms_Calculate_Callback();

  void RTOS_1ms_Calculate_Callback();

protected:

  float Min_Pitch_Angle = -0.45f;
  float Max_Pitch_Angle = 0.45f;

  float Now_Yaw_Angle = 0.0f;
  float Now_Pitch_Angle = 0.0f;

  Enum_Gimbal_Work_Status Gimbal_Work_Status = Gimbal_Mode_DISABLE;

  bool Yaw_Encoder_Lock_Prev = false;

  //内部函数
  void Self_Resolution();
  void Encoder_Lock_Control();

  void Output_To_Motor();
  //云台电机就近转位
  void _Motor_Nearest_Transposition(float *target_angle_raw, float now_angle);
};

inline float Class_Gimbal::Get_Yaw_Angle() { return Now_Yaw_Angle; }

inline float Class_Gimbal::Get_Pitch_Angle() { return Now_Pitch_Angle; }

inline void Class_Gimbal::Set_Target_Yaw_Angle(float __Big_Yaw_Angle) {
  Target_Yaw_Angle = __Big_Yaw_Angle;
}

inline void Class_Gimbal::Set_Target_Pitch_Angle(float __Pitch_Angle) {
  Target_Pitch_Angle = __Pitch_Angle;
}

inline void Class_Gimbal::Set_Target_Velocity_X(float X)
{
  Target_Velocity_X = X;
}

inline void Class_Gimbal::Set_Target_Velocity_Y(float Y)
{ Target_Velocity_Y = Y; }

inline void Class_Gimbal::Set_Target_Omega(float Omega)
{ Target_Omega = Omega; }

inline Enum_Gimbal_Work_Status Class_Gimbal::Get_Gimbal_Work_Status() {
  return Gimbal_Work_Status;
}

inline void Class_Gimbal::Set_Gimbal_Work_Status(Enum_Gimbal_Work_Status __Work_Status)
{
  Gimbal_Work_Status = __Work_Status;
}



#endif // C_BOARD_GIMBAL_H
