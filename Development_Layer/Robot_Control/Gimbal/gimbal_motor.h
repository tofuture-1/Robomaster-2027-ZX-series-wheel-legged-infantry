#ifndef __GIMBAL_MOTOR_H_
#define __GIMBAL_MOTOR_H_

#include "drv_math.h"
#include "filter.h"
#include "imu.h"
#include "motor_dm.h"
#include "stm32h7xx_hal_rcc_ex.h"

/**
 * @brief 云台电机轴类型
 *
 */
typedef enum { The_Pitch = 0, The_Yaw } Gimbal_Axis_Type;


/**
 * @brief 这是一坨针对云台关节电机的达妙电机类复写
 *
 */
class Class_4310_Joint_With_IMU : public Class_Motor_DM_Normal {
public:
  void Init(FDCAN_HandleTypeDef *hcan, uint16_t __CAN_Rx_ID, uint16_t __CAN_Tx_ID,
            Gimbal_Axis_Type __axis_type,
            Enum_Motor_DM_Control_Method __Motor_DM_Control_Method =
                Motor_DM_Control_Method_NORMAL_MIT,
            float __Angle_Max = 12.5f, float __Omega_Max = 30.0f,
            float __Torque_Max = 10.0f, float __Current_Max = 10.261194f);

  void Bind_IMU(Class_Board_IMU *__IMU);

  Gimbal_Axis_Type axis_type;

  bool Follow_Chassis_Mode = false;

  Class_PID PID_Angle;

  Class_PID PID_Omega;

  Class_Board_IMU *Motor_Absolute_Angle;

  Class_Filter_Kalman Filter_Omega_DM_4310;

  virtual inline float Get_Now_Angle();

  inline float Get_Real_Angle();

  inline float Get_Target_Angle();

  virtual inline float Get_Now_Omega();

  inline float Get_Real_Omega();

  inline Class_Board_IMU* Get_Bind_IMU() { return Motor_Absolute_Angle; }

  inline void Set_Target_Angle(float __Target_Angle);

  inline void Set_Target_Omega(float __Target_Omega);

  inline void Set_Control_FeedForward_Omega(float feedforward);
  inline void Set_Control_FeedForward_Current(float feedforward);
  inline void Set_Control_FeedForward_Angle(float feedforward);

  virtual void RTOS_100ms_Alive_Callback();

  void RTOS_1ms_Calculate_Callback();
    // 电机反馈的角速度
  float after_filter_Omega = 0.0f;

private:

  float Target_Angle = 0.0f;

  float Target_Omega = 0.0f;

  float Control_FeedForward_Angle = 0.0f;

  float Control_FeedForward_Omega = 0.0f;

  float Control_FeedForward_Current = 0.0f;

  void PID_Calculate();

  virtual void Output();
};



/**
 * @brief 外置目标角度值
 *
 * @param __Target_Angle
 */
inline void Class_4310_Joint_With_IMU::Set_Target_Angle(float __Target_Angle) {
  Target_Angle = __Target_Angle;
}

/**
 * @brief 设置目标角速度（速度模式）
 *
 * @param __Target_Omega 目标角速度 rad/s
 */
inline void Class_4310_Joint_With_IMU::Set_Target_Omega(float __Target_Omega) {
  Target_Omega = __Target_Omega;
}



inline void Class_4310_Joint_With_IMU::Set_Control_FeedForward_Angle(float feedforward) {
  Control_FeedForward_Angle = feedforward;
}
/**
 * @brief 设置前馈速度，用于小陀螺云台自稳
 *
 * @param feedforward
 */
inline void
Class_4310_Joint_With_IMU::Set_Control_FeedForward_Omega(float feedforward) {
  Control_FeedForward_Omega = feedforward;
}

/**
 * @brief 设置前馈电流，用于小陀螺云台自稳
 *
 * @param feedforward
 */
inline void
Class_4310_Joint_With_IMU::Set_Control_FeedForward_Current(float feedforward) {
  Control_FeedForward_Current = feedforward;
}



inline float Class_4310_Joint_With_IMU::Get_Target_Angle() {
  return (Target_Angle);
}

/**
 * @brief 复写
 *
 * @return float
 */
inline float Class_4310_Joint_With_IMU::Get_Now_Angle() {
  if (axis_type == The_Yaw) {
    if (Follow_Chassis_Mode) {
      return Rx_Data.Now_Angle;
    }
    return (Motor_Absolute_Angle->Get_YawTotalAngle() * DEG_TO_RAD);
  } 
	else {
		if (Follow_Chassis_Mode) {
      return Rx_Data.Now_Angle;
    }
    return (Motor_Absolute_Angle->Get_Roll() * DEG_TO_RAD);
  }
}

/**
 *
 */
inline float Class_4310_Joint_With_IMU::Get_Real_Angle() {
  return (Rx_Data.Now_Angle);
}

/**
 * @brief 滤波后的角速度
 *
 * @return float
 */
inline float Class_4310_Joint_With_IMU::Get_Now_Omega() {
  return after_filter_Omega;
}

inline float Class_4310_Joint_With_IMU::Get_Real_Omega() {
   return (Rx_Data.Now_Omega);
}



#endif /* __GIMBAL_MOTOR_H_ */
