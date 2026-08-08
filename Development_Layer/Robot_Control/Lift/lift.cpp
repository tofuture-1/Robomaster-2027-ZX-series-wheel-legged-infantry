#include "lift.h"
#include "rbt_class.h"

void Class_FSM_Lift::RTOS_1ms_Calculate_Callback() {
  Status[Now_Status_Serial].Count_Time++;

  float yaw_angle = Robot->Gimbal.Motor_Yaw.Get_Real_Angle();
  bool yaw_at_zero = (fabsf(yaw_angle) < Yaw_Zero_Threshold);

  bool key_f_trigger = (Robot->VT03.Get_Keyboard_Key_F() == VTM_RX_Key_Status_TRIG_FREE_PRESSED);
  bool key_g_trigger = (Robot->VT03.Get_Keyboard_Key_G() == VTM_RX_Key_Status_TRIG_FREE_PRESSED);

  float tof_distance_cm = Robot->TOF050F.Get_Distance_MM() / 10.0f;

  switch (Now_Status_Serial) {
  case (FSM_Lift_Status_UP): {
    if (tof_distance_cm < TOF_Up_Threshold && tof_distance_cm > TOF_Down_Threshold) {
      Set_Status(FSM_Lift_Status_MIDDLE);
    }
    else if (yaw_at_zero && key_g_trigger) {
      Set_Status(FSM_Lift_Status_FALL);
    }
    break;
  }
  case (FSM_Lift_Status_DOWN): {
    if (tof_distance_cm < TOF_Up_Threshold && tof_distance_cm > TOF_Down_Threshold) {
      Set_Status(FSM_Lift_Status_MIDDLE);
    }
    else if (yaw_at_zero && key_f_trigger) {
      Set_Status(FSM_Lift_Status_RISE);
    }
    break;
  }
  case (FSM_Lift_Status_MIDDLE): {
    if (yaw_at_zero && key_f_trigger) {
      Set_Status(FSM_Lift_Status_RISE);
    }
    else if (yaw_at_zero && key_g_trigger) {
      Set_Status(FSM_Lift_Status_FALL);
    }
    break;
  }
  case (FSM_Lift_Status_RISE): {
    if (tof_distance_cm >= TOF_Up_Threshold) {
      Set_Status(FSM_Lift_Status_UP);
    }
    break;
  }
  case (FSM_Lift_Status_FALL): {
    if (tof_distance_cm <= TOF_Down_Threshold) {
      Set_Status(FSM_Lift_Status_DOWN);
    }
    break;
  }
  }
}

void Class_Lift::Init(CLASS_ROBOT *robot) {
  Robot = robot;
  FSM_Lift.Robot = robot;
  FSM_Lift.Init(5, FSM_Lift_Status_MIDDLE);

  Motor_Lift.Init(&hfdcan1, Motor_DJI_ID_0x204, Motor_DJI_Control_Method_OMEGA,
                  1.0f, Motor_DJI_Power_Limit_Status_DISABLE, 16384.0f);

  Motor_Lift.Filter_Kalman_Omega.Init(0.01f, 2.0f, 0.1f, 0.0f);
  Motor_Lift.PID_Omega.Init(0.2f, 0.3f, 0.0f, 0.0f, 0.0f, 8.0f, 20.0f);
  Motor_Lift.PID_Angle.Init(3.0f, 0.0f, 0.001f, 0.0f, 0.0f, 15.0f * PI, 3.0f * PI);
}

void Class_Lift::RTOS_1ms_Calculate_Callback() {
  FSM_Lift.RTOS_1ms_Calculate_Callback();
  Motor_Lift.RTOS_Calculate_Callback();
}

void Class_Lift::RTOS_100ms_Alive_Callback() {
  Motor_Lift.RTOS_100ms_Alive_Callback();
}

float Class_Lift::Calculate_Kinematics(float tof_distance_cm) {
  float effective_h = tof_distance_cm + 7.0f;
  Math_Constrain(&effective_h, 0.0f, Linkage_Length);

  float base = sqrtf(Linkage_Length * Linkage_Length - effective_h * effective_h);

  float x = Total_Length - base;

  float turns = x * CM_To_Turns;

  return turns * 2.0f * PI;
}

void Class_Lift::Control() 
{
//  ====================״̬�����Զ�����=======================
//  float tof_distance_cm = Robot->TOF050F.Get_Distance_MM() / 10.0f;

//  if (Motor_Lift.Get_Status() == Motor_DJI_Status_DISABLE) {
//    Start_Angle = Motor_Lift.Get_Now_Angle();
//    return;
//  }

//  switch (FSM_Lift.Get_Now_Status_Serial()) {
//  case FSM_Lift_Status_UP: {
//    Start_Angle = Motor_Lift.Get_Now_Angle();
//    Motor_Lift.Set_Target_Angle(Start_Angle);
//    break;
//  }
//  case FSM_Lift_Status_DOWN: {
//    Start_Angle = Motor_Lift.Get_Now_Angle();
//    Motor_Lift.Set_Target_Angle(Start_Angle);
//    break;
//  }
//  case FSM_Lift_Status_MIDDLE: {
//    Start_Angle = Motor_Lift.Get_Now_Angle();
//    Motor_Lift.Set_Target_Angle(Start_Angle);
//    break;
//  }
//  case FSM_Lift_Status_RISE: {
//    Target_Angle = Start_Angle + Calculate_Kinematics(tof_distance_cm);
//    Math_Constrain(&Target_Angle, Angle_Min, Angle_Max);
//    Motor_Lift.Set_Target_Angle(Target_Angle);
//    break;
//  }
//  case FSM_Lift_Status_FALL: {
//    Target_Angle = Start_Angle - Calculate_Kinematics(tof_distance_cm);
//    Math_Constrain(&Target_Angle, Angle_Min, Angle_Max);
//    Motor_Lift.Set_Target_Angle(Target_Angle);
//    break;
//  }
//  }

  Robot->Gimbal.Yaw_Encoder_Lock =
      (FSM_Lift.Get_Now_Status_Serial() == FSM_Lift_Status_DOWN);

  if (Motor_Lift.Get_Status() == Motor_DJI_Status_DISABLE) {
    Start_Angle = Motor_Lift.Get_Now_Angle();
    return;
  }

  if (Robot->VT03.Get_Status() == VTM_RX_Status_ENABLE) {
    if (Robot->current_control_mode == CONTROL_MODE_REMOTE) {
      float wheel = Robot->VT03.Get_Wheel();

      if (wheel > 0.1f || wheel < -0.1f) {
        float wheel_speed = wheel * Wheel_Max_Speed;
        Math_Constrain(&wheel_speed, -Wheel_Max_Speed, Wheel_Max_Speed);
        Motor_Lift.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor_Lift.Set_Target_Omega(wheel_speed);
      } else {
        Motor_Lift.Set_Control_Method(Motor_DJI_Control_Method_ANGLE);
        Motor_Lift.Set_Target_Angle(Motor_Lift.Get_Now_Angle());
      }
    }
    else if (Robot->current_control_mode == CONTROL_MODE_KEYBOARD) {
      // ==================== ԭ̧����������ģʽ�����߼� =======================
      // bool key_f = (Robot->VT03.Get_Keyboard_Key_F() == VTM_RX_Key_Status_PRESSED ||
      //               Robot->VT03.Get_Keyboard_Key_F() == VTM_RX_Key_Status_TRIG_FREE_PRESSED);
      // bool key_g = (Robot->VT03.Get_Keyboard_Key_G() == VTM_RX_Key_Status_PRESSED ||
      //               Robot->VT03.Get_Keyboard_Key_G() == VTM_RX_Key_Status_TRIG_FREE_PRESSED);
      //
      // if (key_f && !key_g) {
      //   Motor_Lift.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
      //   Motor_Lift.Set_Target_Omega(Rise_Speed);
      // }
      // else if (key_g && !key_f) {
      //   Motor_Lift.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
      //   Motor_Lift.Set_Target_Omega(Fall_Speed);
      // }
      // else {
      //   Motor_Lift.Set_Control_Method(Motor_DJI_Control_Method_ANGLE);
      //   Motor_Lift.Set_Target_Angle(Motor_Lift.Get_Now_Angle());
      // }

      // ==================== �¼���ģʽ�����߼� =======================
      bool key_f_pressed = (Robot->VT03.Get_Keyboard_Key_F() == VTM_RX_Key_Status_PRESSED ||
                            Robot->VT03.Get_Keyboard_Key_F() == VTM_RX_Key_Status_TRIG_FREE_PRESSED);
      bool key_g_trigger = (Robot->VT03.Get_Keyboard_Key_G() == VTM_RX_Key_Status_TRIG_FREE_PRESSED);

      float tof_distance_cm = Robot->TOF050F.Get_Distance_MM() / 10.0f;

      // G�������Զ�����
      if (key_g_trigger) {
        Auto_Rise_Active = true;
      }

      if (Auto_Rise_Active && tof_distance_cm >= 15.0f && tof_distance_cm <= 80.0f) {
        Auto_Rise_Active = false;
      }

      // F���½�����������ͬʱȡ���Զ�����
      if (key_f_pressed) {
        Auto_Rise_Active = false;
        Motor_Lift.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor_Lift.Set_Target_Omega(Fall_Speed);
      }
      else if (Auto_Rise_Active) {
        Motor_Lift.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor_Lift.Set_Target_Omega(Rise_Speed);
      }
      else {
        Motor_Lift.Set_Control_Method(Motor_DJI_Control_Method_ANGLE);
        Motor_Lift.Set_Target_Angle(Motor_Lift.Get_Now_Angle());
      }
    }
    else {
      Motor_Lift.Set_Control_Method(Motor_DJI_Control_Method_ANGLE);
      Motor_Lift.Set_Target_Angle(Motor_Lift.Get_Now_Angle());
    }
  } else {
    Motor_Lift.Set_Control_Method(Motor_DJI_Control_Method_ANGLE);
    Motor_Lift.Set_Target_Angle(Motor_Lift.Get_Now_Angle());
  }
}

void Class_Lift::Set_Motor_Enable(bool enable) {
  if (enable) {
    Motor_Lift.Set_Control_Method(Motor_DJI_Control_Method_ANGLE);
  }
  else {
    Motor_Lift.Set_Target_Omega(0.0f);
    Start_Angle = Motor_Lift.Get_Now_Angle();
  }
}
