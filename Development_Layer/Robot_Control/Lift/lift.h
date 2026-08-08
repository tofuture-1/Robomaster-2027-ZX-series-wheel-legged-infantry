#ifndef C_BOARD_LIFT_H
#define C_BOARD_LIFT_H

#include "fsm_class.h"
#include "motor_dji.h"
#include "VTM_RX.h"
#include "tof050.h"
#include "booster.h"

class CLASS_ROBOT;

enum Enum_FSM_Lift_Status
{
    FSM_Lift_Status_UP = 0,
    FSM_Lift_Status_DOWN,
    FSM_Lift_Status_RISE,
    FSM_Lift_Status_FALL,
    FSM_Lift_Status_MIDDLE,
};

class Class_FSM_Lift : public Class_FSM
{
public:
    CLASS_ROBOT *Robot;

    float Yaw_Zero_Threshold = 0.1f;   // 0.05rad��Լ����2.86��
    float TOF_Up_Threshold = 17.0f;     // TOF �� 17cm ��Ϊ������λ
    float TOF_Down_Threshold = 11.5f;   // TOF �� 4cm ��Ϊ�½���λ

    void RTOS_1ms_Calculate_Callback();
};

class Class_Lift
{
public:
    Class_FSM_Lift FSM_Lift;
    Class_Booster_Friction_Motor_DJI_C620 Motor_Lift;

    float Target_Angle = 0.0f;
    float Start_Angle = 0.0f;
    float Angle_Min = -15.0f * PI;
    float Angle_Max =  15.0f * PI;
    float Wheel_Max_Speed = 10.0f * PI;
    float Rise_Speed = -18.0f * PI;      // �������ٶ�, rad/s
    float Fall_Speed = 14.0f * PI;     // �½����ٶ�, rad/s

    bool Auto_Rise_Active = false;      // �Զ�������־λ

    float Linkage_Length = 24.0f; // ���˳���, cm
    float Total_Length = 23.5f;   // �ױ��ܳ���, cm
    float CM_To_Turns = 2.5f;     // cm�����Ȧ����ת��ϵ��

    float Calculate_Kinematics(float tof_distance_cm);

    void Init(CLASS_ROBOT *robot);
    void RTOS_1ms_Calculate_Callback();
    void RTOS_100ms_Alive_Callback();
    void Control();

    void Set_Motor_Enable(bool enable);

private:
    CLASS_ROBOT *Robot;
};

#endif
