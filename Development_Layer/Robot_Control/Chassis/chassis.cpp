//
// Created by 24158 on 2026/8/8.
//

#include "chassis.h"


/**
 * @brief TIM定时器中断计算回调函数, 计算周期取决于电机反馈周期
 *
 */
void Class_Chassis_Motor_DJI_C620::RTOS_Calculate_Callback()
{
    Filter_Kalman_Omega.Set_Measurement(Rx_Data.Now_Omega);
    Filter_Kalman_Omega.Filter_Calculate();
    after_filter_Omega = Filter_Kalman_Omega.Get_Out();

    PID_Calculate();

    float tmp_value = Target_Current + Feedforward_Current;
    Math_Constrain(&tmp_value, -Current_Max, Current_Max);
    Out = tmp_value * Current_To_Out;

    Output();
}





/**
 * @brief 计算PID
 *
 */
void Class_Chassis_Motor_DJI_C620::PID_Calculate()
{
    switch (Motor_DJI_Control_Method)
    {
    case (Motor_DJI_Control_Method_CURRENT):
        {
            break;
        }
    case (Motor_DJI_Control_Method_OMEGA):
        {
            PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
            PID_Omega.Set_Now(after_filter_Omega);
            PID_Omega.PID_Process();

            Target_Current = PID_Omega.Get_Out();

            break;
        }
    case (Motor_DJI_Control_Method_ANGLE):
        {
            PID_Angle.Set_Target(Target_Angle);
            PID_Angle.Set_Now(Rx_Data.Now_Angle);
            PID_Angle.PID_Process();

            Target_Omega = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
            PID_Omega.Set_Now(after_filter_Omega);
            PID_Omega.PID_Process();

            Target_Current = PID_Omega.Get_Out();

            break;
        }
    default:
        {
            Target_Current = 0.0f;

            break;
        }
    }
    Feedforward_Current = 0.0f;
    Feedforward_Omega = 0.0f;
}


void Class_Chassis::Init()
{



    // 滤波器初始化
    Motor_Chassis_Left.Filter_Kalman_Omega.Init(0.01f, 2.0f, 0.1f, 0.0f);
    // PID初始化
    Motor_Chassis_Left.PID_Omega.Init(0.2f, 0.25f, 0.0f, 0.0f, 0.0f , 15.0f, 20.0f);
    // 电机初始化

    Motor_Chassis_Left.Init(&hfdcan2,Motor_DJI_ID_0x202, Motor_DJI_Control_Method_OMEGA);

    // 滤波器初始化
    Motor_Chassis_Right.Filter_Kalman_Omega.Init(0.01f, 2.0f, 0.1f, 0.0f);
    // PID初始化
    Motor_Chassis_Right.PID_Omega.Init(0.2f, 0.25f, 0.0f, 0.0f, 0.0f , 15.0f, 20.0f);

    Motor_Chassis_Right.Init(&hfdcan2,Motor_DJI_ID_0x201, Motor_DJI_Control_Method_OMEGA);


}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Chassis::RTOS_100ms_Alive_Callback()
{

    Motor_Chassis_Left.RTOS_100ms_Alive_Callback();
    Motor_Chassis_Right.RTOS_100ms_Alive_Callback();
}
/**
 * @brief 定时器计算函数
 *
 */
void Class_Chassis::RTOS_1ms_Calculate_Callback()
{

    Output();



    Motor_Chassis_Left.RTOS_Calculate_Callback();
    Motor_Chassis_Right.RTOS_Calculate_Callback();
}
void Class_Chassis::Output()
{
    Motor_Chassis_Left.Set_Target_Omega(Chassis_Omega);
    Motor_Chassis_Right.Set_Target_Omega(Chassis_Omega);
}
