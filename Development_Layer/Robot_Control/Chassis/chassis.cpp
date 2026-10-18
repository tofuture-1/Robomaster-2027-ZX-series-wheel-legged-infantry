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
void Class_Chassis_Motor_DJI_C620::PID_Calculate()//HACK：实际上非PID的输出也走这里了？应当另外实现
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
        case (Motor_DJI_Control_Method_TORQUE)://自实现的定力矩输出
        {
            Target_Current = Hub_Torque * torque_constant_reciprocal;

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


void Class_Chassis::Init()//HACK：本chassis.cpp貌似跟rbt_class中的chassis有点重合
{


    M8009p_1.Init(&hfdcan1,0x11,0x01);
    M8009p_2.Init(&hfdcan1,0x12,0x02);//HACK:是否确定在此封装？
    //TODO：理解上述电机控制逻辑
    //注意在callback中配置其FDCAN的回调
    M8009p_3.Init(&hfdcan1,0x13,0x03);
    M8009p_4.Init(&hfdcan1,0x14,0x04);//HACK：注意左右腿在同一杆旋转时关节电机旋转方向不一致

    // 滤波器初始化
    Motor_Chassis_Left.Filter_Kalman_Omega.Init(0.01f, 2.0f, 0.1f, 0.0f);
    // PID初始化
    Motor_Chassis_Left.PID_Omega.Init(0.2f, 0.25f, 0.0f, 0.0f, 0.0f , 15.0f, 19.0f);//原booster处的Output_Max是20.0f
    // 电机初始化

    //Motor_Chassis_Left.Init(&hfdcan2,Motor_DJI_ID_0x202, Motor_DJI_Control_Method_OMEGA);
    Motor_Chassis_Left.Init(&hfdcan2,Motor_DJI_ID_0x202, Motor_DJI_Control_Method_TORQUE);

    // 滤波器初始化
    Motor_Chassis_Right.Filter_Kalman_Omega.Init(0.01f, 2.0f, 0.1f, 0.0f);
    // PID初始化
    Motor_Chassis_Right.PID_Omega.Init(0.2f, 0.25f, 0.0f, 0.0f, 0.0f , 15.0f, 19.0f);

    //Motor_Chassis_Right.Init(&hfdcan2,Motor_DJI_ID_0x201, Motor_DJI_Control_Method_OMEGA);
    Motor_Chassis_Right.Init(&hfdcan2,Motor_DJI_ID_0x201, Motor_DJI_Control_Method_TORQUE);



}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Chassis::RTOS_100ms_Alive_Callback()
{

    Motor_Chassis_Left.RTOS_100ms_Alive_Callback();//轮毂电机
    Motor_Chassis_Right.RTOS_100ms_Alive_Callback();
}
/**
 * @brief 定时器计算函数
 *
 */
void Class_Chassis::RTOS_1ms_Calculate_Callback()//此为最终的函数外调用
{



    //Output();//HACK:



//此为最终的函数内部调用
    Motor_Chassis_Left.RTOS_Calculate_Callback();//轮毂电机
    Motor_Chassis_Right.RTOS_Calculate_Callback();
}
void Class_Chassis::Output()//HACK:功能与原有Callback中的output混淆？
{

// //注意轮子的实际正转速方向
//     Motor_Chassis_Left.Set_Target_Omega(Chassis_Omega_Left);
//     Motor_Chassis_Right.Set_Target_Omega(Chassis_Omega_Right);
}
