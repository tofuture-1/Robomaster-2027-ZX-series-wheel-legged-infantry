//
// Created by 谭恩泽 on 2025/10/23.
//

#include "booster.h"


/**
 * @brief C610拨弹盘电机: 滤波后PID
 *
 */
void Class_Booster_Driver_Motor_DJI_C610::RTOS_Calculate_Callback()
{
    Filter_Kalman_Omega.Set_Measurement(Rx_Data.Now_Omega);
    Filter_Kalman_Omega.Filter_Calculate();
    after_filter_Omega = Filter_Kalman_Omega.Get_Out();

    PID_Calculate();

    float tmp_value = Target_Current + Feedforward_Current;
    Out = tmp_value * Current_To_Out;
    Math_Constrain(&Out, -Current_Max, Current_Max);

    Output();

    Feedforward_Current = 0.0f;
    Feedforward_Omega = 0.0f;
}

void Class_Booster_Driver_Motor_DJI_C610::PID_Calculate()
{
    switch (Motor_DJI_Control_Method)
    {
    case (Motor_DJI_Control_Method_CURRENT):
        break;
    case (Motor_DJI_Control_Method_OMEGA):
        PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
        PID_Omega.Set_Now(after_filter_Omega);
        PID_Omega.PID_Process();
        Target_Current = PID_Omega.Get_Out();
        break;
    case (Motor_DJI_Control_Method_ANGLE):
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Rx_Data.Now_Angle);
        PID_Angle.PID_Process();
        Target_Omega = PID_Angle.Get_Out();

        PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
        PID_Omega.Set_Now(after_filter_Omega);
        PID_Omega.PID_Process();
        Target_Current = PID_Omega.Get_Out();
        break;
    default:
        Target_Current = 0.0f;
        break;
    }
}

/**
 * @brief TIM定时器中断计算回调函数, 计算周期取决于电机反馈周期
 *
 */
void Class_Booster_Friction_Motor_DJI_C620::RTOS_Calculate_Callback()
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
void Class_Booster_Friction_Motor_DJI_C620::PID_Calculate()
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



/**
 * @brief 卡弹策略有限自动机
 *
 */
void Class_FSM_Anti_Jamming::RTOS_1ms_Calculate_Callback()
{
    Status[Now_Status_Serial].Count_Time++;

    // 自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
    case (FSM_Anti_Jamming_Status_NORMAL):
    {
        // 正常状态

        if (Booster->Motor_Driver.Get_Now_Current() >= Driver_Torque_Threshold)
        {
            // 大扭矩->卡弹嫌疑状态
            Set_Status(FSM_Anti_Jamming_Status_JAMMING_SUSPECT);
        }

        break;
    }
    case (FSM_Anti_Jamming_Status_JAMMING_SUSPECT):
    {
        // 卡弹嫌疑状态

        if (Status[Now_Status_Serial].Count_Time >= Jamming_Suspect_Time_Threshold)
        {
            // 长时间大扭矩->卡弹反应状态
            Set_Status(FSM_Anti_Jamming_Status_JAMMING_CONFIRM);
        }
        else if (Booster->Motor_Driver.Get_Now_Current() < Driver_Torque_Threshold)
        {
            // 短时间大扭矩->正常状态
            Set_Status(FSM_Anti_Jamming_Status_NORMAL);
        }

        break;
    }
    case (FSM_Anti_Jamming_Status_JAMMING_CONFIRM):
    {
        // 卡弹反应状态->准备卡弹处理

        Booster->Motor_Driver.Set_Control_Method(Motor_DJI_Control_Method_ANGLE);
        Booster->Motor_Driver.Set_Target_Angle(Booster->Motor_Driver.Get_Now_Angle() - Driver_Back_Angle);

        Set_Status(FSM_Anti_Jamming_Status_PROCESSING);

        break;
    }
    case (FSM_Anti_Jamming_Status_PROCESSING):
    {
        // 卡弹处理状态

        if (Status[Now_Status_Serial].Count_Time >= Jamming_Solving_Time_Threshold)
        {
            // 长时间回拨->正常状态
            Set_Status(FSM_Anti_Jamming_Status_NORMAL);
        }

        break;
    }
    }
}

/**
 * @brief 发射机构初始化
 *
 */
void Class_Booster::Init()
{

    // 正常状态, 卡弹嫌疑状态, 卡弹反应状态, 卡弹处理状态
    FSM_Anti_Jamming.Booster = this;
    FSM_Anti_Jamming.Init(4, FSM_Anti_Jamming_Status_NORMAL);

    // 拨弹盘电机初始化

    // 滤波器初始化
    Motor_Driver.Filter_Kalman_Omega.Init(0.01f, 2.0f, 0.1f, 0.0f);
    // PID初始化
    Motor_Driver.PID_Angle.Init(2.5f, 0.0f, 0.001f, 0.0f, 0.0f, PI, 4.0f * PI);
    Motor_Driver.PID_Omega.Init(0.5f, 0.5f, 0.0f, 0.0f, 0.0f, 10.0f, 10.0f);
    // 电机初始化
    Motor_Driver.Init(&hfdcan1, Motor_DJI_ID_0x203, Motor_DJI_Control_Method_OMEGA);

    // 摩擦轮电机左

    // 滤波器初始化
    Motor_Friction_Left.Filter_Kalman_Omega.Init(0.01f, 2.0f, 0.1f, 0.0f);
    // PID初始化
    Motor_Friction_Left.PID_Omega.Init(0.2f, 0.25f, 0.0f, 0.0f, 0.0f , 15.0f, 20.0f);
    // 电机初始化
    Motor_Friction_Left.Init(&hfdcan1, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_OMEGA, 1.0f);

    // 摩擦轮电机右

    // 滤波器初始化
    Motor_Friction_Right.Filter_Kalman_Omega.Init(0.01f, 2.0f, 0.1f, 0.0f);
    // PID初始化
    Motor_Friction_Right.PID_Omega.Init(0.2f, 0.25f, 0.0f, 0.0f, 0.0f, 15.0f, 20.0f);
    // 电机初始化
    Motor_Friction_Right.Init(&hfdcan1, Motor_DJI_ID_0x202, Motor_DJI_Control_Method_OMEGA, 1.0f);
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Booster::RTOS_100ms_Alive_Callback()
{
    Motor_Driver.RTOS_100ms_Alive_Callback();
    Motor_Friction_Left.RTOS_100ms_Alive_Callback();
    Motor_Friction_Right.RTOS_100ms_Alive_Callback();
}

/**
 * @brief 定时器计算函数
 *
 */
void Class_Booster::RTOS_1ms_Calculate_Callback()
{
    FSM_Anti_Jamming.RTOS_Calculate_Callback();

    // 如果有限状态机正常则执行拨弹逻辑, 否则由有限状态机进行卡弹处理
    if (FSM_Anti_Jamming.Get_Now_Status_Serial() == FSM_Anti_Jamming_Status_NORMAL || FSM_Anti_Jamming.Get_Now_Status_Serial() == FSM_Anti_Jamming_Status_JAMMING_SUSPECT)
    {
        Output();
    }

    Motor_Driver.RTOS_Calculate_Callback();
    Motor_Friction_Left.RTOS_Calculate_Callback();
    Motor_Friction_Right.RTOS_Calculate_Callback();
}




/**
 * @brief 输出到电机
 *
 */
void Class_Booster::Output()
{
    switch (Booster_Control_Type)
    {
    case (Booster_Control_Type_DISABLE):
    {
        // 发射机构失能

        Motor_Driver.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor_Friction_Left.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor_Friction_Right.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);

        Motor_Driver.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Driver.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Friction_Left.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Friction_Right.PID_Omega.Set_Integral_Error(0.0f);

        Motor_Driver.Set_Target_Omega(0.0f);
        Motor_Friction_Left.Set_Target_Omega(0.0f);
        Motor_Friction_Right.Set_Target_Omega(0.0f);

        break;
    }
    case (Booster_Control_Type_CEASEFIRE):
    {
        // 停火

        Now_Ammo_Shoot_Frequency = 0.0f;
        if (Motor_Driver.Get_Control_Method() == Motor_DJI_Control_Method_ANGLE)
        {
        }
        else if (Motor_Driver.Get_Control_Method() == Motor_DJI_Control_Method_OMEGA)
        {
            Motor_Driver.Set_Target_Omega(0.0f);
        }
        Motor_Friction_Left.Set_Target_Omega(Friction_Omega);
        Motor_Friction_Right.Set_Target_Omega(-Friction_Omega);

        break;
    }
    case (Booster_Control_Type_SPOT):
    {
        // 单发模式

        Motor_Driver.Set_Control_Method(Motor_DJI_Control_Method_ANGLE);
        Motor_Friction_Left.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor_Friction_Right.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);

        Motor_Driver.Set_Target_Angle(Motor_Driver.Get_Now_Angle() + 2.0f * PI / Ammo_Num_Per_Round);
        Motor_Friction_Left.Set_Target_Omega(Friction_Omega);
        Motor_Friction_Right.Set_Target_Omega(-Friction_Omega);

        // 打出去立刻停火
        Booster_Control_Type = Booster_Control_Type_CEASEFIRE;

        break;
    }
    case (Booster_Control_Type_AUTO):
    {
        // 自动模式

        // 热量控制
        float tmp_delta = Heat_Limit_Max - Now_Heat;
        if (tmp_delta >= Heat_Limit_Slowdown_Threshold)
        {
            Now_Ammo_Shoot_Frequency = Target_Ammo_Shoot_Frequency;
        }
        else if (tmp_delta >= Heat_Limit_Ceasefire_Threshold && tmp_delta < Heat_Limit_Slowdown_Threshold)
        {
            Now_Ammo_Shoot_Frequency = (Target_Ammo_Shoot_Frequency * (Heat_Limit_Ceasefire_Threshold - tmp_delta) + Heat_CD / 10.0f * (tmp_delta - Heat_Limit_Slowdown_Threshold)) / (Heat_Limit_Ceasefire_Threshold - Heat_Limit_Slowdown_Threshold);
        }
        else if (tmp_delta < Heat_Limit_Ceasefire_Threshold)
        {
            Now_Ammo_Shoot_Frequency = 0.0f;
        }

        Motor_Driver.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor_Friction_Left.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor_Friction_Right.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);

        Motor_Friction_Left.Set_Target_Omega(Friction_Omega);
        Motor_Friction_Right.Set_Target_Omega(-Friction_Omega);

        if (Driver_Direction == 1)
        {
            Motor_Driver.Set_Target_Omega(Now_Ammo_Shoot_Frequency * 2.0f * PI / Ammo_Num_Per_Round);
        }
        else if (Driver_Direction == -1)
        {
            Motor_Driver.Set_Target_Omega(-10.0f);
        }
        else if (Driver_Direction == 0)
        {
            Motor_Driver.Set_Target_Omega(0.0f);
        }
        break;
    }
    }
}
