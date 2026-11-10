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

    Out = tmp_value * Current_To_Out;
    Math_Constrain(&Out, -Current_Max, Current_Max);

    Output();
}

/**
 * @brief TIM定时器中断计算回调函数, 计算周期取决于电机反馈周期,不输出，debug用
 *
 */
void Class_Chassis_Motor_DJI_C620::RTOS_Calculate_Callback_debug()
{
    Filter_Kalman_Omega.Set_Measurement(Rx_Data.Now_Omega);
    Filter_Kalman_Omega.Filter_Calculate();
    after_filter_Omega = Filter_Kalman_Omega.Get_Out();

    PID_Calculate();

    float tmp_value = Target_Current + Feedforward_Current;
    // Math_Constrain(&tmp_value, -Current_Max, Current_Max);原来框架中的错误限幅
    //Out = tmp_value * Current_To_Out;
    Out = 0;//debug用
    Math_Constrain(&Out, -Current_Max, Current_Max);
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


void Class_FSM_Chassis::RTOS_Calculate_Callback()
{
    Status[Now_Status_Serial].Count_Time++;

    switch (Now_Status_Serial)
    {
        case (Chassis_STATE_SELF_RESCUE):
            {

                break;
            }
        case (Chassis_STATE_NORMAL_LQR_BALANCE):
            {

                break;
            }
        case (Chassis_STATE_JUMPING):
            {

                break;
            }
        case (Chassis_STATE_OFF_GROUND):
            {

                break;
            }
        default:
            {

                break;
            }

    }


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




void Class_Chassis::Normal_LQR_Balance_Control()
{

//VMC数据传入，TODO：部分与LQR共用的运动学解析数据应该解耦
    Foot_right.Set_theta1(M8009p_1.Get_Now_Angle() - 0.16802f);//DONE：202610152130为什么上位机读数值为0.246时此函数Get到的值为12.7几-0.016802:此前的Data_Prgress中的角度解析有误
    Foot_right.Set_theta2(M8009p_2.Get_Now_Angle() + 0.16802f);

    Foot_left.Set_theta1(-(M8009p_3.Get_Now_Angle() + 0.16802f));
    Foot_left.Set_theta2(-(M8009p_4.Get_Now_Angle() - 0.16802f));//HACK:此处临时用补偿和取反,此函数名是否需要再优化以防止误解为设置目标角度（实际只是将回传角度传入）

    Foot_right.Set_theta1_dot(M8009p_1.Get_Now_Omega());
    Foot_right.Set_theta2_dot(M8009p_2.Get_Now_Omega());
    Foot_left.Set_theta1_dot( - (M8009p_3.Get_Now_Omega()) );
    Foot_left.Set_theta2_dot( - (M8009p_4.Get_Now_Omega()) );





    if (FS_I6X.Get_Ibus_msg(5) >= 1751)//下拨SWB拨杆启动腿长控制
    {
        //theta_polar = Math_Int_To_Float(FS_I6X.Get_Ibus_msg(3),1000,2000,3.925f,5.495f);

        //暂时注释掉，先测试定腿长LQR
        // Foot_left.Set_L_des(Math_Int_To_Float(FS_I6X.Get_Ibus_msg(2),1000,2000,0.125f,0.330f));
        // Foot_right.Set_L_des(Math_Int_To_Float(FS_I6X.Get_Ibus_msg(2),1000,2000,0.125f,0.330f));



        // //HACK:测试VMC时用
        // Foot_left.Set_leg_swing_torque( Math_Int_To_Float(FS_I6X.Get_Ibus_msg(3),1000,2000,-5.125f,5.125f) );
        // Foot_right.Set_leg_swing_torque( Math_Int_To_Float(FS_I6X.Get_Ibus_msg(3),1000,2000,-5.125f,5.125f) );


    }
    Foot_right.VMC_Calculate();
    Foot_left.VMC_Calculate();//计算相关值，但未计算力矩

    //左右腿对称补偿
    Foot_right.Set_delta_leg_rl_phi( -( Foot_left.Get_phi() - Foot_right.Get_phi() ) *0.5f);
    Foot_right.Set_delta_leg_rl_phi_dot( -(Foot_left.Get_phi_dot() - Foot_right.Get_phi_dot() ) * 0.5f);
    Foot_left.Set_delta_leg_rl_phi(  ( Foot_left.Get_phi() - Foot_right.Get_phi() ) *0.5f);
    Foot_left.Set_delta_leg_rl_phi_dot(  (Foot_left.Get_phi_dot() - Foot_right.Get_phi_dot() ) * 0.5f);
    //HACK：解决剪刀腿问题,此处的赋值、调用逻辑需要优化//该项因为phi_dot的噪声较多所以引用阻尼D项时会导致疯？

    //HACK：临时debug用
    if ( fabs(Foot_left.Get_phi()) >1.15f || fabs(Foot_right.Get_phi()) >1.15f )
    {
        safe_flag = 1;
    }//安全保护
    //HACK：未完整获取电机数据之前会直接被赋值为1？
    //fabs(Chassis.Motor_Chassis_Right.Get_Now_Omega()) > 40.0f || fabs(Chassis.Motor_Chassis_Left.Get_Now_Omega()) > 40.0f

    if (FS_I6X.Get_Ibus_msg(6) >= 1751)
    {
        safe_flag = 0;
    }
    //复位按钮



    //LQR数据传入
    Foot_LQR.Set_theta(BMI088.Get_Pitch());//HACK：减去180.0f在该函数内部进行——关于归一化的逻辑设计
    Foot_LQR.Set_theta_dot(BMI088.Get_Gyro_X());//Get_Gyro_X单位是弧度
    Foot_LQR.Set_phi_dot( (Foot_left.Get_phi_dot() + Foot_right.Get_phi_dot() ) * 0.5f ) ;
    Foot_LQR.Set_phi( ( Foot_left.Get_phi() + Foot_right.Get_phi() ) *0.5f );//对称处理，状态平均
    Foot_LQR.Set_wheel_speed( 0.5f * ( Chassis.Motor_Chassis_Right.Get_Now_Omega() - Chassis.Motor_Chassis_Left.Get_Now_Omega() ) );
    //HACK：3508默认正方向为电机输出轴的逆时针方向
    //DONE：之前传入的速度方向错了导致一直在发散！！！！！

    //HACK：注意滤波器的更新在下面的RTOS_Calculate_Callback中，会晚一个周期，TODO：是否做解耦？

    //HACK：此处的速度方向分配方式可以优化,与下方的输出会联动？

    // Foot_LQR_Right.Set_theta(BMI088.Get_Pitch());
    // Foot_LQR_Right.Set_theta_dot(BMI088.Get_Gyro_X());//Get_Gyro_X单位是弧度
    // Foot_LQR_Right.Set_phi_dot(Foot_right.Get_phi_dot());
    // Foot_LQR_Right.Set_phi(Foot_right.Get_phi());
    // Foot_LQR_Right.Set_wheel_speed(  Chassis.Motor_Chassis_Right.Get_Now_Omega() );//HACK：此处的速度方向分配方式可以优化,与下方的输出会联动？

    Foot_LQR.LQR_Calculate();
    //Foot_LQR_Right.LQR_Calculate();

        if (FS_I6X.Get_Ibus_msg(4) >= 1751 && safe_flag == 0)//下拨SWA启动轮毂电机力矩控制
        {
            //HACK：对转向的控制实现可以优化一下，此外包括传参的参数常量化等
            // Chassis.Motor_Chassis_Left.Set_Hub_Torque( - (Math_Int_To_Float(FS_I6X.Get_Ibus_msg(1),1000,2000,-10.0f,+10.0f) + Math_Int_To_Float(FS_I6X.Get_Ibus_msg(0),1000,2000,-10.0f,+10.0f)) );
            // Chassis.Motor_Chassis_Right.Set_Hub_Torque( Math_Int_To_Float(FS_I6X.Get_Ibus_msg(1),1000,2000,-10.0f,+10.0f) - Math_Int_To_Float(FS_I6X.Get_Ibus_msg(0),1000,2000,-10.0f,+10.0f) );

            Chassis.Motor_Chassis_Left.Set_Hub_Torque_fine( - (+ Math_Int_To_Float(FS_I6X.Get_Ibus_msg(0),1000,2000,-1.0f,+1.0f)) );
            Chassis.Motor_Chassis_Right.Set_Hub_Torque_fine(  - Math_Int_To_Float(FS_I6X.Get_Ibus_msg(0),1000,2000,-1.0f,+1.0f) );
            //HACK：实际补偿在Set_Hub_Torque的函数定义中实现了

            Foot_LQR.Set_p_dot_fine(Math_Int_To_Float(FS_I6X.Get_Ibus_msg(1),1000,2000,-10.0f,+10.0f));//HACK:此处输入的是转速，需要转换成轮速m/s

            // //HACK:摩擦前馈？
            // //HACK:速度限制？
            Chassis.Motor_Chassis_Left.Set_Hub_Torque( -Foot_LQR.Get_tau_w() * 0.5f  );//HACK：此处的速度方向分配方式可以优化
            Chassis.Motor_Chassis_Right.Set_Hub_Torque( Foot_LQR.Get_tau_w() * 0.5f );//HACK:此处的速度补偿在函数定义中实现了

            //Chassis.RTOS_1ms_Calculate_Callback();
            // Chassis.Motor_Chassis_Left.RTOS_Calculate_Callback_debug();//HACK:滤波器的更新在此，位于 LQR 之后，导致天然晚一个周期？？
            // Chassis.Motor_Chassis_Right.RTOS_Calculate_Callback_debug();//HACK：debug用，后续封装进入Chassis.RTOS_1ms_Calculate_Callback()
            Chassis.Motor_Chassis_Left.RTOS_Calculate_Callback();//HACK:滤波器的更新在此，位于 LQR 之后，导致天然晚一个周期？？
            Chassis.Motor_Chassis_Right.RTOS_Calculate_Callback();//HACK：debug用，后续封装进入Chassis.RTOS_1ms_Calculate_Callback()

        }else
        {
            //置0，安全保护
            Chassis.Motor_Chassis_Left.Set_Hub_Torque( 0 );//HACK：此处的速度方向分配方式可以优化
            Chassis.Motor_Chassis_Right.Set_Hub_Torque( 0);

            Chassis.Motor_Chassis_Left.RTOS_Calculate_Callback();//HACK:滤波器的更新在此，位于 LQR 之后，导致天然晚一个周期？？
            Chassis.Motor_Chassis_Right.RTOS_Calculate_Callback();//HACK：debug用，后续封装进入Chassis.RTOS_1ms_Calculate_Callback()


        }

    Foot_left.VMC_Motor_tau_map();
    Foot_right.VMC_Motor_tau_map();//计算实际力矩输出
    if (FS_I6X.Get_Ibus_msg(5) >= 1751 && safe_flag == 0 )//下拨SWB拨杆启动腿长控制
    {

        // // //HACK:测试VMC时用
        // Foot_left.Set_leg_swing_torque( Math_Int_To_Float(FS_I6X.Get_Ibus_msg(3),1000,2000,-5.125f,5.125f) );
        // Foot_right.Set_leg_swing_torque( Math_Int_To_Float(FS_I6X.Get_Ibus_msg(3),1000,2000,-5.125f,5.125f) );


        Foot_left.Set_leg_swing_torque(Foot_LQR.Get_tau_leg() * 0.5f);
        Foot_right.Set_leg_swing_torque(Foot_LQR.Get_tau_leg() * 0.5f);




        Chassis.M8009p_1.Set_Control_Torque(Foot_right.Get_Motor_tau1());
        Chassis.M8009p_2.Set_Control_Torque(Foot_right.Get_Motor_tau2());

        Chassis.M8009p_3.Set_Control_Torque( - Foot_left.Get_Motor_tau1());
        Chassis.M8009p_4.Set_Control_Torque( - Foot_left.Get_Motor_tau2());



        // Chassis.M8009p_1.RTOS_Send_Callback_debug();
        // Chassis.M8009p_2.RTOS_Send_Callback_debug();//TODO：为什么直接调用类原版的Callback函数一样会触发通信丢失错误(超时）,Enable里的output被禁用的话就会导致单片机获得的电机状态是enable但实际已经丢失通信？
        // //TODO::达妙电机的回传数据似乎在上位机中不失能也可以收到，但为何这里不行？
        // Chassis.M8009p_3.RTOS_Send_Callback_debug();
        // Chassis.M8009p_4.RTOS_Send_Callback_debug();
        //DONE：2610161321：VMC的解算基本跑通


        Chassis.M8009p_1.RTOS_Send_Callback();
        Chassis.M8009p_2.RTOS_Send_Callback();
        //TODO::达妙电机的回传数据似乎在上位机中不失能也可以收到，但为何这里不行？
        Chassis.M8009p_3.RTOS_Send_Callback();
        Chassis.M8009p_4.RTOS_Send_Callback();
        //DONE：2610161321：VMC的解算基本跑通
    }else
    {
        Foot_LQR.Set_wheel_S(0);//HACK：临时用于清0里程计

        Foot_left.Set_leg_swing_torque( 0 );
        Foot_right.Set_leg_swing_torque( 0 );

        Chassis.M8009p_1.Set_Control_Torque(0);
        Chassis.M8009p_2.Set_Control_Torque(0);

        Chassis.M8009p_3.Set_Control_Torque( 0);
        Chassis.M8009p_4.Set_Control_Torque( 0);

        // Chassis.M8009p_1.RTOS_Send_Callback();
        // Chassis.M8009p_2.RTOS_Send_Callback();
        // //TODO::达妙电机的回传数据似乎在上位机中不失能也可以收到，但为何这里不行？
        // Chassis.M8009p_3.RTOS_Send_Callback();
        // Chassis.M8009p_4.RTOS_Send_Callback();

        Chassis.M8009p_1.CAN_Send_Exit();
        Chassis.M8009p_2.CAN_Send_Exit();
        Chassis.M8009p_3.CAN_Send_Exit();
        Chassis.M8009p_4.CAN_Send_Exit();


    }
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
