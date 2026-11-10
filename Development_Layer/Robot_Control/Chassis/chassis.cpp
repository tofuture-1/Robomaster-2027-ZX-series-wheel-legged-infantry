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


    //以下应写状态转移函数



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


    //状态机初始化
    Chassis_FSM.Init(5,Chassis_STATE_IDLE);

    //HACK:是否在这部分直接初始化？
    Foot_right_VMC.Init();
    Foot_left_VMC.Init();


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


void Class_Chassis::IDLE_Control()
{
    //HACK:优化这部分的实现
    //3508的关断
    Motor_Chassis_Left.Set_Hub_Torque( 0 );//HACK：此处的速度方向分配方式可以优化
    Motor_Chassis_Right.Set_Hub_Torque( 0);

    Motor_Chassis_Left.RTOS_Calculate_Callback();//HACK:滤波器的更新在此，位于 LQR 之后，导致天然晚一个周期？？
    Motor_Chassis_Right.RTOS_Calculate_Callback();//HACK：debug用，后续封装进入RTOS_1ms_Calculate_Callback()


    //8009p的关断
    Foot_LQR.Set_wheel_S(0);//HACK：临时用于清0里程计

    Foot_left_VMC.Set_leg_swing_torque_LQR( 0 );
    Foot_right_VMC.Set_leg_swing_torque_LQR( 0 );

    M8009p_1.Set_Control_Torque(0);
    M8009p_2.Set_Control_Torque(0);

    M8009p_3.Set_Control_Torque( 0);
    M8009p_4.Set_Control_Torque( 0);


    M8009p_1.CAN_Send_Exit();
    M8009p_2.CAN_Send_Exit();
    M8009p_3.CAN_Send_Exit();
    M8009p_4.CAN_Send_Exit();
}
void Class_Chassis::SELF_RESCUE_Control()
{
//HACK：相关数据初始化写在状态转移函数中


    //先实现腿部的姿态自救

    //测试用
    // Foot_left_VMC.Set_phi_des_VMC_Mode( Math_Int_To_Float(FS_I6X.Get_Ibus_msg(3),1000,2000,-1.125f,1.125f) );
    // Foot_right_VMC.Set_phi_des_VMC_Mode( Math_Int_To_Float(FS_I6X.Get_Ibus_msg(3),1000,2000,-1.125f,1.125f) );

    Foot_left_VMC.Set_phi_des_VMC_Mode( 0.0f );
    Foot_right_VMC.Set_phi_des_VMC_Mode( 0.0f );


    M8009p_1.Set_Control_Torque(Foot_right_VMC.Get_Motor_tau1());
    M8009p_2.Set_Control_Torque(Foot_right_VMC.Get_Motor_tau2());

    M8009p_3.Set_Control_Torque( - Foot_left_VMC.Get_Motor_tau1());
    M8009p_4.Set_Control_Torque( - Foot_left_VMC.Get_Motor_tau2());



    M8009p_1.RTOS_Send_Callback();
    M8009p_2.RTOS_Send_Callback();

    M8009p_3.RTOS_Send_Callback();
    M8009p_4.RTOS_Send_Callback();




    //再实现机体的自救



}


void Class_Chassis::NORMAL_LQR_BALANCE_Control()
{
    //HACK：相关数据初始化写在状态转移函数中






    // if (FS_I6X.Get_Ibus_msg(5) >= 1751)//下拨SWB拨杆启动腿长控制
    // {
    //     //theta_polar = Math_Int_To_Float(FS_I6X.Get_Ibus_msg(3),1000,2000,3.925f,5.495f);
    //
    //     //暂时注释掉，先测试定腿长LQR
    //     // Foot_left.Set_L_des(Math_Int_To_Float(FS_I6X.Get_Ibus_msg(2),1000,2000,0.125f,0.330f));
    //     // Foot_right.Set_L_des(Math_Int_To_Float(FS_I6X.Get_Ibus_msg(2),1000,2000,0.125f,0.330f));
    //
    //
    //
    //     // //HACK:测试VMC时用
    //     // Foot_left.Set_leg_swing_torque( Math_Int_To_Float(FS_I6X.Get_Ibus_msg(3),1000,2000,-5.125f,5.125f) );
    //     // Foot_right.Set_leg_swing_torque( Math_Int_To_Float(FS_I6X.Get_Ibus_msg(3),1000,2000,-5.125f,5.125f) );
    //
    //
    // }


    // //HACK：临时debug用
    // if ( fabs(Foot_left.Get_phi()) >1.15f || fabs(Foot_right.Get_phi()) >1.15f )
    // {
    //     safe_flag = 1;
    // }//安全保护
    // //HACK：未完整获取电机数据之前会直接被赋值为1？
    // //fabs(Motor_Chassis_Right.Get_Now_Omega()) > 40.0f || fabs(Motor_Chassis_Left.Get_Now_Omega()) > 40.0f
    //
    // if (FS_I6X.Get_Ibus_msg(6) >= 1751)
    // {
    //     safe_flag = 0;
    // }
    //复位按钮






        // if (FS_I6X.Get_Ibus_msg(4) >= 1751 && state_change_flag == 1)//下拨SWA启动轮毂电机力矩控制
        // {
            //HACK：对转向的控制实现可以优化一下，此外包括传参的参数常量化等
            // Motor_Chassis_Left.Set_Hub_Torque( - (Math_Int_To_Float(FS_I6X.Get_Ibus_msg(1),1000,2000,-10.0f,+10.0f) + Math_Int_To_Float(FS_I6X.Get_Ibus_msg(0),1000,2000,-10.0f,+10.0f)) );
            // Motor_Chassis_Right.Set_Hub_Torque( Math_Int_To_Float(FS_I6X.Get_Ibus_msg(1),1000,2000,-10.0f,+10.0f) - Math_Int_To_Float(FS_I6X.Get_Ibus_msg(0),1000,2000,-10.0f,+10.0f) );

            Motor_Chassis_Left.Set_Hub_Torque_fine( - (+ Math_Int_To_Float(FS_I6X.Get_Ibus_msg(0),1000,2000,-1.0f,+1.0f)) );
            Motor_Chassis_Right.Set_Hub_Torque_fine(  - Math_Int_To_Float(FS_I6X.Get_Ibus_msg(0),1000,2000,-1.0f,+1.0f) );
            //HACK：实际补偿在Set_Hub_Torque的函数定义中实现了

            Foot_LQR.Set_p_dot_fine(Math_Int_To_Float(FS_I6X.Get_Ibus_msg(1),1000,2000,-15.0f,+15.0f));//HACK:此处输入的是转速，需要转换成轮速m/s

            // //HACK:摩擦前馈？
            // //HACK:速度限制？
            Motor_Chassis_Left.Set_Hub_Torque( -Foot_LQR.Get_tau_w() * 0.5f  );//HACK：此处的速度方向分配方式可以优化
            Motor_Chassis_Right.Set_Hub_Torque( Foot_LQR.Get_tau_w() * 0.5f );//HACK:此处的速度补偿在函数定义中实现了

            //RTOS_1ms_Calculate_Callback();
            // Motor_Chassis_Left.RTOS_Calculate_Callback_debug();//HACK:滤波器的更新在此，位于 LQR 之后，导致天然晚一个周期？？
            // Motor_Chassis_Right.RTOS_Calculate_Callback_debug();//HACK：debug用，后续封装进入RTOS_1ms_Calculate_Callback()
            Motor_Chassis_Left.RTOS_Calculate_Callback();//HACK:滤波器的更新在此，位于 LQR 之后，导致天然晚一个周期？？
            Motor_Chassis_Right.RTOS_Calculate_Callback();//HACK：debug用，后续封装进入RTOS_1ms_Calculate_Callback()

        // }else
        // {
        //     //置0，安全保护
        //     Motor_Chassis_Left.Set_Hub_Torque( 0 );//HACK：此处的速度方向分配方式可以优化
        //     Motor_Chassis_Right.Set_Hub_Torque( 0);
        //
        //     Motor_Chassis_Left.RTOS_Calculate_Callback();//HACK:滤波器的更新在此，位于 LQR 之后，导致天然晚一个周期？？
        //     Motor_Chassis_Right.RTOS_Calculate_Callback();//HACK：debug用，后续封装进入RTOS_1ms_Calculate_Callback()
        //
        //
        // }


    // if (FS_I6X.Get_Ibus_msg(5) >= 1751 && state_change_flag == 1 )//下拨SWB拨杆启动腿长控制
    // {

        // // //HACK:测试VMC时用
        // Foot_left.Set_leg_swing_torque( Math_Int_To_Float(FS_I6X.Get_Ibus_msg(3),1000,2000,-5.125f,5.125f) );
        // Foot_right.Set_leg_swing_torque( Math_Int_To_Float(FS_I6X.Get_Ibus_msg(3),1000,2000,-5.125f,5.125f) );


        Foot_left_VMC.Set_leg_swing_torque_LQR(Foot_LQR.Get_tau_leg() * 0.5f);
        Foot_right_VMC.Set_leg_swing_torque_LQR(Foot_LQR.Get_tau_leg() * 0.5f);




        M8009p_1.Set_Control_Torque(Foot_right_VMC.Get_Motor_tau1());
        M8009p_2.Set_Control_Torque(Foot_right_VMC.Get_Motor_tau2());

        M8009p_3.Set_Control_Torque( - Foot_left_VMC.Get_Motor_tau1());
        M8009p_4.Set_Control_Torque( - Foot_left_VMC.Get_Motor_tau2());



        // M8009p_1.RTOS_Send_Callback_debug();
        // M8009p_2.RTOS_Send_Callback_debug();//TODO：为什么直接调用类原版的Callback函数一样会触发通信丢失错误(超时）,Enable里的output被禁用的话就会导致单片机获得的电机状态是enable但实际已经丢失通信？
        // //TODO::达妙电机的回传数据似乎在上位机中不失能也可以收到，但为何这里不行？
        // M8009p_3.RTOS_Send_Callback_debug();
        // M8009p_4.RTOS_Send_Callback_debug();
        //DONE：2610161321：VMC的解算基本跑通


        M8009p_1.RTOS_Send_Callback();
        M8009p_2.RTOS_Send_Callback();
        //TODO::达妙电机的回传数据似乎在上位机中不失能也可以收到，但为何这里不行？
        M8009p_3.RTOS_Send_Callback();
        M8009p_4.RTOS_Send_Callback();
        //DONE：2610161321：VMC的解算基本跑通
    // }else
    // {
    //     Foot_LQR.Set_wheel_S(0);//HACK：临时用于清0里程计
    //
    //     Foot_left.Set_leg_swing_torque( 0 );
    //     Foot_right.Set_leg_swing_torque( 0 );
    //
    //     M8009p_1.Set_Control_Torque(0);
    //     M8009p_2.Set_Control_Torque(0);
    //
    //     M8009p_3.Set_Control_Torque( 0);
    //     M8009p_4.Set_Control_Torque( 0);
    //
    //     // M8009p_1.RTOS_Send_Callback();
    //     // M8009p_2.RTOS_Send_Callback();
    //     // //TODO::达妙电机的回传数据似乎在上位机中不失能也可以收到，但为何这里不行？
    //     // M8009p_3.RTOS_Send_Callback();
    //     // M8009p_4.RTOS_Send_Callback();
    //
    //     M8009p_1.CAN_Send_Exit();
    //     M8009p_2.CAN_Send_Exit();
    //     M8009p_3.CAN_Send_Exit();
    //     M8009p_4.CAN_Send_Exit();
    //
    //
    // }
}




void Class_Chassis::JUMPING_Control()
{

}
void Class_Chassis::OFF_GROUND_Control()
{

}


void Class_Chassis::Chassis_State_Update()
{
    //VMC数据传入，TODO：部分与LQR共用的运动学解析数据应该解耦
    Foot_right_VMC.Set_theta1(M8009p_1.Get_Now_Angle() - 0.16802f);//DONE：202610152130为什么上位机读数值为0.246时此函数Get到的值为12.7几-0.016802:此前的Data_Prgress中的角度解析有误
    Foot_right_VMC.Set_theta2(M8009p_2.Get_Now_Angle() + 0.16802f);

    Foot_left_VMC.Set_theta1(-(M8009p_3.Get_Now_Angle() + 0.16802f));
    Foot_left_VMC.Set_theta2(-(M8009p_4.Get_Now_Angle() - 0.16802f));//HACK:此处临时用补偿和取反,此函数名是否需要再优化以防止误解为设置目标角度（实际只是将回传角度传入）

    Foot_right_VMC.Set_theta1_dot(M8009p_1.Get_Now_Omega());
    Foot_right_VMC.Set_theta2_dot(M8009p_2.Get_Now_Omega());
    Foot_left_VMC.Set_theta1_dot( - (M8009p_3.Get_Now_Omega()) );
    Foot_left_VMC.Set_theta2_dot( - (M8009p_4.Get_Now_Omega()) );


    Foot_right_VMC.VMC_Calculate();
    Foot_left_VMC.VMC_Calculate();//计算相关值，但未计算力矩

    //左右腿对称补偿
    Foot_right_VMC.Set_delta_leg_rl_phi( -( Foot_left_VMC.Get_phi() - Foot_right_VMC.Get_phi() ) *0.5f);
    Foot_right_VMC.Set_delta_leg_rl_phi_dot( -(Foot_left_VMC.Get_phi_dot() - Foot_right_VMC.Get_phi_dot() ) * 0.5f);
    Foot_left_VMC.Set_delta_leg_rl_phi(  ( Foot_left_VMC.Get_phi() - Foot_right_VMC.Get_phi() ) *0.5f);
    Foot_left_VMC.Set_delta_leg_rl_phi_dot(  (Foot_left_VMC.Get_phi_dot() - Foot_right_VMC.Get_phi_dot() ) * 0.5f);
    //HACK：解决剪刀腿问题,此处的赋值、调用逻辑需要优化//该项因为phi_dot的噪声较多所以引用阻尼D项时会导致疯？


    //LQR数据传入
    Foot_LQR.Set_theta(Pitch);//HACK：减去180.0f在该函数内部进行——关于归一化的逻辑设计
    Foot_LQR.Set_theta_dot(Pitch_dot);//Get_Gyro_X单位是弧度
    //HACK:传入数据在外部进行?

    Foot_LQR.Set_phi_dot( (Foot_left_VMC.Get_phi_dot() + Foot_right_VMC.Get_phi_dot() ) * 0.5f ) ;
    Foot_LQR.Set_phi( ( Foot_left_VMC.Get_phi() + Foot_right_VMC.Get_phi() ) *0.5f );//对称处理，状态平均
    Foot_LQR.Set_wheel_speed( 0.5f * ( Motor_Chassis_Right.Get_Now_Omega() - Motor_Chassis_Left.Get_Now_Omega() ) );
    //HACK：3508默认正方向为电机输出轴的逆时针方向
    //DONE：之前传入的速度方向错了导致一直在发散(............?)
    //HACK：注意滤波器的更新在下面的RTOS_Calculate_Callback中，会晚一个周期，TODO：考虑解耦？
    //HACK：此处的速度方向分配方式可以优化


    Foot_LQR.LQR_Calculate();


    Foot_left_VMC.VMC_Motor_tau_map();
    Foot_right_VMC.VMC_Motor_tau_map();//计算实际力矩输出


    /*以上为数据更新及计算*///HACK:解耦





    /*以下为状态转移函数*/

    Chassis_FSM.RTOS_Calculate_Callback();//原状态转移函数应在处

//HACK:目前的断控需要与FS_I6X 的失控保护配合

    //自动检测部分
    //HACK：临时debug用，超出LQR可控范围检测（暂时先检测腿部摆角）
    if ( fabs(Foot_left_VMC.Get_phi()) > 0.78498f || fabs(Foot_right_VMC.Get_phi()) > 0.78498f )//摆角偏离0位超过接近45°时进入，注意不要留出死区
    {
        if (Chassis_FSM.Get_Now_Status_Serial() != Chassis_STATE_IDLE && state_change_flag != 1
            && (fabs(Foot_left_VMC.Get_phi()) > 0.98498f || fabs(Foot_right_VMC.Get_phi()) > 0.98498f))
        {
            Chassis_FSM.Set_Status(Chassis_STATE_IDLE);
        }else if (Chassis_FSM.Get_Now_Status_Serial() != Chassis_STATE_SELF_RESCUE && state_change_flag == 1)
        {
            //HACK:需要先清空一下所有状态
            IDLE_Control();//HACK:解耦

            Chassis_FSM.Set_Status(Chassis_STATE_SELF_RESCUE);//进入自救

            //HACK:相关参数初始化
            Foot_right_VMC.Init(0.12f,130.0f,30.0f,-12.0f,10.0f,2.0f,VMC_Mode,10.15f,2.15f);
            Foot_left_VMC.Init(0.12f,130.0f,30.0f,-12.0f,10.0f,2.0f,VMC_Mode,10.15f,2.15f);

        }
    }else
    {
        if (Chassis_FSM.Get_Now_Status_Serial() != Chassis_STATE_NORMAL_LQR_BALANCE &&
            fabs(Foot_left_VMC.Get_phi()) < 0.52333f && fabs(Foot_right_VMC.Get_phi()) < 0.52333f)//0.52333对应30° 0.1744444对应10° 0.78498f对应45°
        {

            if (state_change_flag == 1 && FS_I6X.Get_Ibus_msg(5) >= 1751)//HACK：优化FS_I6X对外的拨杆读取函数
            {
                Foot_right_VMC.Init();
                Foot_left_VMC.Init();//HACK：重置参数

                //先清空原数据
                IDLE_Control();//HACK:解耦
                Chassis_FSM.Set_Status(Chassis_STATE_NORMAL_LQR_BALANCE);
            }else//XXX：小于限幅摆角后就不会再进来了
            {
                if (Chassis_FSM.Get_Now_Status_Serial() != Chassis_STATE_IDLE || state_change_flag != 1)
                {
                    IDLE_Control();//HACK:解耦
                    Chassis_FSM.Set_Status(Chassis_STATE_IDLE);
                }
            }

        }

    }


    //人控部分
    if (FS_I6X.Get_Ibus_msg(6) >= 1751)//HACK：注意与上部是否冲突
    {
        state_change_flag =1;//HACK:临时用
        //启用状态机自动切换控制
    }else
    {
        state_change_flag = 0;
        Chassis_FSM.Set_Status(Chassis_STATE_IDLE);
    }
}


/**
 * @brief 定时器计算函数
 *
 */
void Class_Chassis::RTOS_1ms_Calculate_Callback()//此为最终的函数外调用HACK:本函数目前未使用
{



    //Output();//HACK:



//此为最终的函数内部调用,HACK:本函数目前未使用，直接在其他函数中调用了下面的函数
    Motor_Chassis_Left.RTOS_Calculate_Callback();//轮毂电机
    Motor_Chassis_Right.RTOS_Calculate_Callback();
}
void Class_Chassis::Output()//HACK:功能与原有Callback中的output混淆？
{

// //注意轮子的实际正转速方向
//     Motor_Chassis_Left.Set_Target_Omega(Chassis_Omega_Left);
//     Motor_Chassis_Right.Set_Target_Omega(Chassis_Omega_Right);
}
