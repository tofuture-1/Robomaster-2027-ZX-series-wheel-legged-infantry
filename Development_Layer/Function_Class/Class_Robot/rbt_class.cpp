//
// Created by ̷���� on 2025/10/23.
//

#include "rbt_class.h"
#include "arm_math.h"
#include "dr16.h"
#include "drv_math.h"
#include "imu.h"
#include "usart.h"

CLASS_ROBOT* CLASS_ROBOT::Instance = nullptr;

volatile bool robot_1ms_cycle_flag = false;
volatile bool robot_100ms_cycle_flag = false;
volatile bool robot_1000ms_cycle_flag = false;

// ���鲹���Ƕ�
const float AIM_Yaw = 0.0f * DEG_TO_RAD;  // 11.0
const float AIM_Pitch = 0.0f * DEG_TO_RAD;  // -5.0


// �������ȹ��ʴ��
static const float chassis_power_first_power[10] = {60.0f,
                                                    65.0f,
                                                    70.0f,
                                                    75.0f,
                                                    80.0f,
                                                    85.0f,
                                                    90.0f,
                                                    95.0f,
                                                    100.0f,
                                                    100.0f,};

// Ѫ�����ȹ��ʴ��
static const float chassis_hp_first_power[10] = {45.0f,
                                                 50.0f,
                                                 55.0f,
                                                 60.0f,
                                                 65.0f,
                                                 70.0f,
                                                 75.0f,
                                                 80.0f,
                                                 90.0f,
                                                 100.0f,};

// ���������������
static const float booster_burst_first_heat_max[10] = {170.0f,
                                                       180.0f,
                                                       190.0f,
                                                       200.0f,
                                                       210.0f,
                                                       220.0f,
                                                       230.0f,
                                                       240.0f,
                                                       250.0f,
                                                       260.0f,};

// ����������ȴ���
static const float booster_burst_first_heat_cd[10] = {5.0f,
                                                      7.0f,
                                                      9.0f,
                                                      11.0f,
                                                      12.0f,
                                                      13.0f,
                                                      14.0f,
                                                      16.0f,
                                                      18.0f,
                                                      20.0f,};

// ��ȴ�����������
static const float booster_cd_first_heat_max[10] = {40.0f,
                                                    48.0f,
                                                    56.0f,
                                                    64.0f,
                                                    72.0f,
                                                    80.0f,
                                                    88.0f,
                                                    96.0f,
                                                    114.0f,
                                                    120.0f,};

// ��ȴ������ȴ���
static const float booster_cd_first_heat_cd[10] = {12.0f,
                                                   14.0f,
                                                   16.0f,
                                                   18.0f,
                                                   20.0f,
                                                   22.0f,
                                                   24.0f,
                                                   26.0f,
                                                   28.0f,
                                                   30.0f,};




/**
 * @brief ������������Զ���
 *
 */
void Class_FSM_Press_Hold::RTOS_1ms_Calculate_Callback() {
  Status[Now_Status_Serial].Count_Time++;

  // �Լ����ű�д״̬ת�ƺ���
  switch (Now_Status_Serial) {
  case (FSM_Press_Hold_Status_STOP): {
    // ͣ��״̬

    if (Robot->VT03.Get_Mouse_Left_Key() ==
        VTM_RX_Key_Status_TRIG_FREE_PRESSED) {
      // ͣ��->����״̬
      Set_Status(FSM_Press_Hold_Status_PRESSED);
    }

    break;
  }
  case (FSM_Press_Hold_Status_FREE): {
    // �ſ�״̬

    if (Robot->VT03.Get_Mouse_Left_Key() ==
        VTM_RX_Key_Status_TRIG_FREE_PRESSED) {
      // �ɿ�->����״̬
      Set_Status(FSM_Press_Hold_Status_PRESSED);
    }

    break;
  }
  case (FSM_Press_Hold_Status_PRESSED): {
    // ����״̬

    if (Robot->VT03.Get_Mouse_Left_Key() ==
        VTM_RX_Key_Status_TRIG_PRESSED_FREE) {
      // ��ʱ�䳤��->����״̬

      Set_Status(FSM_Press_Hold_Status_CLICK);
    }
    if (Status[Now_Status_Serial].Count_Time >= Hold_Time_Threshold) {
      // ��ʱ�䰴��->����״̬
      Set_Status(FSM_Press_Hold_Status_HOLD);
    }

    break;
  }
  case (FSM_Press_Hold_Status_CLICK): {
    // ����״̬

    Set_Status(FSM_Press_Hold_Status_FREE);

    break;
  }
  case (FSM_Press_Hold_Status_HOLD): {
    // ����״̬
    Hold_Time_Threshold = Default_Hold_Time_Threshold;

    if (Robot->VT03.Get_Mouse_Left_Key() == VTM_RX_Key_Status_FREE) {
      Set_Status(FSM_Press_Hold_Status_FREE);
    }

    break;
  }
  }
}

/**
 * @brief �����Լ���߼�
 *
 */
void Class_FSM_Heat_Detector::RTOS_1ms_Calculate_Callback() {
  Status[Now_Status_Serial].Count_Time++;

  // �Լ����ű�д״̬ת�ƺ���
  switch (Now_Status_Serial) {
  case (FSM_Heat_Detector_Status_CLOSE): {
    // ͣ��״̬

    if ((Robot->Booster.Motor_Friction_Left.Get_Status() ==
             Motor_DJI_Status_ENABLE &&
         Robot->Booster.Motor_Friction_Right.Get_Status() ==
             Motor_DJI_Status_ENABLE) &&
        (Robot->Booster.Motor_Friction_Left.Get_Target_Omega() > 0.0f &&
         Robot->Booster.Motor_Friction_Right.Get_Target_Omega() < 0.0f) &&
        (Robot->Booster.Motor_Friction_Left.Get_Now_Filter_Omega() >=
             Robot->Booster.Motor_Friction_Left.Get_Target_Omega() * 0.99f &&
         Robot->Booster.Motor_Friction_Right.Get_Now_Filter_Omega() <=
             Robot->Booster.Motor_Friction_Right.Get_Target_Omega() * 0.99f)) {
      // Ħ���ִﵽ��Ŀ���ٶ��ҵ�����->����״̬
      Set_Status(FSM_Heat_Detector_Status_OPEN);
    }

    break;
  }
  case (FSM_Heat_Detector_Status_OPEN): {
    // ����״̬

    float now_current = Robot->Booster.Motor_Friction_Left.Get_Now_Current() -
                        Robot->Booster.Motor_Friction_Right.Get_Now_Current();
    Current_Queue_Sum += now_current;
    Current_Queue.Push(now_current);

    // ���㴰���ڵ�����
    if (Current_Queue.Get_Length() > Max_Queue_Size) {
      Current_Queue_Sum -= Current_Queue.Get_Front();
      Current_Queue.Pop();
    }

    // ����������ж���ֵ����Ϊ�Ǵ���ӵ�
    if (Current_Queue_Sum > Current_Queue_Sum_Threshold) {
      Current_Queue.Clear();
      Current_Queue_Sum = 0.0f;
      Now_Heat += 10.0f;
      Total_Ammo_Num++;
    }

    // ������ȴ
    // Now_Heat -= (float) (Robot->Referee.Get_Self_Booster_Heat_CD()) * 0.001f;
    if (Now_Heat < 0.0f) {
      Now_Heat = 0.0f;
    }

    if ((Robot->Booster.Motor_Friction_Left.Get_Status() ==
             Motor_DJI_Status_DISABLE &&
         Robot->Booster.Motor_Friction_Right.Get_Status() ==
             Motor_DJI_Status_DISABLE) ||
        (Robot->Booster.Motor_Friction_Left.Get_Target_Omega() == 0.0f &&
         Robot->Booster.Motor_Friction_Right.Get_Target_Omega() == 0.0f)) {
      // �������->�ػ�״̬

      Current_Queue.Clear();
      Current_Queue_Sum = 0.0f;
      Now_Heat = 0.0f;

      Set_Status(FSM_Heat_Detector_Status_CLOSE);
    }

    break;
  }
  }
}

/**
 * @brief ���ƽ����˳�ʼ��
 *
 */
void CLASS_ROBOT::Init() {
  // δ����״̬, �ɿ�״̬, ����״̬, ����״̬

    //腿部运动学解析验证
    //默认使用MIT模式
    Foot1_Left.SingleFoot_Init(&hfdcan1,0x12,0x02,&hfdcan1,0x11,0x01,Left,kp,kd,angle,Omega,-Torque);
    Foot2_Right.SingleFoot_Init(&hfdcan1,0x14,0x04,&hfdcan1,0x13,0x03,Right,kp,kd,angle,Omega,Torque);

    Chassis.Init();

    //
  // FSM_VT03_Left_Mouse_Press_Hold.Robot = this;
  // FSM_VT03_Left_Mouse_Press_Hold.Init(4, FSM_Press_Hold_Status_STOP);
  //
  // // �����Լ���߼���ʼ��
  // FSM_Heat_Detector.Robot = this;
  // FSM_Heat_Detector.Init(2, FSM_Heat_Detector_Status_CLOSE);
  //
  // // ���̸���PID��ʼ��, ��P����
  // PID_Chassis_Follow.Init(4.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f * PI, 3.0f * PI);
  //
  // // ���̳������ݹ��ʿ���PID��ʼ��
  // PID_Supercap_Chassis_Power.Init(0.10f, 1.5f, 0.0f, 0.0f, 120.0f, 120.0f, 0.1f,
  //                                 0.0f, 0.0f, 0.0f, 45.0f);
  //
  // // ���̲���ϵͳ���ʿ���PID��ʼ��
  // PID_Referee_Chassis_Power.Init(0.05f, 1.5f, 0.0f, 0.0f, 120.0f, 120.0f, 0.1f,
  //                                0.0f, 0.0f, 0.0f, 45.0f);
  //
  // // ����б�º�����ʼ��
  // Slope_Speed_X.Init(0.004f, 0.005f);
  // Slope_Speed_Y.Init(0.004f, 0.005f);
  // Slope_Omega.Init( PI / 1000.0f, PI / 1000.0f);
  //
  // // ���þ�̬ʵ��ָ�벢��ʼ�� PC ͨ��
  // Instance = this;
  //
  // NUC_PC.Init(PC_Receive_Callback);
  //
  // // ң������ʼ��
  // //DR16.Init(&huart5);
  // VT03.Init(&huart1);
  //
  // Debug.Init(&huart10);
  //
  // // ��̨��ʼ��
  // Gimbal.Init(&(NUC_PC.rxPacket));
  //
  // // ���������ʼ��
  // Booster.Init();
  //
  // BMI088.Init(&hspi2);
	 //
  // Gimbal.Motor_Yaw.Bind_IMU(&BMI088);
  // Gimbal.Motor_Pitch.Bind_IMU(&BMI088);
  //
  // The_Chassis.Init(&hfdcan3);
  //
  // // ̧��������ʼ��
  // Lift.Init(this);
  //
  // // ���ģ���ʼ��
  // TOF050F.Init(&huart7, 0x01, TOF050F_Range_Mode_HIGH_PRECISION, 1, 1);
  // TOF050F.Set_Auto_Output(1);
}

/**
 * @brief systick��ʱ���ж϶��ڼ��ģ���Ƿ���
 *
 */
void CLASS_ROBOT::RTOS_1000ms_Alive_PeriodElapsedCallback() {}


/**
 * @brief systick��ʱ���ж϶��ڼ��ģ���Ƿ���
 *
 */
void CLASS_ROBOT::RTOS_100ms_Alive_PeriodElapsedCallback() {

    Chassis.RTOS_100ms_Alive_Callback();
  // VT03.RTOS_100ms_Alive_Callback();
  // The_Chassis.RTOS_100ms_Alive_Callback();
  // Gimbal.RTOS_100ms_Alive_Callback();
  // Booster.RTOS_100ms_Alive_Callback();
  // //��λ��ͨѶ�����ж�
  // NUC_PC.RTOS_100ms_Alive_Callback();
  // Lift.RTOS_100ms_Alive_Callback();
  // TOF050F.TIM_100ms_Alive_PeriodElapsedCallback();
}

/**
 * @brief ���㺯��
 *
 */
void CLASS_ROBOT::RTOS_1ms_Calculate_Callback() {

    Chassis_Control();
    Chassis.RTOS_1ms_Calculate_Callback();

  //
  // // ������������Զ���
  // FSM_VT03_Left_Mouse_Press_Hold.RTOS_1ms_Calculate_Callback();
  //
  // FSM_Heat_Detector.RTOS_1ms_Calculate_Callback();
  //
  //
  // // ң���������������½���
  // VT03.RTOS_1ms_Calculate_Callback();
  //
  // Status_Control();
  //
  // Booster.RTOS_1ms_Calculate_Callback();
  //
  // Gimbal.RTOS_1ms_Calculate_Callback();
  //
  // Lift.RTOS_1ms_Calculate_Callback();
  //
  // //����
  // if(current_control_mode != CONTROL_MODE_DISABLE)
  // {
  //     Gimbal_Control();
  //
  //     Chassis_Control();
  //
  //     Booster_Control();
  //
  //     Lift.Control();
		//
  // }

  // The_Chassis.RTOS_1ms_Calculate_Callback();

}

void CLASS_ROBOT::RTOS_1ms_Conmunicate_Callback() {



  // BMI088.RTOS_IMU_1ms_Sampling_Task();
  // BMI088.Set_EKF_Dt(BMI088_Timer.dt);
  // BMI088_Timer.Get_Dt_Result();
  // BMI088.RTOS_IMU_1ms_EKF_Callback();
  //
  // // IMU 姿态 -> VOFA+ Cube 控件 (100Hz, 每 10ms 发一帧)
  // // Cube 控件绑定: I0=Roll, I1=Pitch, I2=Yaw (欧拉角模式)
  // {
  //   static uint8_t imu_plot_counter = 0;
  //   imu_plot_counter++;
  //   if (imu_plot_counter >= 10) {
  //     imu_plot_counter = 0;
  //     Debug.PlotEuler(BMI088.Get_Roll(),
  //                     BMI088.Get_Pitch(),
  //                     BMI088.Get_Yaw());
  //   }
  // }
  //
  RTOS_1ms_FDCAN_Motor_Callback();
  //
  // TOF050F.TIM_1ms_Poll_PeriodElapsedCallback();


 /*�����ǵ��Լ�������ʱע�͵�*/

 // Debug.Plot(4,Chassis.Motor_Wheel[0].Get_Target_Current(),Chassis.Motor_Wheel[1].Get_Target_Current(),Chassis.Motor_Wheel[2].Get_Target_Current(),Chassis.Motor_Wheel[3].Get_Target_Current()
 //   ,Chassis.Motor_Wheel[0].after_filter_target_current,Chassis.Motor_Wheel[1].after_filter_target_current,Chassis.Motor_Wheel[2].after_filter_target_current,Chassis.Motor_Wheel[3].after_filter_target_current);

 // Debug.Plot(7, Gimbal.Motor_Small_Yaw.Get_Target_Angle(),Gimbal.Motor_Small_Yaw.Get_Now_Angle(),Gimbal.Motor_Small_Yaw.Get_Target_Omega(),Gimbal.Motor_Small_Yaw.Get_Now_Omega(),Gimbal.Motor_Small_Yaw.Get_Real_Omega(),Gimbal.Motor_Small_Yaw.Get_Out(),Gimbal.Motor_Small_Yaw.Get_Now_Current()*16384.0f / 3.0f);
 // Debug.Plot(10, Gimbal.Gyro_Motor_Omega.Get_Omega_X(),Gimbal.Gyro_Motor_Omega.Get_Omega_Y(),Gimbal.Gyro_Motor_Omega.Get_Omega_Z(),Gimbal.Gyro_Motor_Omega.Get_Filter_Omega_X(),Gimbal.Gyro_Motor_Omega.Get_Filter_Omega_Y(),
 // Gimbal.Gyro_Motor_Omega.Get_Filter_Omega_Z(), Gimbal.Gyro_Motor_Omega.Get_Kalman_Estimate_Error(),Gimbal.Gyro_Motor_Omega.Get_Kalman_Process_Noise(),
 // Gimbal.Gyro_Motor_Omega.Get_Kalman_Measure_Noise(), Gimbal.Gyro_Motor_Omega.Get_Kalman_Gain());
 // ��̨pitch()
 // Debug.Plot(5,Gimbal.Motor_Pitch.Get_Target_Angle(), Gimbal.Motor_Pitch.Get_Now_Angle(),Gimbal.Motor_Pitch.Get_Control_Omega(),Gimbal.Motor_Pitch.Get_Now_Omega(),Gimbal.Motor_Pitch.Get_Real_Omega());
 // Debug.Plot(3,Booster.Motor_Friction_Left.Get_Target_Omega(), Booster.Motor_Friction_Left.Get_Now_Omega(),Booster.Motor_Friction_Left.after_filter_Omega);
 // Debug.Plot(4, Lift.Motor_Lift.Get_Target_Omega(), Lift.Motor_Lift.Get_Now_Omega(), Lift.Motor_Lift.after_filter_Omega, (float)TOF050F.Get_Distance_MM() / 10.0f);
 // Debug.Plot(3, Lift.Motor_Lift.Get_Target_Angle(), Lift.Motor_Lift.Get_Now_Angle(), (float)TOF050F.Get_Distance_MM() / 10.0f);
 // Debug.Plot(6, Gimbal.Motor_Yaw.Get_Target_Angle(),Gimbal.Motor_Yaw.Get_Now_Angle(),Gimbal.Motor_Yaw.Get_Control_Omega(),Gimbal.Motor_Yaw.Get_Now_Omega(),Gimbal.Motor_Yaw.Get_Real_Omega(),BMI088_Timer.dt);
 // Debug.Plot(4, Booster.Motor_Driver.Get_Target_Omega(),Booster.Motor_Driver.Get_Now_Omega(),Booster.Motor_Driver.after_filter_Omega,Booster.Motor_Driver.Get_Target_Angle(),Booster.Motor_Driver.Get_Now_Angle());
 // Debug.Plot(3, Booster.Motor_Friction_Left.Get_Target_Omega(), Booster.Motor_Friction_Left.Get_Now_Omega(), Booster.Motor_Friction_Left.after_filter_Omega);
 // Debug.Plot(4, The_Chassis.Data.Chassis_Omega,Gimbal.Motor_Yaw.Get_Now_Omega());
 // Debug.Plot(3, Gimbal.Motor_Yaw.Get_Now_Angle(),Gimbal.Motor_Yaw.Get_Real_Angle());
 // Debug.Plot(2, BMI088.Get_AccelNorm(), 9.807f / BMI088.Get_AccelNorm());
 //
 //
 //    static uint8_t usb_send_counter = 0;
 //    usb_send_counter++;
 //    if (usb_send_counter >= 10) {
 //        usb_send_counter = 0;
 //        if (NUC_PC.IsConnected()) {
 //            PC_SendPacket my_packet;
 //            my_packet.head[0] = 'S';
 //            my_packet.head[1] = 'P';
 //            my_packet.mode = 1;
 //            const float* imu_q = BMI088.Get_q();
 //            my_packet.q[0] = imu_q[0];
 //            my_packet.q[1] = imu_q[1];
 //            my_packet.q[2] = imu_q[2];
 //            my_packet.q[3] = imu_q[3];
 //            my_packet.yaw = Gimbal.Motor_Yaw.Get_Now_Angle();
 //            my_packet.yaw_vel = 0.0f;
 //            my_packet.pitch = Gimbal.Motor_Pitch.Get_Now_Angle();
 //            my_packet.pitch_vel = 0.0f;
 //            my_packet.bullet_speed = 23.0f;
 //            // my_packet.bullet_speed = (float)Slave_Referee.UART_To_CAN_0x1F1.Ammo_Speed;
 //            my_packet.bullet_count++;
 //            NUC_PC.Send(my_packet);
    //     }
    // }




    //�����ǵ���
		// const uint8_t num = 22;

    // float packet[num];
    // packet[0] = BMI088.Get_Gyro_X();
    // packet[1] = BMI088.Get_Gyro_Y();
    // packet[2] = BMI088.Get_Gyro_Z();
    // packet[3] = BMI088.Get_Accel_X();
    // packet[4] = BMI088.Get_Accel_Y();
    // packet[5] = BMI088.Get_Accel_Z();
    // packet[6] = BMI088.Get_Roll();
    // packet[7] = BMI088.Get_Pitch();
    // packet[8] = BMI088.Get_Yaw();
    // packet[9] = BMI088.Get_YawTotalAngle();
    // packet[10] = BMI088.Get_dt();
    // packet[11] = BMI088.Get_q()[0];
    // packet[12] = BMI088.Get_q()[1];
    // packet[13] = BMI088.Get_q()[2];
    // packet[14] = BMI088.Get_q()[3];
    // packet[15] = BMI088.Get_GyroOffset_X();
    // packet[16] = BMI088.Get_GyroOffset_Y();
    // packet[17] = BMI088.Get_GyroOffset_Z();
    // packet[18] = BMI088.Get_IsStationary() ? 1.0f : 0.0f;
    // packet[19] = BMI088.Get_GyroNorm();
    // packet[20] = BMI088.Get_AccelNorm();
    // packet[21] = BMI088.Get_AccelError();

    // uint8_t Data[num*4+4];
    // memcpy(Data, packet, sizeof(packet));
    // Data[num*4] = 0x00;
    // Data[num*4+1] = 0x00;
    // Data[num*4+2] = 0x80;
    // Data[num*4+3] = 0x7F;
    // USB_VCP_Send((uint8_t*)Data, sizeof(Data));


}

/**
 * @brief ���̿����߼�������1ms
 * @note ����̨����ϵĿ���ٶ���ת�任����������ϵ��д��ͨ�����ݰ�
 */
void CLASS_ROBOT::Chassis_Control() {



    FSi6x_control_polar();
    Foot1_Left.Foot_control();

    Foot2_Right.Foot_control();


    // float gimbal_yaw = Gimbal.Motor_Yaw.Get_Real_Angle();
    // float delta_angle = Math_Modulus_Normalization(-gimbal_yaw, 2.0f*PI);
    // float sin_yaw = arm_sin_f32(delta_angle);
    // float cos_yaw = arm_cos_f32(delta_angle);
    //
    // float gimbal_vx = Gimbal.Target_Velocity_X;
    // float gimbal_vy = Gimbal.Target_Velocity_Y;
    // float gimbal_omega = Gimbal.Target_Omega;
    //
    // float target_vx = (gimbal_vx * cos_yaw - gimbal_vy * sin_yaw);
    // float target_vy = gimbal_vx * sin_yaw + gimbal_vy * cos_yaw;
    // float target_omega = gimbal_omega;
    //
    // memcpy((void *)&The_Chassis.SendPacket.target_v_x, &target_vx, 4);
    // memcpy((void *)&The_Chassis.SendPacket.target_v_y, &target_vy, 4);
    // memcpy((void *)&The_Chassis.SendPacket.target_v_omega, &target_omega, 4);

}

/**
 * @brief ��̨�����߼�
 * @note ���ݿ���ģʽѡ��ң������PC���ƣ���ֹ���ݾ���
 */
void CLASS_ROBOT::Gimbal_Control() 
{
    if (VT03.Get_Status() == VTM_RX_Status_DISABLE)
    {

        return;
    }
    else
    {
        // ��̨ǰ��ֵ
        float tmp_gimbal_yaw_feedforward_omega = 0.0f;
        float tmp_gimbal_yaw_feedforward_current = 0.0f;
        float tmp_gimbal_pitch_feedforward_omega = 0.0f;
        float tmp_gimbal_pitch_feedforward_current = 0.0f;

    // ����ң����ģʽ�´���ң��������
    if (current_control_mode == CONTROL_MODE_REMOTE) 
    {
        // ң����ҡ��ֵ
        float VT03_left_x = VT03.Get_Left_X();
        float VT03_left_y = VT03.Get_Left_Y();
        float VT03_right_x = VT03.Get_Right_X();
        float VT03_right_y = VT03.Get_Right_Y();

        // ҡ����������
        VT03_left_x = (fabsf(VT03_left_x) > 0.01f) ? VT03_left_x : 0.0f;
        VT03_left_y = (fabsf(VT03_left_y) > 0.01f) ? VT03_left_y : 0.0f;

        // ң�������Ƶ����ٶ�
        Gimbal.Target_Velocity_X = VT03_left_y * 4.0f;
        Gimbal.Target_Velocity_Y = -VT03_left_x * 4.0f;
        
        if (!Auto_Aim_Enable)
        {
            if (Lift.FSM_Lift.Get_Now_Status_Serial() == FSM_Lift_Status_DOWN) {
                Gimbal.Target_Omega = -VT03_right_x * 4.0f;
            } else {
                Gimbal.Target_Yaw_Angle -= VT03_right_x * 0.005f;
            }
            Gimbal.Target_Pitch_Angle = -VT03_right_y * 0.44f;
        }
        else
        {
            // ����ģʽ����̨��PC���������ݿ���
            float target_yaw = Gimbal.Motor_Yaw.Get_Now_Angle() + NUC_PC.rxPacket.yaw;
            float target_pitch = Gimbal.Motor_Pitch.Get_Now_Angle()
                                   + NUC_PC.rxPacket.pitch * 1.1f;
            if (NUC_PC.rxPacket.yaw != 0.0f) 
            {
                target_yaw += AIM_Yaw;
                target_pitch += AIM_Pitch;
            }

            Gimbal.Target_Yaw_Angle = target_yaw;
            Gimbal.Target_Pitch_Angle = target_pitch;

            // �ٶ�ǰ����������
            float raw_yaw_vel = NUC_PC.rxPacket.yaw_vel;
            float raw_pitch_vel = NUC_PC.rxPacket.pitch_vel;
            if (fabsf(raw_yaw_vel) < Yaw_Vel_DeadZone) raw_yaw_vel = 0.0f;
            if (fabsf(raw_pitch_vel) < Pitch_Vel_DeadZone) raw_pitch_vel = 0.0f;

            // �ٶ�ǰ����ͨ�˲�
            Filtered_Yaw_Vel = Yaw_Vel_Filter_Coeff * raw_yaw_vel + (1.0f - Yaw_Vel_Filter_Coeff) * Filtered_Yaw_Vel;
            Filtered_Pitch_Vel = Pitch_Vel_Filter_Coeff * raw_pitch_vel + (1.0f - Pitch_Vel_Filter_Coeff) * Filtered_Pitch_Vel;

            tmp_gimbal_yaw_feedforward_omega += Filtered_Yaw_Vel * 0.0f;
            tmp_gimbal_pitch_feedforward_omega += Filtered_Pitch_Vel * 0.0f;
        }
    }
    // �������ģʽ
    else if (current_control_mode == CONTROL_MODE_KEYBOARD)
    {
      if (VT03.Get_Keyboard_Key_W() == VTM_RX_Key_Status_PRESSED)         
      {
          Gimbal.Target_Velocity_X += 1.0f;
      }
      if (VT03.Get_Keyboard_Key_S() == VTM_RX_Key_Status_PRESSED)
      {
          Gimbal.Target_Velocity_X -= 1.0f;
      }
      if (VT03.Get_Keyboard_Key_A() == VTM_RX_Key_Status_PRESSED)
      {
          Gimbal.Target_Velocity_Y += 1.0f;
      }
      if (VT03.Get_Keyboard_Key_D() == VTM_RX_Key_Status_PRESSED)
      {
          Gimbal.Target_Velocity_Y -= 1.0f;
      }
      Math_Constrain(&Gimbal.Target_Velocity_X, -4.0f, 4.0f);
      Math_Constrain(&Gimbal.Target_Velocity_Y, -4.0f, 4.0f);
      if(VT03.Get_Keyboard_Key_W() == VTM_RX_Key_Status_FREE && VT03.Get_Keyboard_Key_S() == VTM_RX_Key_Status_FREE)
      {
        Gimbal.Target_Velocity_X = 0.0f;
      }
      if(VT03.Get_Keyboard_Key_A() == VTM_RX_Key_Status_FREE && VT03.Get_Keyboard_Key_D() == VTM_RX_Key_Status_FREE)
      {
        Gimbal.Target_Velocity_Y = 0.0f;
      }
      
      // ������ģʽ
      if (!Auto_Aim_Enable)
      {
          if (Lift.FSM_Lift.Get_Now_Status_Serial() == FSM_Lift_Status_DOWN) {
              Gimbal.Target_Omega = -VT03.Get_Mouse_X() * 0.1f;
          } else {
              Gimbal.Target_Yaw_Angle -= VT03.Get_Mouse_X() * 0.4f;
          }
          Gimbal.Target_Pitch_Angle -= VT03.Get_Mouse_Y() * 0.3f;
      }
      // ����ģʽ
      else
      {
            float target_yaw = Gimbal.Motor_Yaw.Get_Now_Angle() + NUC_PC.rxPacket.yaw;
            float target_pitch = Gimbal.Motor_Pitch.Get_Now_Angle()
                                   + NUC_PC.rxPacket.pitch * 1.1f;
            if (NUC_PC.rxPacket.yaw != 0.0f) {
                target_yaw += AIM_Yaw;
                target_pitch += AIM_Pitch;
            }

            Gimbal.Target_Yaw_Angle = target_yaw;
            Gimbal.Target_Pitch_Angle = target_pitch;

        // �ٶ�ǰ����������
        float raw_yaw_vel = NUC_PC.rxPacket.yaw_vel;
        float raw_pitch_vel = NUC_PC.rxPacket.pitch_vel;
        if (fabsf(raw_yaw_vel) < Yaw_Vel_DeadZone) raw_yaw_vel = 0.0f;
        if (fabsf(raw_pitch_vel) < Pitch_Vel_DeadZone) raw_pitch_vel = 0.0f;

        // �ٶ�ǰ����ͨ�˲�
        Filtered_Yaw_Vel = Yaw_Vel_Filter_Coeff * raw_yaw_vel + (1.0f - Yaw_Vel_Filter_Coeff) * Filtered_Yaw_Vel;
        Filtered_Pitch_Vel = Pitch_Vel_Filter_Coeff * raw_pitch_vel + (1.0f - Pitch_Vel_Filter_Coeff) * Filtered_Pitch_Vel;

        tmp_gimbal_yaw_feedforward_omega += Filtered_Yaw_Vel * 0.0f;
        tmp_gimbal_pitch_feedforward_omega += Filtered_Pitch_Vel * 0.0f;
//        tmp_gimbal_yaw_feedforward_omega += Filtered_Yaw_Vel * 0.05f;
//        tmp_gimbal_pitch_feedforward_omega += Filtered_Pitch_Vel * 0.5f;
      }
  }

  // ������ת����
  tmp_gimbal_yaw_feedforward_omega = The_Chassis.Data.Chassis_Omega * K_Gyro_Omega;


  // ������̨����ϵ���ٶ�
  if (Chassis_Gyroscope_Mode_Status == Robot_Gyroscope_Type_CLOCKWISE && Chassis_Follow_Mode_Status == false)
  {
      Gimbal.Target_Omega = 3*PI;
  }
  else if (Chassis_Gyroscope_Mode_Status == Robot_Gyroscope_Type_COUNTERCLOCKWISE && Chassis_Follow_Mode_Status == false)
  {
      Gimbal.Target_Omega = -3*PI;
  }
  else if(Chassis_Gyroscope_Mode_Status == Robot_Gyroscope_Type_DISABLE && Chassis_Follow_Mode_Status == true)
  {   
      PID_Chassis_Follow.Set_Target(0.0f);
      float gimbal_yaw = Gimbal.Motor_Yaw.Get_Real_Angle();
      float delta_angle = Math_Modulus_Normalization(-gimbal_yaw, 2.0f*PI);
      PID_Chassis_Follow.Set_Now(-delta_angle);
      PID_Chassis_Follow.PID_Process();
      Gimbal.Target_Omega = PID_Chassis_Follow.Get_Out();
  }


  //������������λ
  Math_Constrain(&Gimbal.Target_Pitch_Angle, -28.0f * DEG_TO_RAD, 15.0f * DEG_TO_RAD);
  
  Gimbal.Motor_Yaw.Set_Control_FeedForward_Omega(tmp_gimbal_yaw_feedforward_omega);
  Gimbal.Motor_Yaw.Set_Control_FeedForward_Current(tmp_gimbal_yaw_feedforward_current);
  Gimbal.Motor_Pitch.Set_Control_FeedForward_Current(tmp_gimbal_pitch_feedforward_current);
  Gimbal.Motor_Pitch.Set_Control_FeedForward_Omega(tmp_gimbal_pitch_feedforward_omega);
  }
  
}

/**
 * @brief ������������߼�
 *
 */
/**
 * @brief �������ģʽ���ƺ���
 * @details �ú������ݵ�ǰ����Դѡ�����������ģʽ����ͬ������ϵͳ����������
 *          ��ң����ģʽ��ʹ��ң����ͨ����������Ƶ�ʣ��ڼ���ģʽ��ʹ��������ʵ�ֵ�����������
 *          �� PC ģʽ��ʹ����λ���·��� fire_flag ����ͣ����������ȷ�����ֿ���ģʽ���������
 * @note �� DR16 ����ʱ�����������ֱ�ӽ���ʧ��״̬���Ա�֤ϵͳ��ȫ��
 */
void CLASS_ROBOT::Booster_Control() {
   // �����Լ��״̬��
    FSM_Heat_Detector.RTOS_1ms_Calculate_Callback();

    // ���ݲ���ϵͳ(����)�Ƿ���������ž�������
    if (The_Chassis.Get_Status() == false)
    {
        // ����ϵͳ���߻�Զ����Ϣ������

        if (Robot_Booster_Type == Robot_Booster_Type_BURST)
        {
            Booster.Set_Heat_Limit_Max(booster_burst_first_heat_max[Robot_Level - 1]);
            Booster.Set_Heat_CD(booster_burst_first_heat_cd[Robot_Level - 1]);
        }
        else if (Robot_Booster_Type == Robot_Booster_Type_CD)
        {
            Booster.Set_Heat_Limit_Max(booster_cd_first_heat_max[Robot_Level - 1]);
            Booster.Set_Heat_CD(booster_cd_first_heat_cd[Robot_Level - 1]);
        }
        Booster.Set_Now_Heat(FSM_Heat_Detector.Get_Now_Heat());
    }
    else
    {
        // ����ϵͳ����
        Booster.Set_Heat_Limit_Max(The_Chassis.Data.Self_Booster_Heat_Max);
        Booster.Set_Heat_CD(The_Chassis.Data.Self_Booster_Heat_CD);
        Booster.Set_Now_Heat(The_Chassis.Data.Booster_17mm_1_Heat);
        Robot_Level = The_Chassis.ReceivePacket.Robot_Level;
    }

    // �ж�ң����״̬�Ƿ�����, �����߻�ֱͣ�ӶϿ�
    if (VT03.Get_Status() == VTM_RX_Status_DISABLE || current_control_mode == CONTROL_MODE_DISABLE)
    {
        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
        return;
    }
    else
    {
        if (Booster_Disable_Flag)
        {
            Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            return;
        }
        // ң��ģʽ�����ģʽ�µķ����߼�����
        if (Auto_Aim_Enable == false)
        {
            bool v_key = (current_control_mode == CONTROL_MODE_KEYBOARD &&
                         (VT03.Get_Keyboard_Key_V() == VTM_RX_Key_Status_PRESSED ||
                          VT03.Get_Keyboard_Key_V() == VTM_RX_Key_Status_TRIG_FREE_PRESSED));

            bool trigger_rising = (current_control_mode == CONTROL_MODE_REMOTE &&
                                   VT03.Get_Trigger_Key() == VTM_RX_Key_Status_TRIG_FREE_PRESSED);
            bool trigger_held   = (current_control_mode == CONTROL_MODE_REMOTE &&
                                   VT03.Get_Trigger_Key() == VTM_RX_Key_Status_PRESSED);
            bool mouse_click    = (FSM_VT03_Left_Mouse_Press_Hold.Get_Now_Status_Serial() == FSM_Press_Hold_Status_CLICK);
            bool mouse_hold     = (FSM_VT03_Left_Mouse_Press_Hold.Get_Now_Status_Serial() == FSM_Press_Hold_Status_HOLD);

            if (v_key)
            {
                Booster.Set_Driver_Direction(-1);
                Booster.Set_Booster_Control_Type(Booster_Control_Type_AUTO);

                if (The_Chassis.Get_Status() == true)
                {
                    float tmp_frequency = The_Chassis.Data.Self_Booster_Heat_Max / 10.0f;
                    Math_Constrain(&tmp_frequency, 5.0f, 20.0f);
                    Booster.Set_Max_Ammo_Shoot_Frequency(tmp_frequency);
                }
                else
                {
                    Booster.Set_Max_Ammo_Shoot_Frequency(15.0f);
                }
            }
            else if (trigger_rising || mouse_click)
            {
                // �����߼������������ �� ���̰�
                Booster.Set_Driver_Direction(1);
                Booster.Set_Booster_Control_Type(Booster_Control_Type_SPOT);
            }
            else if (trigger_held || mouse_hold)
            {
                // �����߼��������ס �� ��곤��
                Booster.Set_Driver_Direction(1);
                Booster.Set_Booster_Control_Type(Booster_Control_Type_AUTO);

                if (The_Chassis.Get_Status() == true)
                {
                    float tmp_frequency = The_Chassis.Data.Self_Booster_Heat_Max / 10.0f;
                    Math_Constrain(&tmp_frequency, 5.0f, 20.0f);
                    Booster.Set_Max_Ammo_Shoot_Frequency(tmp_frequency);
                }
                else
                {
                    Booster.Set_Max_Ammo_Shoot_Frequency(15.0f);
                }
            }
            else
            {
                // ͣ��
                Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            }
        }
        // ����ģʽ
//        else
//        {
//            if (current_control_mode == CONTROL_MODE_KEYBOARD)
//            {
//                bool mouse_pressed = (FSM_VT03_Left_Mouse_Press_Hold.Get_Now_Status_Serial() == FSM_Press_Hold_Status_PRESSED ||
//                                      FSM_VT03_Left_Mouse_Press_Hold.Get_Now_Status_Serial() == FSM_Press_Hold_Status_HOLD);

//                if (mouse_pressed)
//                {
//                    Booster.Set_Driver_Direction(1);
//                    Booster.Set_Booster_Control_Type(Booster_Control_Type_AUTO);

//                    if (The_Chassis.Get_Status() == true)
//                    {
//                        float tmp_frequency = The_Chassis.Data.Self_Booster_Heat_Max / 10.0f;
//                        Math_Constrain(&tmp_frequency, 5.0f, 20.0f);
//                        Booster.Set_Max_Ammo_Shoot_Frequency(tmp_frequency);
//                    }
//                    else
//                    {
//                        Booster.Set_Max_Ammo_Shoot_Frequency(15.0f);
//                    }
//                }
//                else
//                {
//                    Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
//                }
//            }
//            else if (NUC_PC.rxPacket.mode == 2)
//            {
//                // ���Զ���PC ����Ŀ�꣬�����ְ�ס���/���ſ���
//                bool trigger_rising = (VT03.Get_Trigger_Key() == VTM_RX_Key_Status_TRIG_FREE_PRESSED);
//                bool trigger_held   = (VT03.Get_Trigger_Key() == VTM_RX_Key_Status_PRESSED);
//                bool mouse_click    = (FSM_VT03_Left_Mouse_Press_Hold.Get_Now_Status_Serial() == FSM_Press_Hold_Status_CLICK);
//                bool mouse_hold     = (FSM_VT03_Left_Mouse_Press_Hold.Get_Now_Status_Serial() == FSM_Press_Hold_Status_HOLD);
//                
//                if (trigger_held || mouse_hold)
//                {
//                    Booster.Set_Booster_Control_Type(Booster_Control_Type_AUTO);

//                    if (The_Chassis.Get_Status() == true)
//                    {
//                        float tmp_frequency = The_Chassis.Data.Self_Booster_Heat_Max / 10.0f;
//                        Math_Constrain(&tmp_frequency, 5.0f, 20.0f);
//                        Booster.Set_Max_Ammo_Shoot_Frequency(tmp_frequency);
//                    }
//                    else
//                    {
//                        Booster.Set_Max_Ammo_Shoot_Frequency(15.0f);
//                    }
//                }
//                else if (trigger_rising || mouse_click)
//                {
//                    Booster.Set_Booster_Control_Type(Booster_Control_Type_SPOT);
//                }
//                else
//                {
//                    Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
//                }
//            }
//            else
//            {
//                // PC ��Ŀ�꣬ͣ��
//                Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
//            }
//        }
    }
}

/**
 * @brief ����״̬����,����Ϊ��ʱ�
 *
 */
void CLASS_ROBOT::Status_Control()
{
  // ����ͼ����·��λ�����жϿ�����Դ��N��Ϊң��������ģʽ��S��Ϊ�������ģʽ
  // ͼ����·��λֵ: C=0, N=3, S=6 (��������״̬)
  if (VT03.Get_Status() == VTM_RX_Status_ENABLE)
  {
    if (VT03.Get_Mode_Switch() == VTM_RX_Mode_Switch_N)
    {
      // N����ң��������ģʽ
      current_control_mode = CONTROL_MODE_REMOTE;
    }
    else if (VT03.Get_Mode_Switch() == VTM_RX_Mode_Switch_S)
    {
      // S�����������ģʽ
      current_control_mode = CONTROL_MODE_KEYBOARD;
    }
    else
    {
      // C��������:����ģʽ
      current_control_mode = CONTROL_MODE_DISABLE;
    }
  }
  else
  {
    // ͼ�����ߣ�ʹ��DR16
    current_control_mode = CONTROL_MODE_DISABLE;
  }

  // ����ģʽ����
  if (NUC_PC.Get_Status() == PC_Status_ENABLE)
  {
    if (current_control_mode == CONTROL_MODE_KEYBOARD)
    {
      if (VT03.Get_Mouse_Right_Key() == VTM_RX_Key_Status_PRESSED ||
          VT03.Get_Mouse_Right_Key() == VTM_RX_Key_Status_TRIG_FREE_PRESSED)
      {
        Auto_Aim_Enable = true;
      }
      else
      {
        Auto_Aim_Enable = false;
      }
    }
    else if (current_control_mode == CONTROL_MODE_REMOTE)
    {
      if (VT03.Get_Pause_Key() == VTM_RX_Key_Status_TRIG_FREE_PRESSED)
      {
        Auto_Aim_Enable = !Auto_Aim_Enable;
      }
    }
  }


    float tof_distance_cm = TOF050F.Get_Distance_MM() / 10.0f;

    // ͼ��N��ģʽ��ң�������
    if (current_control_mode == CONTROL_MODE_REMOTE)
    {
      if ((tof_distance_cm <= 12.0f || tof_distance_cm > 80.0f) && Chassis_Gyroscope_Mode_Status != Robot_Gyroscope_Type_DISABLE)
      {
        Chassis_Gyroscope_Mode_Status = Robot_Gyroscope_Type_DISABLE;
        Chassis_Follow_Mode_Status = true;
      }
      if (VT03.Get_Fn_2_Key() == VTM_RX_Key_Status_TRIG_FREE_PRESSED && 
      Chassis_Gyroscope_Mode_Status == Robot_Gyroscope_Type_DISABLE && tof_distance_cm > 12.0f && tof_distance_cm <= 80.0f)
      {
        Chassis_Gyroscope_Mode_Status = Robot_Gyroscope_Type_CLOCKWISE;
        Chassis_Follow_Mode_Status = false;
				The_Chassis.SendPacket.is_gyro = 0x01;
      }
      else if(VT03.Get_Fn_2_Key() == VTM_RX_Key_Status_TRIG_FREE_PRESSED && 
      Chassis_Gyroscope_Mode_Status == Robot_Gyroscope_Type_CLOCKWISE)
      {
        Chassis_Gyroscope_Mode_Status = Robot_Gyroscope_Type_COUNTERCLOCKWISE;
        Chassis_Follow_Mode_Status = false;
		The_Chassis.SendPacket.is_gyro = 0x01;
      }
      else if(VT03.Get_Fn_2_Key() == VTM_RX_Key_Status_TRIG_FREE_PRESSED && 
      Chassis_Gyroscope_Mode_Status == Robot_Gyroscope_Type_COUNTERCLOCKWISE)
      {
        Chassis_Gyroscope_Mode_Status = Robot_Gyroscope_Type_DISABLE;
        Chassis_Follow_Mode_Status = true;
      }
    }
    // ͼ��S��ģʽ��������- С�����ɰ�������
    else if (current_control_mode == CONTROL_MODE_KEYBOARD)
    {
      if ((tof_distance_cm <= 12.0f || tof_distance_cm > 80.0f) && Chassis_Gyroscope_Mode_Status != Robot_Gyroscope_Type_DISABLE)
      {
        Chassis_Gyroscope_Mode_Status = Robot_Gyroscope_Type_DISABLE;
        Chassis_Follow_Mode_Status = true;
      }
      if (VT03.Get_Keyboard_Key_SHIFT() == VTM_RX_Key_Status_TRIG_FREE_PRESSED && 
      Chassis_Gyroscope_Mode_Status == Robot_Gyroscope_Type_DISABLE && tof_distance_cm > 12.0f && tof_distance_cm <= 80.0f)
      {
        Chassis_Gyroscope_Mode_Status = Robot_Gyroscope_Type_CLOCKWISE;
        Chassis_Follow_Mode_Status = false;
		The_Chassis.SendPacket.is_gyro = 0x01;
      }
      else if (VT03.Get_Keyboard_Key_SHIFT() == VTM_RX_Key_Status_TRIG_FREE_PRESSED && 
      Chassis_Gyroscope_Mode_Status == Robot_Gyroscope_Type_CLOCKWISE)
      {
        Chassis_Gyroscope_Mode_Status = Robot_Gyroscope_Type_DISABLE;
        Chassis_Follow_Mode_Status = true;
      }
    }
    // ͼ�����߻�C��ģʽ���������- С���ݹر�
    else
    {
      Chassis_Gyroscope_Mode_Status = Robot_Gyroscope_Type_DISABLE;
    }

    // �������ʹ��״̬��Fn1 / Z �½����л����� N/S ����Ч��
    if (current_control_mode == CONTROL_MODE_REMOTE || current_control_mode == CONTROL_MODE_KEYBOARD)
    {
        if (VT03.Get_Fn_1_Key() == VTM_RX_Key_Status_TRIG_PRESSED_FREE ||
            VT03.Get_Keyboard_Key_Z() == VTM_RX_Key_Status_TRIG_PRESSED_FREE)
        {
            Booster_Disable_Flag = !Booster_Disable_Flag;
        }
    }
    if (Booster_Disable_Flag)
    {
        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
    }
}


/**
 * @brief PC �������ݻص���������̬��
 * @param packet: ���յ������ݰ�
 * @note ͨ����̬ʵ��ָ��������Ա
 */
void CLASS_ROBOT::PC_Receive_Callback(const PC_ReceivePacket &packet) {
  if (Instance == NULL) return;

   // ����ģʽ������ģʽ�½���PC���ݣ�����ֱ�ӷ��ط�ֹ������Ⱦ
    if (Instance->current_control_mode != CONTROL_MODE_KEYBOARD && !Instance->Auto_Aim_Enable) {
      return;
    }

    Instance->NUC_PC.rxPacket = packet;
    
    if(Instance->current_control_mode == CONTROL_MODE_KEYBOARD || Instance->Auto_Aim_Enable)
    {
    if (!isfinite(Instance->NUC_PC.rxPacket.pitch) || !isfinite(Instance->NUC_PC.rxPacket.yaw) ||
        !isfinite(Instance->NUC_PC.rxPacket.yaw_vel) || !isfinite(Instance->NUC_PC.rxPacket.pitch_vel)) {
      return;
    }

    const float MAX_PITCH_ANGLE = 0.45f;

    Instance->NUC_PC.rxPacket.pitch = fmaxf(-MAX_PITCH_ANGLE, fminf(Instance->NUC_PC.rxPacket.pitch, MAX_PITCH_ANGLE));

    const float MAX_OMEGA = 3.0f;

    Instance->NUC_PC.rxPacket.yaw_vel = fmaxf(-MAX_OMEGA, fminf(Instance->NUC_PC.rxPacket.yaw_vel, MAX_OMEGA));
    Instance->NUC_PC.rxPacket.pitch_vel = fmaxf(-MAX_OMEGA, fminf(Instance->NUC_PC.rxPacket.pitch_vel, MAX_OMEGA));
  }
}

/**
 * @brief ��������Ų����̵߳�Hardfault��MSP��ջ���׼��һЩ
 */
void CLASS_ROBOT::Main_Loop_Things() {


  // 1ms�����ж�

  if (robot_1ms_cycle_flag) {
    // ��־λ��λ
    robot_1ms_cycle_flag = false;
    // 1ms���ڴ�������
		RTOS_1ms_Conmunicate_Callback();
		RTOS_1ms_Calculate_Callback();
  }
  // 100ms�����ж�
  if (robot_100ms_cycle_flag) {
    robot_100ms_cycle_flag = false;
    RTOS_100ms_Alive_PeriodElapsedCallback();
  }
  // 1000ms�����ж�
  if (robot_1000ms_cycle_flag) {
    robot_1000ms_cycle_flag = false;
    RTOS_1000ms_Alive_PeriodElapsedCallback();
  }
}



/**
 * @brief 极坐标系下的四足步态控制（当前主用模式）
 *
 * 核心思想：
 * 遥控器映射极坐标参数(R, theta_polar)，转换到直角坐标系(X, Y)后，
 * 叠加相位角theta实现四足步行。
 *
 * 极坐标参数（来自遥控器）：
 * - ch[2] → R（极径，米）：映射1000~2000 → 0.060~0.19m，控制机身距离原点半径
 * - ch[3] → theta_polar（极角，弧度）：映射1000~2000 → 3.925~5.495rad
 *   该范围对应225°~315°（第三象限下行方向），使机身自然位于腿下方
 * - ch[5] (SWC): 前推(>1751)/后拉(<1500)模式开关
 * - ch[4] (SWD): 1000=正常模式, 2000=高度微调模式
 *
 * 转换公式：X = cos(theta_polar) * R, Y = sin(theta_polar) * R
 * 正常直立时theta_polar≈4.71rad(270°), Y≈-R, X≈0
 *
 * 步行叠加：在(X,Y)基础上用theta控制四足相位偏移
 * - 前后腿组相位差Pi实现交替迈步
 * - fine_FB：前后步幅微调
 * - fine_RL：左右转向微调
 * - fine_R_L_direction：左右侧偏置，实现原地旋转
 * - fine_R_hight/fine_L_hight：左右侧腿足端高度独立微调
 *
 * 跳跃状态机集成（见下半部分switch-case）：
 * 当flag_jump==1时进入跳跃状态机：
 *   状态0：起跳（高刚度锁紧，R=0.17）
 *   状态1：收腿（低刚度缩回，R=0.06）
 *   状态2：落地缓冲（低刚度缓冲，R=0.085）
 *   状态3：空闲（直接跳回状态0）
 */

float theta_polar;
float X;
float Y;
float Xe1;
float Ye1;
float Xe2;
float Ye2;
float R;

float Omega1 = 0.0f;

float f1_out_angle;
float f1_in_angle;
float f2_out_angle;
float f2_in_angle;

void CLASS_ROBOT ::FSi6x_control_polar()//极坐标系
{

    if (fsia6b_msg.ch[5] >= 1751  )
    {

        theta_polar = Math_Int_To_Float(fsia6b_msg.ch[3],1000,2000,3.925f,5.495f);
        R = Math_Int_To_Float(fsia6b_msg.ch[2],2000,1000,0.125f,0.450f);
        //theta_polar = 4.4f;
    }
    // else if (fsia6b_msg.ch[4] >= 1751  )
    // {
    //     theta_polar = 4.71f;   // 270°,正下方
    //     R           = 0.15f;   // 已知腿长 15cm
    //
    // }

    if (fsia6b_msg.ch[4] >= 1751  )
    {
        Omega1 = Math_Int_To_Float(fsia6b_msg.ch[1],1000,2000,-100.0f,+100.0f);

    }



    /** @brief 极坐标→直角坐标转换：由极径R和极角theta_polar计算机身参考位姿(X,Y) */
    //极坐标转换成直角坐标
    X = cos(theta_polar) * R;
    Y = sin(theta_polar) * R;

    Xe1 = X;
    Ye1 = Y;
    Xe2 = X;
    Ye2 = Y;

    Foot1_Left.Set_x_y(Xe1,Ye1);
    Foot2_Right.Set_x_y(Xe2,Ye2);

    Chassis.Set_Chassis_Omega(Omega1);

    f1_in_angle = Foot1_Left.Get_in_set_angle();
    f1_out_angle = Foot1_Left.Get_out_set_angle();//id=1
    f2_in_angle = Foot2_Right.Get_in_set_angle();
    f2_out_angle = Foot2_Right.Get_out_set_angle();//id=3
}

void CLASS_ROBOT::Handle_RC_Data(uint8_t *data)
{
    ibus_parse(&fsia6b_msg, data);

}