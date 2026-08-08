//
// Created by 谭恩泽 on 2025/10/23.
//

#ifndef C_BOARD_RBT_CLASS_H
#define C_BOARD_RBT_CLASS_H

#include "sampling_queue.h"
#include "fsm_class.h"
#include "slope_class.h"
#include "PID.h"
#include "booster.h"
#include "gimbal.h"
#include "dr16.h"
#include "imu.h"
#include "Serial_Debug.h"
#include "class_timer.h"
#include "referee.h"
#include "pc_driver.h"
#include "VTM_RX.h"
#include "g_c_communicate.h"
#include "lift.h"

#include "foot_control.h"
#include "fs_ia6b.h"
#include "../../Robot_Control/Chassis/chassis.h"

extern volatile bool robot_1ms_cycle_flag;
extern volatile bool robot_100ms_cycle_flag;
extern volatile bool robot_1000ms_cycle_flag ;





/**
 * @brief 长按左键状态类型
 *
 */
enum Enum_FSM_Press_Hold_Status
{
    FSM_Press_Hold_Status_STOP = 0,
    FSM_Press_Hold_Status_FREE,
    FSM_Press_Hold_Status_PRESSED,
    FSM_Press_Hold_Status_CLICK,
    FSM_Press_Hold_Status_HOLD,
};

/**
 * @brief 热量自检测状态类型
 *
 */
enum Enum_FSM_Heat_Detector_Status
{
    FSM_Heat_Detector_Status_CLOSE = 0,
    FSM_Heat_Detector_Status_OPEN,
};

/**
 * @brief 底盘类型
 *
 */
enum Enum_Robot_Chassis_Type
{
    Robot_Chassis_Type_POWER = 0,
    Robot_Chassis_Type_HP,
};

/**
 * @brief 小陀螺类型
 *
 */
enum Enum_Robot_Gyroscope_Type
{
    Robot_Gyroscope_Type_DISABLE = 0,
    Robot_Gyroscope_Type_CLOCKWISE,
    Robot_Gyroscope_Type_COUNTERCLOCKWISE,
};

/**
 * @brief 发射机构类型
 *
 */
enum Enum_Robot_Booster_Type
{
    Robot_Booster_Type_BURST = 0,
    Robot_Booster_Type_CD,
};
 // 控制模式枚举
enum Control_Mode 
{
    CONTROL_MODE_DISABLE = 0,   //失能状态
    CONTROL_MODE_REMOTE = 1,  // 遥控器控制模式
    CONTROL_MODE_KEYBOARD = 2, // 键盘控制模式
};


class CLASS_ROBOT;

/**
 * @brief Specialized, 长按左键有限自动机
 *
 */
class Class_FSM_Press_Hold : public Class_FSM
{
public:
    CLASS_ROBOT *Robot;

    uint32_t Get_Default_Hold_Time_Threshold();

    void Set_Hold_Time_Threshold(uint32_t __Hold_Time_Threshold);

    void RTOS_1ms_Calculate_Callback();

protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    // 读变量

    // 默认长按时间阈值, 超出被认为长按
    uint32_t Default_Hold_Time_Threshold = 100;

    // 写变量

    // 长按时间阈值, 超出被认为长按
    uint32_t Hold_Time_Threshold = 100;

    // 读写变量

    // 内部函数
};

/**
 * @brief Specialized, 热量自检测有限自动机
 *
 */
class Class_FSM_Heat_Detector : public Class_FSM
{
public:
    CLASS_ROBOT *Robot;

    float Get_Now_Heat();

    uint32_t Get_Total_Ammo_Num();

    void RTOS_1ms_Calculate_Callback();

protected:
    // 初始化相关常量

    // 常量

    // 最大队列长度
    uint32_t Max_Queue_Size = 100;
    // 电流队列和阈值
    float Current_Queue_Sum_Threshold = 100000.0f;

    // 内部变量

    // 电流值队列
    Class_Queue<float, 100> Current_Queue;
    // 电流值队列和
    float Current_Queue_Sum = 0.0f;

    // 读变量

    // 当前热量
    float Now_Heat = 0.0f;
    // 累计子弹数
    uint32_t Total_Ammo_Num = 0;

    // 写变量

    // 读写变量

    // 内部函数
};

class CLASS_ROBOT
{
public:
    //腿部运动学解析测试
    /** @brief 左后腿（CAN1, ID 0x11/0x12） */
    Class_SingleDogFoot_DM Foot1_Left;
    /** @brief 右后腿（CAN1, ID 0x14/0x13） */
    Class_SingleDogFoot_DM Foot2_Right;

    void FSi6x_control_polar();
    void Handle_RC_Data(uint8_t *data);

    Class_Chassis Chassis;
    //

    // 静态实例指针，用于回调函数访问
    static CLASS_ROBOT *Instance;

    // 当前控制模式
    Control_Mode current_control_mode = CONTROL_MODE_REMOTE;
    
    // 长按左键有限自动机
    Class_FSM_Press_Hold FSM_VT03_Left_Mouse_Press_Hold;

    friend class Class_FSM_Press_Hold;

    // 热量自检测逻辑
    Class_FSM_Heat_Detector FSM_Heat_Detector;

    friend class Class_FSM_Heat_Detector;

    // 底盘跟随PID
    Class_PID PID_Chassis_Follow;

    // 超级电容对底盘功率控制PID
    Class_PID PID_Supercap_Chassis_Power;

    // 裁判系统对底盘功率控制PID
    Class_PID PID_Referee_Chassis_Power;

    // 斜坡函数底盘速度x
    Class_Slope Slope_Speed_X;

    // 斜坡函数底盘速度y
    Class_Slope Slope_Speed_Y;

    // 斜坡函数底盘角速度
    Class_Slope Slope_Omega;

    // // 遥控器
    // Class_DR16 DR16;
    //图传遥控器
    Class_VTM_RX VT03;
    // 云台
    Class_Gimbal Gimbal;

    // 发射机构
    Class_Booster Booster;
    
    // 抬升机构
    Class_Lift Lift;

    //板载IMU
    Class_Board_IMU BMI088;

    User_Timer BMI088_Timer;
    
    // 测距模块
    Class_TOF050F TOF050F;
    
    // 电脑端
    Robot_PC NUC_PC;

    Class_Serial_Debug Debug;

    //底盘通讯
    G_C_Communicate The_Chassis;

    void Init();

    void RTOS_1000ms_Alive_PeriodElapsedCallback();

    void RTOS_100ms_Alive_PeriodElapsedCallback();

    void RTOS_100ms_Calculate_Callback();

    void RTOS_1ms_Conmunicate_Callback();

    void RTOS_1ms_Calculate_Callback();

    void Main_Loop_Things();
protected:

    //腿部运动学解析测试
    /** @brief FS-IA6B遥控器IBUS解析后的通道数据 */
    ibus_msg fsia6b_msg;

    /**
     * @brief 跳跃触发标志
     * @note 0=正常行走, 1=跳跃模式激活, 2=站立模式激活
     */
    uint8_t flag_jump = 0;

    /** @brief MIT控制模式参数：KP刚度(默认5.5), KD阻尼(0.1), 角度, 角速度, 前馈扭矩 */
    float kp = 5.5f,kd = 1.0f,angle = 0.0f,Omega = 0.0f,Torque = 0.00f;




    // 初始化相关常量

    // 常量

    //底盘小陀螺观测系数 
    const float K_Gyro_Omega = 1.0f;

    // 内部变量

    // 限制功率控制缓冲能量阈值
    uint16_t Restrict_Enable_Buffer_Energy = 20;
    uint16_t Restrict_Disable_Buffer_Energy = 55;

    // 小陀螺模式是否使能
    Enum_Robot_Gyroscope_Type Chassis_Gyroscope_Mode_Status = Robot_Gyroscope_Type_DISABLE;
    // 底盘跟随模式是否使能
    bool Chassis_Follow_Mode_Status = true;

    // 底盘属性
    Enum_Robot_Chassis_Type Robot_Chassis_Type = Robot_Chassis_Type_HP;
    // 发射机构属性
    Enum_Robot_Booster_Type Robot_Booster_Type = Robot_Booster_Type_CD;
    // 机器人等级
    int32_t Robot_Level = 1;

    // UI初始化属性
    bool UI_Init_Status = false;

    // 读变量
    // 自瞄Yaw偏移低通滤波器
    float Filtered_Yaw_Offset = 0.0f;
    float Filtered_Pitch_Offset = 0.0f;
    const float Yaw_Filter_Coeff = 0.3f;
    const float Pitch_Filter_Coeff = 0.3f;
    // 自瞄速度前馈低通滤波器
    float Filtered_Yaw_Vel = 0.0f;
    float Filtered_Pitch_Vel = 0.0f;
    const float Yaw_Vel_Filter_Coeff = 0.2f;
    const float Pitch_Vel_Filter_Coeff = 0.2f;
    // 自瞄速度前馈死区
    const float Yaw_Vel_DeadZone = 0.5f;
    const float Pitch_Vel_DeadZone = 0.3f;
    // 自瞄角度偏移死区（消除静止时抖动）
    const float Yaw_Offset_DeadZone = 0.5f;

    bool Auto_Aim_Enable = false;
    bool Booster_Disable_Flag = true;
    // 读写变量

    // 内部函数

    // 上位机数据解算
    static void PC_Receive_Callback(const PC_ReceivePacket &packet);

    void Chassis_Control();

    void Gimbal_Control();

    void Booster_Control();

    void Status_Control();

};

/**
 * @brief 设定长按时间阈值, 超出被认为长按
 *
 * @param __Hold_Time_Threshold 长按时间阈值, 超出被认为长按
 */
inline void Class_FSM_Press_Hold::Set_Hold_Time_Threshold(uint32_t __Hold_Time_Threshold)
{
    Hold_Time_Threshold = __Hold_Time_Threshold;
}

/**
 * @brief 设定长按时间阈值, 超出被认为长按
 *
 * @param __Hold_Time_Threshold 长按时间阈值, 超出被认为长按
 */
inline uint32_t Class_FSM_Press_Hold::Get_Default_Hold_Time_Threshold()
{
    return (Default_Hold_Time_Threshold);
}

/**
 * @brief 获取当前热量
 *
 * @return float 当前热量
 */
inline float Class_FSM_Heat_Detector::Get_Now_Heat()
{
    return (Now_Heat);
}

/**
 * @brief 获取累计子弹数
 *
 * @return float 累计子弹数
 */
inline uint32_t Class_FSM_Heat_Detector::Get_Total_Ammo_Num()
{
    return (Total_Ammo_Num);
}

#endif //C_BOARD_RBT_CLASS_H
