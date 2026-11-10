//
// Created by 24158 on 2026/8/8.
//

#ifndef H7_BOARD_CHASSIS_H
#define H7_BOARD_CHASSIS_H

#include "fsm_class.h"
#include "motor_dji.h"
#include "motor_dm.h"
#include "filter.h"
#include "LQR.h"
#include "VMC.h"
#include "fs_ia6b.h"


enum Enum_FSM_Chassis
{
    Chassis_STATE_IDLE = 0,
    Chassis_STATE_SELF_RESCUE,
    Chassis_STATE_NORMAL_LQR_BALANCE,
    Chassis_STATE_JUMPING,
    Chassis_STATE_OFF_GROUND,

};

/**
 * @brief Specialized, 轮毂电机, 加了滤波器
 *
 */
class Class_Chassis_Motor_DJI_C620 : public Class_Motor_DJI_C620
{
public:
    Class_Filter_Kalman Filter_Kalman_Omega;
    float after_filter_Omega = 0.0f;

    inline float Get_Now_Filter_Omega();

    virtual void RTOS_Calculate_Callback();
    void RTOS_Calculate_Callback_debug();

    void Set_Hub_Torque(float Torque){Hub_Torque = Torque + Hub_Torque_fine;};
    void Set_Hub_Torque_fine(float Torque){Hub_Torque_fine = Torque;};
    float Get_Now_Hub_Torque(){return Get_Now_Current() * 0.24628236875f; };
    //HACK:实际似乎会因为反电动势导致读取到的电流为0？无负载时确实没有力矩输出？


protected:
    // 初始化相关常量

    // 常量

    // 内部变量
    float Hub_Torque = 0.0f;//轮毂电机的输出力矩
    float Hub_Torque_fine = 0.0f;//用于差速的补偿值TODO：也可用于保证不乱转向?
    const float torque_constant_reciprocal = 4.06037998f;//单位：安培/牛米
    //此为取倒数（节约浮点数除法运算算力消耗？），原值为：0.3牛米/安培(搭配原有减速比3591/187减速箱时的数据）
    //（0.3/（3591/187））*268/17 = 0.24628236875f牛米/安培(目前268/17减速箱下的转矩常数)
    //const float reduction_ratio_reciprocal = 0.0634328f;//此为取倒数，原值为268/17;

    // 读变量

    // 写变量

    // 读写变量

    // 内部函数

    virtual  void PID_Calculate();//加入对力矩的控制

};

/**
 * @brief 获取当前滤波后的速度, rad/s
 *
 * @return float 当前滤波后的速度, rad/s
 */
inline float Class_Chassis_Motor_DJI_C620::Get_Now_Filter_Omega()
{
    return (Filter_Kalman_Omega.Get_Out());
}


class Class_FSM_Chassis : public Class_FSM
{

    public:



    void RTOS_Calculate_Callback();//重写状态转换逻辑


protected:


};

/**
 * @brief Specialized, 底盘类-用于轮腿步兵测试
 *
 */
class Class_Chassis
{
public:

    Class_Motor_DM_Normal M8009p_1;//对应杆零点为x负轴，逆时针为正//注意是看主动大腿杆的位置,正轴的方向定为前进（电池架）方向
    Class_Motor_DM_Normal M8009p_2;//对应杆零点为x正轴，逆时针为正
    Class_Motor_DM_Normal M8009p_3;//对应杆零点为x负轴，逆时针为正//注意是看主动大腿杆的位置
    Class_Motor_DM_Normal M8009p_4;//对应杆零点为x正轴，逆时针为正

    Class_Chassis_Motor_DJI_C620 Motor_Chassis_Left;

    Class_Chassis_Motor_DJI_C620 Motor_Chassis_Right;//HACK：3508默认正方向为电机输出轴的逆时针方向

    //状态机
    Class_FSM_Chassis Chassis_FSM;



    //控制相关
    Class_LQR Foot_LQR;//单LQR实例，假设双腿对称
    //Class_LQR Foot_LQR_Right;

    Class_VMC Foot_right_VMC;
    Class_VMC Foot_left_VMC;//HACK：规范化命名
    //VMC测试

    Class_FS_IA6B FS_I6X;//HACK:过渡写法,之后再剥离解耦到Robot层


    void Init();


    inline float Get_Chassis_Omega();


    inline void Set_Chassis_Omega(float __Chassis_Omega_Left,float __Chassis_Omega_Right);

    inline void Set_Banclance_Kp(float __Kp);
    inline void Set_Banclance_Kd(float __Kd);
    void Set_Chassis_Pitch(float __Pitch){Pitch = __Pitch;};
    void Set_Chassis_Pitch_dot(float __Pitch_dot){ Pitch_dot= __Pitch_dot;};


    //不同状态下的控制函数
    void IDLE_Control();
    void SELF_RESCUE_Control();
    void NORMAL_LQR_BALANCE_Control();
    void JUMPING_Control();
    void OFF_GROUND_Control();

    void Chassis_State_Update();//HACK：临时用于处理FSM状态转移


    void RTOS_100ms_Alive_Callback();

    void RTOS_1ms_Calculate_Callback();

protected:
    // 初始化相关常量

    Enum_FSM_Chassis Chassis_State = Chassis_STATE_IDLE;
    // 常量


    // 内部变量
    float Pitch = 0.0f;//Pitch轴角度,预期单位为度°
    float Pitch_dot = 0.0f;//Pitch轴角度,预期单位为弧度rad
//外部读取的数据HACK:是否可以再优化逻辑?
    int8_t state_change_flag = 0;//HACK：临时用于状态机特定切换是否开启的一个标志位



    // 读变量



    // 写变量



    // 读写变量



    //目标转速
    float Chassis_Omega_Left = 0.0f;
    float Chassis_Omega_Right = 0.0f;

    //IMU-PID相关参数
    float Banlance_Kp;
    float Banlance_Kd;

    //直立环控制输出量
    float Banlance_Out;






    // 内部函数




    void Output();
};

inline float Class_Chassis::Get_Chassis_Omega(){return Chassis_Omega_Left;}


inline void Class_Chassis::Set_Chassis_Omega(float __Chassis_Omega_Left,float __Chassis_Omega_Right)
{
    Chassis_Omega_Left = __Chassis_Omega_Left;
    Chassis_Omega_Right = __Chassis_Omega_Right;
}

inline void Class_Chassis::Set_Banclance_Kp(float __Kp){Banlance_Kp = __Kp;}
inline void Class_Chassis::Set_Banclance_Kd(float __Kd){Banlance_Kd = __Kd;}


#endif //H7_BOARD_CHASSIS_H
