//
// Created by 24158 on 2026/8/8.
//

#ifndef H7_BOARD_CHASSIS_H
#define H7_BOARD_CHASSIS_H

#include "fsm_class.h"
#include "motor_dji.h"
#include "filter.h"

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


protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    // 读变量

    // 写变量

    // 读写变量

    // 内部函数

    virtual  void PID_Calculate();

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



/**
 * @brief Specialized, 底盘类-测试
 *
 */
class Class_Chassis
{
public:

    Class_Chassis_Motor_DJI_C620 Motor_Chassis_Left;

    Class_Chassis_Motor_DJI_C620 Motor_Chassis_Right;


    void Init();


    inline float Get_Chassis_Omega();


    inline void Set_Chassis_Omega(float __Chassis_Omega_Left,float __Chassis_Omega_Right);
    inline void Set_Banclce_Kp(float __Kp);
    inline void Set_Banclce_Kd(float __Kd);


    void RTOS_100ms_Alive_Callback();

    void RTOS_1ms_Calculate_Callback();

protected:
    // 初始化相关常量

    // 常量


    // 内部变量




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

inline void Class_Chassis::Set_Banclce_Kp(float __Kp){Banlance_Kp = __Kp;}
inline void Class_Chassis::Set_Banclce_Kd(float __Kd){Banlance_Kd = __Kd;}


#endif //H7_BOARD_CHASSIS_H
