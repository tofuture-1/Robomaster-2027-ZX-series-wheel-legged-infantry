//
// Created by 谭恩泽 on 2025/10/23.
//

#ifndef C_BOARD_BOOSTER_H
#define C_BOARD_BOOSTER_H

#include "fsm_class.h"
#include "motor_dji.h"
#include "filter.h"




/**
 * @brief Specialized, 摩擦轮电机, 加了滤波器
 *
 */
class Class_Booster_Friction_Motor_DJI_C620 : public Class_Motor_DJI_C620
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
 * @brief Specialized, 拨弹盘电机C610, 加了滤波器
 *
 */
class Class_Booster_Driver_Motor_DJI_C610 : public Class_Motor_DJI_C610
{
public:
    Class_Filter_Kalman Filter_Kalman_Omega;
    float after_filter_Omega = 0.0f;

    inline float Get_Now_Filter_Omega();

    virtual void RTOS_Calculate_Callback();

protected:
    virtual void PID_Calculate();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取当前滤波后的速度, rad/s
 *
 * @return float 当前滤波后的速度, rad/s
 */
inline float Class_Booster_Friction_Motor_DJI_C620::Get_Now_Filter_Omega()
{
    return (Filter_Kalman_Omega.Get_Out());
}

inline float Class_Booster_Driver_Motor_DJI_C610::Get_Now_Filter_Omega()
{
    return (Filter_Kalman_Omega.Get_Out());
}


/*以上是云台电机实现，下面是云台部分具体实现*/

/**
 * @brief 发射机构控制类型
 *
 */
enum Enum_Booster_Control_Type
{
    Booster_Control_Type_DISABLE = 0,
    Booster_Control_Type_CEASEFIRE,
    Booster_Control_Type_SPOT,
    Booster_Control_Type_AUTO,
};

/**
 * @brief 防卡弹状态类型
 *
 */
enum Enum_FSM_Anti_Jamming_Status
{
    FSM_Anti_Jamming_Status_NORMAL = 0,
    FSM_Anti_Jamming_Status_JAMMING_SUSPECT,
    FSM_Anti_Jamming_Status_JAMMING_CONFIRM,
    FSM_Anti_Jamming_Status_PROCESSING,
};

/**
 * @brief 热量检测状态类型
 *
 */
enum Enum_FSM_Heat_Detect_Status
{
    FSM_Heat_Detect_Status_NORMAL = 0,
    FSM_Heat_Detect_Status_SHOOT_SUSPECT,
    FSM_Heat_Detect_Status_SHOOT_FINISHED,
    FSM_Heat_Detect_Status_FRICTION_STOP,
};

class Class_Booster;

/**
 * @brief Specialized, 卡弹策略有限自动机
 *
 */
class Class_FSM_Anti_Jamming : public Class_FSM
{
public:
    Class_Booster *Booster;

    void RTOS_1ms_Calculate_Callback();

protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    // 拨弹盘堵转扭矩阈值, 超出被认为卡弹
    float Driver_Torque_Threshold = 6.5f;
    // 卡弹判定持续时间阈值, 超出被认为卡弹
    uint32_t Jamming_Suspect_Time_Threshold = 500;
    // 卡弹处理持续时间阈值, 超出则认为处理结束
    uint32_t Jamming_Solving_Time_Threshold = 300;
    // 卡弹处理回退角度
    float Driver_Back_Angle = PI / 4.0f;

    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
};

/**
 * @brief Specialized, 发射机构类
 *
 */
class Class_Booster
{
public:

    // 卡弹策略有限自动机
    Class_FSM_Anti_Jamming FSM_Anti_Jamming;

    friend class Class_FSM_Anti_Jamming;

    // 拨弹盘电机
    Class_Booster_Driver_Motor_DJI_C610 Motor_Driver;

    // 摩擦轮电机左
    Class_Booster_Friction_Motor_DJI_C620 Motor_Friction_Left;

    // 摩擦轮电机右
    Class_Booster_Friction_Motor_DJI_C620 Motor_Friction_Right;

    void Init();

    inline float Get_Now_Ammo_Shoot_Frequency();

    inline Enum_Booster_Control_Type Get_Booster_Control_Type();

    inline float Get_Friction_Omega();

    inline float Get_Target_Ammo_Shoot_Frequency();

    inline void Set_Now_Heat(float __Now_Heat);

    inline void Set_Heat_Limit_Max(float __Heat_Limit_Max);

    inline void Set_Heat_CD(float __Heat_CD);

    inline void Set_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type);

    inline void Set_Friction_Omega(float __Friction_Omega);

    inline void Set_Max_Ammo_Shoot_Frequency(float __Max_Ammo_Shoot_Frequency);

    inline void Set_Driver_Direction(int8_t __Driver_Direction);

    void RTOS_100ms_Alive_Callback();

    void RTOS_1ms_Calculate_Callback();

protected:
    // 初始化相关常量

    // 常量

    // 拨弹盘一圈子弹数
    float Ammo_Num_Per_Round = 8.0f;

    // 内部变量

    // 停火热量阈值
    float Heat_Limit_Ceasefire_Threshold = 20.0f;
    // 减速热量阈值
    float Heat_Limit_Slowdown_Threshold = 50.0f;

    // 读变量

    // 拨弹盘真实每秒子弹数
    float Now_Ammo_Shoot_Frequency = 10.0f;

    // 写变量

    // 当前热量
    float Now_Heat = 0.0f;
    // 热量限制上限
    float Heat_Limit_Max = 0.0f;
    // 热量冷却速度
    float Heat_CD = 0.0f;

    // 读写变量

    // 发射机构状态
    Enum_Booster_Control_Type Booster_Control_Type = Booster_Control_Type_DISABLE;

    // 摩擦轮角速度
    float Friction_Omega = 620.0f;

    // 拨弹盘目标每秒子弹数
    float Target_Ammo_Shoot_Frequency = 8.0f;

    // 拨弹盘方向：1=正转, -1=反转, 0=停止
    int8_t Driver_Direction = 1;

    // 内部函数

    void Output();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取拨弹盘真实每秒子弹数
 *
 * @return float 拨弹盘真实每秒子弹数
 */
inline float Class_Booster::Get_Now_Ammo_Shoot_Frequency()
{
    return (Now_Ammo_Shoot_Frequency);
}

/**
 * @brief 获取发射机构状态
 *
 * @return float 发射机构状态
 */
inline Enum_Booster_Control_Type Class_Booster::Get_Booster_Control_Type()
{
    return (Booster_Control_Type);
}

/**
 * @brief 获取摩擦轮角速度
 *
 * @return float 摩擦轮角速度
 */
inline float Class_Booster::Get_Friction_Omega()
{
    return (Friction_Omega);
}

/**
 * @brief 获取拨弹盘目标每秒子弹数
 *
 * @return float 拨弹盘目标每秒子弹数
 */
inline float Class_Booster::Get_Target_Ammo_Shoot_Frequency()
{
    return (Target_Ammo_Shoot_Frequency);
}

/**
 * @brief 设定当前热量
 *
 * @param __Now_Heat 当前热量
 */
inline void Class_Booster::Set_Now_Heat(float __Now_Heat)
{
    Now_Heat = __Now_Heat;
}

/**
 * @brief 设定热量限制上限
 *
 * @param __Heat_Limit_Max 热量限制上限
 */
inline void Class_Booster::Set_Heat_Limit_Max(float __Heat_Limit_Max)
{
    Heat_Limit_Max = __Heat_Limit_Max;
}

/**
 * @brief 设定热量冷却速度
 *
 * @param __Heat_CD 热量冷却速度
 */
inline void Class_Booster::Set_Heat_CD(float __Heat_CD)
{
    Heat_CD = __Heat_CD;
}

/**
 * @brief 设定发射机构状态
 *
 * @param __Booster_Control_Type 发射机构状态
 */
inline void Class_Booster::Set_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type)
{
    Booster_Control_Type = __Booster_Control_Type;
}

/**
 * @brief 设定摩擦轮角速度
 *
 * @param __Friction_Omega 摩擦轮角速度
 */
inline void Class_Booster::Set_Friction_Omega(float __Friction_Omega)
{
    Friction_Omega = __Friction_Omega;
}

/**
 * @brief 设定拨弹盘最大每秒子弹数
 *
 * @param __Friction_Omega 拨弹盘最大每秒子弹数
 */
inline void Class_Booster::Set_Max_Ammo_Shoot_Frequency(float __Max_Ammo_Shoot_Frequency)
{
    Target_Ammo_Shoot_Frequency = __Max_Ammo_Shoot_Frequency;
}

inline void Class_Booster::Set_Driver_Direction(int8_t __Driver_Direction)
{
    Driver_Direction = __Driver_Direction;
}


#endif //C_BOARD_BOOSTER_H
