//
// Created by 谭恩泽 on 2025/11/1.
//

#ifndef C_BOARD_SLOPE_H
#define C_BOARD_SLOPE_H


#include "drv_math.h"


/**
 * @brief 规划优先类型, 分为目标值优先和真实值优先
 * 目标值优先, 即硬规划
 * 真实值优先, 即当前真实值夹在当前规划值和目标值之间, 当前规划值转为当前真实值
 *
 */
enum Enum_Slope_First
{
    Slope_First_REAL = 0,
    Slope_First_TARGET,
};

/**
 * @brief Reusable, 斜坡函数本体
 *
 */
class Class_Slope
{
public:
    void Init(float __Increase_Value, float __Decrease_Value, Enum_Slope_First __Slope_First = Slope_First_REAL);

    inline float Get_Out();

    inline void Set_Now_Real(float __Now_Real);

    inline void Set_Increase_Value(float __Increase_Value);

    inline void Set_Decrease_Value(float __Decrease_Value);

    inline void Set_Target(float __Target);

    void RTOS_1ms_Calculate_Callback();

protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    // 读变量

    // 输出值
    float Out = 0.0f;

    // 写变量

    // 规划优先类型
    Enum_Slope_First Slope_First = Slope_First_REAL;
    // 当前规划值
    float Now_Planning = 0.0f;
    // 当前真实值
    float Now_Real = 0.0f;

    // 绝对值增量, 一次计算周期改变值
    float Increase_Value = 0.0f;
    // 绝对值减量, 一次计算周期改变值
    float Decrease_Value = 0.0f;
    // 目标值
    float Target = 0.0f;

    // 读写变量

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取输出值
 *
 * @return 输出值
 */
inline float Class_Slope::Get_Out()
{
    return (Out);
}

/**
 * @brief 设定当前规划值
 *
 * @param __Now_Real 当前规划值
 */
inline void Class_Slope::Set_Now_Real(float __Now_Real)
{
    Now_Real = __Now_Real;
}

/**
 * @brief 设定绝对值增量, 一次计算周期改变值
 *
 * @param __Increase_Value 绝对值增量, 一次计算周期改变值
 */
inline void Class_Slope::Set_Increase_Value(float __Increase_Value)
{
    Increase_Value = __Increase_Value;
}

/**
 * @brief 设定绝对值减量, 一次计算周期改变值
 *
 * @param __Decrease_Value 绝对值减量, 一次计算周期改变值
 */
inline void Class_Slope::Set_Decrease_Value(float __Decrease_Value)
{
    Decrease_Value = __Decrease_Value;
}

/**
 * @brief 设定目标值
 *
 * @param __Target 目标值
 */
inline void Class_Slope::Set_Target(float __Target)
{
    Target = __Target;
}


#endif //C_BOARD_SLOPE_H
