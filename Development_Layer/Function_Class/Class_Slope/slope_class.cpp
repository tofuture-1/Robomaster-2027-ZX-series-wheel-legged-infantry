//
// Created by 谭恩泽 on 2025/11/1.
//

#include "slope_class.h"

/**
 * @brief 初始化
 *
 * @param __Increase_Value 增长最大幅度
 * @param __Decrease_Value 降低最大幅度
 */
void Class_Slope::Init(float __Increase_Value, float __Decrease_Value, Enum_Slope_First __Slope_First)
{
    Increase_Value = __Increase_Value;
    Decrease_Value = __Decrease_Value;
    Slope_First = __Slope_First;
}

/**
 * @brief 斜坡函数调整值, 计算周期取决于调用者
 *
 */
void Class_Slope::RTOS_1ms_Calculate_Callback()
{
    // 规划为当前真实值优先的额外逻辑
    if (Slope_First == Slope_First_REAL)
    {
        if ((Target >= Now_Real && Now_Real >= Now_Planning) || (Target <= Now_Real && Now_Real <= Now_Planning))
        {
            Out = Now_Real;
        }
    }

    if (Now_Planning > 0.0f)
    {
        if (Target > Now_Planning)
        {
            // 正值加速
            if (Math_Abs(Now_Planning - Target) > Increase_Value)
            {
                Out += Increase_Value;
            }
            else
            {
                Out = Target;
            }
        }
        else if (Target < Now_Planning)
        {
            // 正值减速
            if (Math_Abs(Now_Planning - Target) > Decrease_Value)
            {
                Out -= Decrease_Value;
            }
            else
            {
                Out = Target;
            }
        }
    }
    else if (Now_Planning < 0.0f)
    {
        if (Target < Now_Planning)
        {
            // 负值加速
            if (Math_Abs(Now_Planning - Target) > Increase_Value)
            {
                Out -= Increase_Value;
            }
            else
            {
                Out = Target;
            }
        }
        else if (Target > Now_Planning)
        {
            // 负值减速
            if (Math_Abs(Now_Planning - Target) > Decrease_Value)
            {
                Out += Decrease_Value;
            }
            else
            {
                Out = Target;
            }
        }
    }
    else
    {
        if (Target > Now_Planning)
        {
            // 0值正加速
            if (Math_Abs(Now_Planning - Target) > Increase_Value)
            {
                Out += Increase_Value;
            }
            else
            {
                Out = Target;
            }
        }
        else if (Target < Now_Planning)
        {
            // 0值负加速
            if (Math_Abs(Now_Planning - Target) > Increase_Value)
            {
                Out -= Increase_Value;
            }
            else
            {
                Out = Target;
            }
        }
    }

    // 善后工作
    Now_Planning = Out;
}
