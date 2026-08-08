//
// Created by 谭恩泽 on 2025/10/23.
//

#include "filter.h"

void Class_Filter_Kalman::Init(float Process_Noise, float Measure_Noise, float Estimate_Error, float Initial_Value)
{
    this->Process_Noise = Process_Noise;
    this->Measure_Noise = Measure_Noise;
    this->Estimate_Error = Estimate_Error;
    this->State_Estimate = Initial_Value;
}

void Class_Filter_Kalman::Filter_Calculate()
{
    float P_Pred = Estimate_Error + Process_Noise;

    if (P_Pred + Measure_Noise > 0.0001f)
    {
        Kalman_Gain = P_Pred / (P_Pred + Measure_Noise);
    }
    else
    {
        Kalman_Gain = 0.0f;
    }

    Innovation = Now - State_Estimate;
    State_Estimate = State_Estimate + Kalman_Gain * Innovation;
    Estimate_Error = (1.0f - Kalman_Gain) * P_Pred;
}

/**
 * @brief 自适应卡尔曼滤波
 * @note 根据新息（残差）的统计特性自动调整过程噪声 Q
 * 
 * 原理：
 * - 当残差较大时，说明模型跟不上系统变化，增大 Q
 * - 当残差较小时，说明系统稳定，减小 Q
 * - 使用滑动窗口平均来估计残差方差
 */
void Class_Filter_Kalman::Filter_Calculate_Adaptive(float Min_Q, float Max_Q)
{
    float P_Pred = Estimate_Error + Process_Noise;

    if (P_Pred + Measure_Noise > 0.0001f)
    {
        Kalman_Gain = P_Pred / (P_Pred + Measure_Noise);
    }
    else
    {
        Kalman_Gain = 0.0f;
    }

    // 计算新息（残差）
    Innovation = Now - State_Estimate;
    
    // 更新状态估计
    State_Estimate = State_Estimate + Kalman_Gain * Innovation;
    
    // 更新估计误差协方差
    Estimate_Error = (1.0f - Kalman_Gain) * P_Pred;
    
    // 自适应调整过程噪声 Q
    // 优化策略：静止时快速降到 Min_Q，运动时适度增大
    float innovation_abs = fabsf(Innovation);
    
    if (innovation_abs > 0.3f)
    {
        // 残差大，快速增大 Q 提高响应
        Process_Noise = fminf(Process_Noise * 1.15f + 0.02f, Max_Q);
    }
    else if (innovation_abs < 0.03f)
    {
        // 残差小，快速减小 Q 提高滤波效果
        Process_Noise = fmaxf(Process_Noise * 0.92f - 0.002f, Min_Q);
    }
    // 中间状态保持 Q 不变，避免震荡
    
    // 限制 Q 的范围
    Process_Noise = fmaxf(fminf(Process_Noise, Max_Q), Min_Q);
}

/**
 * @brief 低滞后卡尔曼滤波
 * @note 通过增加卡尔曼增益来减少相位滞后
 * 
 * @param Lag_Compensation 滞后补偿系数 (1.0-2.0)，越大滞后越小但噪声越多
 *        - 1.0: 标准卡尔曼滤波
 *        - 1.2: 轻微补偿 (推荐)
 *        - 1.5: 中等补偿
 *        - 2.0: 强补偿 (可能引入噪声)
 */
void Class_Filter_Kalman::Filter_Calculate_Low_Lag(float Lag_Compensation)
{
    float P_Pred = Estimate_Error + Process_Noise;

    if (P_Pred + Measure_Noise > 0.0001f)
    {
        // 计算基础卡尔曼增益
        float K_base = P_Pred / (P_Pred + Measure_Noise);
        
        // 应用滞后补偿：增大增益
        Kalman_Gain = K_base * Lag_Compensation;
        
        // 限制增益不超过 1.0 (防止发散)
        if (Kalman_Gain > 1.0f)
        {
            Kalman_Gain = 1.0f;
        }
    }
    else
    {
        Kalman_Gain = 0.0f;
    }

    Innovation = Now - State_Estimate;
    State_Estimate = State_Estimate + Kalman_Gain * Innovation;
    Estimate_Error = (1.0f - Kalman_Gain) * P_Pred;
}

/**
 * @brief 自适应 + 低滞后复合卡尔曼滤波
 * @note 结合自适应和低滞后的优点，既能减少滞后又能自适应调整
 * 
 * @param Min_Q 最小过程噪声
 * @param Max_Q 最大过程噪声
 * @param Lag_Compensation 滞后补偿系数 (1.0-2.0)
 */
void Class_Filter_Kalman::Filter_Calculate_Adaptive_Low_Lag(float Min_Q, float Max_Q, float Lag_Compensation)
{
    float P_Pred = Estimate_Error + Process_Noise;

    if (P_Pred + Measure_Noise > 0.0001f)
    {
        // 计算基础卡尔曼增益
        float K_base = P_Pred / (P_Pred + Measure_Noise);
        
        // 应用滞后补偿：增大增益
        Kalman_Gain = K_base * Lag_Compensation;
        
        // 限制增益不超过 1.0 (防止发散)
        if (Kalman_Gain > 1.0f)
        {
            Kalman_Gain = 1.0f;
        }
    }
    else
    {
        Kalman_Gain = 0.0f;
    }

    // 计算新息（残差）
    Innovation = Now - State_Estimate;
    
    // 更新状态估计
    State_Estimate = State_Estimate + Kalman_Gain * Innovation;
    
    // 更新估计误差协方差
    Estimate_Error = (1.0f - Kalman_Gain) * P_Pred;
    
    // 自适应调整过程噪声 Q
    // 优化策略：静止时快速降到 Min_Q，运动时适度增大
    float innovation_abs = fabsf(Innovation);
    
    if (innovation_abs > 0.3f)
    {
        // 残差大，快速增大 Q 提高响应
        Process_Noise = fminf(Process_Noise * 1.15f + 0.02f, Max_Q);
    }
    else if (innovation_abs < 0.03f)
    {
        // 残差小，快速减小 Q 提高滤波效果
        Process_Noise = fmaxf(Process_Noise * 0.92f - 0.002f, Min_Q);
    }
    // 中间状态保持 Q 不变，避免震荡
    
    // 限制 Q 的范围
    Process_Noise = fmaxf(fminf(Process_Noise, Max_Q), Min_Q);
}
