//
// Created by 谭恩泽 on 2025/10/23.
//

#ifndef C_BOARD_FILTER_H_H
#define C_BOARD_FILTER_H_H

#include <cstdint>
#include "drv_math.h"


// 采样频率
#define FOURIER_FILTER_DEFAULT_SAMPLING_FREQUENCY (1000.0f)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 滤波器类型
 *
 */
enum Enum_Filter_Fourier_Type
{
    Filter_Fourier_Type_LOWPASS = 0,
    Filter_Fourier_Type_HIGHPASS,
    Filter_Fourier_Type_BANDPASS,
    Filter_Fourier_Type_BANDSTOP,
};

/**
 * @brief Reusable, Fourier滤波器算法
 *
 */
template<uint32_t Filter_Fourier_Order = 50>
class Class_Filter_Fourier
{
public:
    void Init(float Value_Constrain_low = 0.0f, float Value_Constrain_high = 1.0f, Enum_Filter_Fourier_Type Filter_Fourier_type = Filter_Fourier_Type_LOWPASS, float Freq_Low = 0.0f, float Freq_High = FOURIER_FILTER_DEFAULT_SAMPLING_FREQUENCY / 2.0f, float Sampling_Frequency = FOURIER_FILTER_DEFAULT_SAMPLING_FREQUENCY);

    inline float Get_Out();

    inline void Set_Now(float now);

    void Filter_Calculate();

protected:
    // 初始化相关常量

    // 输入限幅
    float Value_Constrain_Low;
    float Value_Constrain_High;

    // 滤波器类型
    Enum_Filter_Fourier_Type Filter_Fourier_Type;
    // 滤波器特征低频
    float Frequency_Low;
    // 滤波器特征高频
    float Frequency_High;
    // 滤波器采样频率
    float Sampling_Frequency;

    // 常量

    // 内部变量

    // 卷积系统函数向量
    float System_Function[Filter_Fourier_Order + 1];

    // 输入信号向量
    float Input_Signal[Filter_Fourier_Order + 1];

    // 新数据指示向量
    uint8_t Signal_Flag = 0;

    // 读变量

    // 输出值
    float Out = 0.0f;

    // 写变量

    // 内部函数
};

/**
 * @brief 完整版一维 Kalman 滤波器
 *
 * 模型:
 *   x_k = x_{k-1} + w    (状态方程，恒定模型)
 *   z_k = x_k + v        (观测方程)
 *
 * 包含预测 + 更新步骤
 * 支持自适应过程噪声调节
 * 支持低滞后模式
 */
class Class_Filter_Kalman
{
public:
    void Init(float Process_Noise = 0.1f, float Measure_Noise = 1.0f, float Estimate_Error = 1.0f, float Initial_Value = 0.0f);

    inline float Get_Out()
    {
        return State_Estimate;
    }

    inline void Set_Measurement(float z)
    {
        Now = z;
    }

    void Filter_Calculate();
    
    // 自适应版本，根据残差自动调整 Q
    void Filter_Calculate_Adaptive(float Min_Q = 0.001f, float Max_Q = 1.0f);
    
    // 低滞后版本，使用更大的增益
    void Filter_Calculate_Low_Lag(float Lag_Compensation = 1.2f);
    
    // 自适应 + 低滞后复合版本
    void Filter_Calculate_Adaptive_Low_Lag(float Min_Q = 0.001f, float Max_Q = 1.0f, float Lag_Compensation = 1.2f);

    inline void Set_Process_Noise(float Q)
    {
        Process_Noise = Q;
    }

    inline void Set_Measure_Noise(float R)
    {
        Measure_Noise = R;
    }

    inline void Set_Estimate_Error(float P)
    {
        Estimate_Error = P;
    }
    
    // 动态调整过程噪声
    inline void Adjust_Process_Noise(float delta)
    {
        Process_Noise = fmaxf(fminf(Process_Noise + delta, Max_Process_Noise), Min_Process_Noise);
    }
    
    inline void Set_Process_Noise_Limits(float min_q, float max_q)
    {
        Min_Process_Noise = min_q;
        Max_Process_Noise = max_q;
    }

    // Getter 函数
    inline float Get_Process_Noise() const
    {
        return Process_Noise;
    }

    inline float Get_Measure_Noise() const
    {
        return Measure_Noise;
    }

    inline float Get_Estimate_Error() const
    {
        return Estimate_Error;
    }

    inline float Get_State_Estimate() const
    {
        return State_Estimate;
    }

    inline float Get_Kalman_Gain() const
    {
        return Kalman_Gain;
    }

    inline float Get_Measurement() const
    {
        return Now;
    }
    
    inline float Get_Innovation() const
    {
        return Innovation;
    }

protected:
    float Process_Noise = 0.001f;       // Q - 过程噪声协方差
    float Measure_Noise = 1.0f;       // R - 测量噪声协方差
    float Estimate_Error = 0.0283f;      // P - 估计误差协方差
    float State_Estimate = 0.0f;      // 状态估计
    float Kalman_Gain = 0.0f;         // K - 卡尔曼增益
    float Now = 0.0f;                 // 当前测量值
    float Innovation = 0.0f;          // 新息（残差）
    
    // 自适应参数
    float Min_Process_Noise = 0.001f;  // Q 的最小值
    float Max_Process_Noise = 1.0f;    // Q 的最大值
};


/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 *
 * @param Value_Constrain_low
 * @param Value_Constrain_high
 * @param Filter_Fourier_type
 * @param Freq_Low
 * @param Freq_High
 * @param Sampling_Freq
 */
template<uint32_t Filter_Fourier_Order>
void Class_Filter_Fourier<Filter_Fourier_Order>::Init(float Value_Constrain_low, float Value_Constrain_high, Enum_Filter_Fourier_Type Filter_Fourier_type, float Freq_Low, float Freq_High, float Sampling_Freq)
{
    Value_Constrain_Low = Value_Constrain_low;
    Value_Constrain_High = Value_Constrain_high;
    Filter_Fourier_Type = Filter_Fourier_type;
    Frequency_Low = Freq_Low;
    Frequency_High = Freq_High;
    Sampling_Frequency = Sampling_Freq;

    // 将所有计算所得值进行softmax操作成和为1的值
    float system_function_sum = 0.0f;
    // 特征低角速度
    float omega_low;
    // 特征高角速度
    float omega_high;

    omega_low = 2.0f * PI * Frequency_Low / Sampling_Frequency;
    omega_high = 2.0f * PI * Frequency_High / Sampling_Frequency;

    // 计算滤波器系统

    switch (Filter_Fourier_type)
    {
    case (Filter_Fourier_Type_LOWPASS):
    {
        for (uint32_t i = 0; i < Filter_Fourier_Order + 1; i++)
        {
            System_Function[i] = omega_low / PI * Math_Sinc((i - Filter_Fourier_Order / 2.0f) * omega_low);
        }

        break;
    }
    case (Filter_Fourier_Type_HIGHPASS):
    {
        for (uint32_t i = 0; i < Filter_Fourier_Order + 1; i++)
        {
            System_Function[i] = Math_Sinc((i - Filter_Fourier_Order / 2.0f) * PI) - omega_high / PI * Math_Sinc((i - Filter_Fourier_Order / 2.0f) * omega_high);
        }

        break;
    }
    case (Filter_Fourier_Type_BANDPASS):
    {
        for (uint32_t i = 0; i < Filter_Fourier_Order + 1; i++)
        {
            System_Function[i] = omega_high / PI * Math_Sinc((i - Filter_Fourier_Order / 2.0f) * omega_high) - omega_low / PI * Math_Sinc((i - Filter_Fourier_Order / 2.0f) * omega_low);
        }

        break;
    }
    case (Filter_Fourier_Type_BANDSTOP):
    {
        for (uint32_t i = 0; i < Filter_Fourier_Order + 1; i++)
        {
            System_Function[i] = Math_Sinc((i - Filter_Fourier_Order / 2.0f) * PI) + omega_low / PI * Math_Sinc((i - Filter_Fourier_Order / 2.0f) * omega_low) - omega_high / PI * Math_Sinc((i - Filter_Fourier_Order / 2.0f) * omega_high);
        }

        break;
    }
    }

    for (uint32_t i = 0; i < Filter_Fourier_Order + 1; i++)
    {
        system_function_sum += System_Function[i];
    }

    for (uint32_t i = 0; i < Filter_Fourier_Order + 1; i++)
    {
        System_Function[i] /= system_function_sum;
    }
}

/**
 * @brief 滤波器调整值, 周期与采样周期相同
 *
 * @tparam Filter_Fourier_Order 滤波器阶数
 */
template<uint32_t Filter_Fourier_Order>
void Class_Filter_Fourier<Filter_Fourier_Order>::Filter_Calculate()
{
    Out = 0.0f;

    // 执行卷积操作
    for (uint32_t i = 0; i < Filter_Fourier_Order + 1; i++)
    {
        Out += System_Function[i] * Input_Signal[(Signal_Flag + i) % (Filter_Fourier_Order + 1)];
    }
}

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
template<uint32_t Filter_Fourier_Order>
inline float Class_Filter_Fourier<Filter_Fourier_Order>::Get_Out()
{
    return (Out);
}

/**
 *
 * @param now
 */
template<uint32_t Filter_Fourier_Order>
inline void Class_Filter_Fourier<Filter_Fourier_Order>::Set_Now(float now)
{
    // 输入限幅, 全0为不限制
    if (Value_Constrain_Low != 0.0f || Value_Constrain_High != 0.0f)
    {
        Math_Constrain(&now, Value_Constrain_Low, Value_Constrain_High);
    }

    // 将当前值放入被卷积的信号中
    Input_Signal[Signal_Flag] = now;
    Signal_Flag++;

    // 若越界则轮回
    if (Signal_Flag == Filter_Fourier_Order + 1)
    {
        Signal_Flag = 0;
    }
}

#endif //C_BOARD_FILTER_H_H
