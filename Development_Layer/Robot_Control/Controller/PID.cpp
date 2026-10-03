//
// Created by 谭恩泽 on 2025/10/23.
//

#include "PID.h"

/**
 * @brief PID 控制器初始化
 *
 * @param K_p                          比例增益 P
 * @param K_i                          积分增益 I
 * @param K_d                          微分增益 D
 * @param K_f                          速度前馈增益, 正比于 Target 本身
 *                                     (仅当 Target 语义为"速度"时成立)
 * @param K_ff                         加速度前馈增益, 正比于 Target 的一阶差分
 *                                     有效增益 = K_ff * D_t
 * @param IOut_Max                     积分项【输出】限幅幅值, 0 为不限制
 *                                     (内部按 ±IOut_Max/K_i 限制积分累加值)
 * @param Output_Max                   总输出限幅幅值(P+I+D+F 求和后), 0 为不限制,
 *                                     建议显式传入, 默认 0 即不限制有风险
 * @param D_t                          计算周期, 单位 s, 必须与实际调用
 *                                     PID_Process() 的周期一致, 否则 I/D 量纲错误
 * @param Dead_zone                    误差死区, 0 为无死区; 死区外误差会被
 *                                     整体平移 Dead_zone 以保证输出连续
 * @param I_Variable_A                 变速积分"定速内段"阈值, |error|<=A 时全速积分
 * @param I_Variable_B                 变速积分"变速区间"上界, A<|error|<B 线性过渡,
 *                                     |error|>=B 停止积分; 须满足 A < B,
 *                                     且 A、B 同时为 0 时不启用
 * @param Integral_Separate_Threshold  积分分离阈值, 须为正数, 0 为不启用;
 *                                     |error| 超阈值时积分值直接清零
 * @param D_first                      微分先行开关, PID_D_First_ENABLE 时
 *                                     对测量值微分(抑制微分冲击),
 *                                     DISABLE 时对误差微分
 */
void Class_PID::Init(float K_p, float K_i, float K_d, float K_f, float K_ff,
                     float IOut_Max, float Output_Max, float D_t,
                     float Dead_zone, float I_Variable_A, float I_Variable_B,
                     float Integral_Separate_Threshold,
                     Enum_PID_D_First D_first) {
  K_P = K_p;
  K_I = K_i;
  K_D = K_d;
  K_F = K_f;
  K_FF = K_ff;
  ;
  I_Out_Max = IOut_Max;
  Out_Max = Output_Max;
  D_T = D_t;
  Dead_Zone = Dead_zone;
  I_Variable_Speed_A = I_Variable_A;
  I_Variable_Speed_B = I_Variable_B;
  I_Separate_Threshold = Integral_Separate_Threshold;
  D_First = D_first;
}

/**
 * @brief PID调整值, 计算周期与D_T相同
 *
 * @return float 输出值
 */
void Class_PID::PID_Process() {
  // P输出
  float p_out = 0.0f;
  // I输出
  float i_out = 0.0f;
  // D输出
  float d_out = 0.0f;
  // F输出
  float f_out = 0.0f;
  // 误差
  float error = 0.0f;
  // 绝对值误差
  float abs_error = 0.0f;
  // 线性变速积分
  float speed_ratio = 0.0f;

  error = Target - Now;
  abs_error = Math_Abs(error);

  // 判断死区
  if (abs_error < Dead_Zone) {
    Target = Now;
    error = 0.0f;
    abs_error = 0.0f;
  } else if (error > 0.0f && abs_error > Dead_Zone) {
    error -= Dead_Zone;
  } else if (error < 0.0f && abs_error > Dead_Zone) {
    error += Dead_Zone;
  }

  // 计算p项

  p_out = K_P * error;

  // 计算i项

  if (I_Variable_Speed_A == 0.0f && I_Variable_Speed_B == 0.0f) {
    // 非变速积分
    speed_ratio = 1.0f;
  } else {
    // 变速积分
    if (abs_error <= I_Variable_Speed_A) {
      speed_ratio = 1.0f;
    } else if (I_Variable_Speed_A < abs_error &&
               abs_error < I_Variable_Speed_B) {
      speed_ratio = (I_Variable_Speed_B - abs_error) /
                    (I_Variable_Speed_B - I_Variable_Speed_A);
    } else if (abs_error >= I_Variable_Speed_B) {
      speed_ratio = 0.0f;
    }
  }
  // 积分限幅
  if (I_Out_Max != 0.0f && K_I != 0.0f) {
    Math_Constrain(&Integral_Error, -I_Out_Max / K_I, I_Out_Max / K_I);
  }
  if (I_Separate_Threshold == 0.0f) {
    // 没有积分分离
    Integral_Error += speed_ratio * D_T * error;
    i_out = K_I * Integral_Error;
  } else {
    // 有积分分离
    if (abs_error < I_Separate_Threshold) {
      // 不在积分分离区间上
      Integral_Error += speed_ratio * D_T * error;
      i_out = K_I * Integral_Error;
    } else {
      // 在积分分离区间上
      Integral_Error = 0.0f;
      i_out = 0.0f;
    }
  }

  // 计算d项

  if (D_First == PID_D_First_DISABLE) {
    // 没有微分先行
    d_out = K_D * (error - Pre_Error) / D_T;
  } else {
    // 微分先行使能
    d_out = -K_D * (Now - Pre_Now) / D_T;
  }

  // 计算前馈

  f_out = K_F * Target + K_FF * (Target - Pre_Target);

  // 计算输出

  Out = p_out + i_out + d_out + f_out;

  // 输出限幅
  if (Out_Max != 0.0f) {
    Math_Constrain(&Out, -Out_Max, Out_Max);
  }

  // 善后工作
  Pre_Now = Now;
  Pre_Target = Target;
  Pre_Out = Out;
  Pre_Error = error;
}
