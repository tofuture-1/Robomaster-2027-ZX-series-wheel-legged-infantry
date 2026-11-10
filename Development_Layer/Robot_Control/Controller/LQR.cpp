//
// Created by 谭恩泽 on 2025/10/23.
//

#include "LQR.h"

#include "drv_math.h"

//控制律: u = -K * x
void Class_LQR::LQR_Calculate()
{

  Math_Constrain(&p_dot_fine, -15.0f,+15.0f);//HACK:底层限制m,传入前还有一层
  p += (p_dot - p_dot_fine * R_w) * D_t;//注意清0
  //HACK：此处的符号问题？貌似不需要 p_dot_fine * R_w？


  Math_Constrain(&p,-0.55f,0.55f);
//积分限幅，避免大范围积分漂移

  Math_Constrain(&phi_fine, -0.34889f- 0.17444f * 0.8f,+0.34889f- 0.17444f * 0.8f);//0.34889f对应20° 0.52333f对应30°
  phi_fine = - 0.17444f * 0.8f;

  // p =0.0f;
//TODO：打滑检测

//XXX:之前传入的速度方向错了！！！！！！！！！！！！
  tau_w   = -K[0][0]*p - K[0][1]*(p_dot - p_dot_fine * R_w) - K[0][2]*(theta - theta_fine) - K[0][3]*theta_dot - K[0][4]*(phi - phi_fine) - K[0][5]*phi_dot;
  tau_leg = -K[1][0]*p - K[1][1]*(p_dot - p_dot_fine * R_w) - K[1][2]*(theta - theta_fine) - K[1][3]*theta_dot - K[1][4]*(phi - phi_fine) - K[1][5]*phi_dot;
//状态量单位全部按 SI 单位定义

  Math_Constrain(&tau_w,-3.5f,3.5f);
  Math_Constrain(&tau_leg,-30.0f,30.0f);
  //HACK限幅在这里统一做？

}

//XXX:注意输出值的方向和大小（平均分配到两条腿上？）
//TODO:LQR 假设 x=0 是平衡点。实车上：
// Get_Pitch() 直立时约为 180°（你现在的 double -180 说明它就在 180 附近），需要标定 theta_eq；
// 站立时 VMC 的 phi 不一定正好为 0，需要标定 phi_eq。
// 否则控制器在“静止”时会输出一个常值力矩。建议：