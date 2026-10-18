//
// Created by 谭恩泽 on 2025/10/23.
//

#include "LQR.h"

//控制律: u = -K * x
void Class_LQR::LQR_Calculate()
{
  tau_w   = -K[1][1]*p - K[1][2]*p_dot - K[1][3]*theta - K[1][4]*theta_dot - K[1][5]*phi - K[1][6]*phi_dot;
  tau_leg = -K[2][1]*p - K[2][2]*p_dot - K[2][3]*theta - K[2][4]*theta_dot - K[2][5]*phi - K[2][6]*phi_dot;
}//XXX：注意数组越界了

//XXX:注意输出值的方向和大小（平均分配到两条腿上？）
