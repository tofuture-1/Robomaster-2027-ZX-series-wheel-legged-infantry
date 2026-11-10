//
// Created by 24158 on 2026/10/5.
//

#include "VMC.h"

#include "drv_math.h"


void Class_VMC::Init()
{

}

void Class_VMC::Kinematic_Solute()
{


    // 输入：theta1, theta2（已补偿零点的物理角，单位 rad）
    // 输出：xD, yD（轮轴坐标）

    // 1. 计算 B, C 坐标
    float xB = -0.21f * cosf(theta1);//传入弧度值
    float yB = -0.21f * sinf(theta1);
    float xC =  0.21f * cosf(theta2);
    float yC =  0.21f * sinf(theta2);


    // float theta1 = 0.0f;//需要以传入的是零点为x负轴，逆时针为正的真实杆物理角（补偿后）
    // float theta2 = 0.0f ;//零点为x正轴，逆时针为正

    // 2. 圆心距
    float dx = xC - xB;
    float dy = yC - yB;
    float d = sqrtf(dx*dx + dy*dy);

    // 3. 存在性检查
    float r = 0.25f;
    if (d > 2.0f * r - 1e-6f) {
        // 无解或接近奇异，报错，保持上一周期输出
        return ;
    }
    if (d < 1e-6f) {
        // 两圆心重合，奇异
        return ;
    }//FIXME:失败时没有置无效标志，成员也未初始化，也无后续处理

    // 4. 中点
    float Mx = 0.5f * (xB + xC);
    float My = 0.5f * (yB + yC);

    // 5. 半弦长
    float h = sqrtf(r*r - 0.25f*d*d);//TODO：减少除法的调用可以节约计算资源？

    // 6. 垂直单位向量
    float nx = -dy / d;
    float ny =  dx / d;

    // 7. 两个交点
    // float xD1 = Mx + h * nx;
    // float yD1 = My + h * ny;
    float xD2 = Mx - h * nx;
    float yD2 = My - h * ny;

    // 8. 选fabs(yD1) > fabs(yD2)？？
    // if (yD1 < yD2) {
    //     x_d = xD1; y_d = yD1;
    // } else {
        x_d = xD2; y_d = yD2;//恒选择该点（与BC垂直且指向远离髋关节方向的单位向量与半弦长的乘积算出）
    // }
    //DONE：2610221413恒选择该点（与BC垂直且指向远离髋关节方向的单位向量与半弦长的乘积算出），否则某些临界位置会跳变
    //如果不取绝对值会导致导致算不出来正的那个
    //DONE：2610162131解决原有解算导致的跨x轴解算错误的问题
    //TODO：跨x轴之后的收腿逻辑
    // 9. 可选：计算 L 和 phi
    L = sqrtf(x_d*x_d + y_d*y_d);
    phi = atan2f(x_d, -y_d);//HACK：超出3.14的跳变是否需要处理？




}



void Class_VMC::Jacobian_Matrix_Calculate()
{
    //计算当前腿长
    L = sqrt(x_d * x_d + y_d * y_d);


    A1 = x_d + l1*cosf(theta1);
    A2 = x_d - l1*cosf(theta2);
    B1 = y_d + l1*sinf(theta1);
    B2 = y_d - l1*sinf(theta2);

    delta = A1*B2 - A2*B1;//中间变量，仅为后续简写使用
    //FIXME:雅可比分母 delta 与L 没有奇异性检查

    K1 = A1*sinf(theta1) - B1*cosf(theta1) ;
    K2 = - A2*sinf(theta2) + B2*cosf(theta2) ;


    Jacobian_Matrix[0][0] = (l1*K1*(x_d*B2 - y_d*A2))/(delta * L);//等效腿长L对theta1的偏导
    Jacobian_Matrix[0][1] = (l1*K2*(y_d*A1 - x_d*B1))/(delta * L);//L对theta2的偏导

    Jacobian_Matrix[1][0] = - (l1*K1*(y_d*B2 + x_d*A2))/(delta * L * L);//等效倾角phi对theta1的偏导
    Jacobian_Matrix[1][1] = (l1*K2*(y_d*B1 + x_d*A1))/(delta * L * L);//等效倾角phi对theta2的偏导

    //TODO：理解这部分的解算
}
//HACK：目前各部分函数的职责
//TODO：重新完成运动学正解和逆解，TODO：并尝试算法上的优化？

void Class_VMC::VMC_Calculate()
{
    Kinematic_Solute();//HACK；临时合并在本函数中，后续再优化封装
    Jacobian_Matrix_Calculate();

    L_dot = Jacobian_Matrix[0][0]*theta1_dot + Jacobian_Matrix[0][1]*theta2_dot;//需要传入电机的速度值
    phi_dot = Jacobian_Matrix[1][0]*theta1_dot + Jacobian_Matrix[1][1]*theta2_dot;





}//算F_L

// void Class_VMC::VMC_leg_swing_torque_restriction()
// {
//     if (fabs(phi) > 1.5707963 && leg_swing_torque > 0)
//     {
//         leg_swing_torque_restriction = leg_swing_torque - 4*(phi-);
//     }
// }
//TODO:待完善

void Class_VMC::VMC_Motor_tau_map()//HACK：注意方向（取负值），此前解算直接适用于右腿，但左腿的电机方向会有区别
{
    F_L = K_L*(L_des - L) - D_L * L_dot + G_Fine;
    leg_swing_torque_delta_fine = - (K_pe * delta_leg_rl_phi + D_pe * delta_leg_rl_phi_dot );
    //左右腿差角补偿力矩
    //TODO：整理符号映射到实际输出的逻辑，并优化当前的

    // Math_Constrain(&leg_swing_torque_delta_fine,-4.0f,4.0f);
    // //HACK：临时限幅

    if (leg_swing_torque > 30.0f)leg_swing_torque = 30.0f;
    if (leg_swing_torque < -30.0f)leg_swing_torque = -30.0f;//HACK:限幅在LQR中就做？暂时保留这一层

    if (F_L > 400.0f)F_L = 400.0f;
    if (F_L < -400.0f)F_L = -400.0f;//HACK:限幅在LQR中就做？暂时保留这一层

    tau_1 = Jacobian_Matrix[0][0] * F_L + Jacobian_Matrix[1][0] * (leg_swing_torque + leg_swing_torque_delta_fine);
    tau_2 = Jacobian_Matrix[0][1] * F_L + Jacobian_Matrix[1][1] * (leg_swing_torque + leg_swing_torque_delta_fine);
    //DONE：2026101717xx这部分的倾倒力矩似乎不正常生效；之前雅可比矩阵部分的某项符号错了

    //最终关节力矩的映射
    //XXX：leg_swing_torque正值 = 腿向后摆（顺时针）

}