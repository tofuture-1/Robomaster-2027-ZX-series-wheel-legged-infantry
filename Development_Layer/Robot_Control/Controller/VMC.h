//
// Created by 24158 on 2026/10/5.
//

#ifndef H7_BOARD_VMC_H
#define H7_BOARD_VMC_H

#include <math.h>


#define l1 0.21f //单位：米

#define l2 0.25f

class Class_VMC
{
public:




    void Init();

    void VMC_Motor_tau_map();

    void Kinematic_Solute();

    void VMC_Calculate();


    // void Set_x_d(float x){x_d = x;};
    // void Set_y_d(float y){y_d = y;};貌似不需要

    void Set_theta1(float theta){theta1 = theta ;};
    void Set_theta2(float theta){theta2 = theta ;};
    void Set_L_des(float L){L_des = L;};
    void Set_theta1_dot(float theta){theta1_dot = theta;};
    void Set_theta2_dot(float theta){theta2_dot = theta;};
    void Set_leg_swing_torque(float swing_torque){leg_swing_torque = swing_torque;};

    float Get_x_d(){return x_d;};
    float Get_y_d(){return y_d;};
    float Get_L(){return L;};
    float Get_L_dot(){return L_dot;};
    float Get_theta1(){return theta1;};
    float Get_theta2(){return theta2;};
    float Get_L_des(){return L_des;};
    float Get_K_L(){return K_L;};
    float Get_D_L(){return D_L;};
    float Get_G_Fine(){return G_Fine;};
    float Get_F_L(){return F_L;};
    float Get_phi(){return phi;};
    float Get_leg_swing_torque(){return leg_swing_torque;};

    float Get_Motor_tau1(){return tau_1;};
    float Get_Motor_tau2(){return tau_2;};


    protected:
    float Jacobian_Matrix[2][2];

    float L_dot = 0.0f;//等效腿长变化速率
    float phi_dot = 0.0f;
    float theta1_dot = 0.0f;//关节电机角速度
    float theta2_dot = 0.0f;

    float L_des = 0.27f;//目标腿长,单位是米
    float K_L = 550.0f;//弹簧项
    float D_L = 100.5f;//阻尼项
    float G_Fine = 0.0f;//重力前馈TODO：是否需要考虑再不同倾角下的力矩输出？HACK：可以暂时直接利用cos映射一下分量XXX：注意做离地检测防止离地时堵转
    //注意这是单条腿的
    float F_L = 0.0f;//虚拟腿长力


    float leg_swing_torque = 0.0f;//等效摆腿力矩
    //目前实测力矩在5.0f左右就可以实现轮抬到到接近水平位置
    //同时方向为顺时针为正，逆时针为负

    //足端轮轴坐标——以髋关节为原点建系
    float x_d;
    float y_d;
    float L;//等效腿长——髋关节和轮轴连杆长度
    float phi;//等效倾角（等效腿与y轴负半轴夹角）


    float theta1;//需要以传入的是零点为x负轴，逆时针为正的真实杆物理角（补偿后）
    float theta2;//零点为x正轴，逆时针为正
    float theta_fine = 0.16802f;
    //目前串联腿构型的,0.16802f为水平轴到当前电机0点的偏移补偿

    //定义中间变量
    float A1;
    float A2;
    float B1;
    float B2;
    float delta;
    float K1;
    float K2;

    void VMC_leg_swing_torque_restriction();
    void Jacobian_Matrix_Calculate();

    //计算得到的关节力矩输出值
    float tau_1;
    float tau_2;
//TODO：明确对应的电机XXX：注意目前电机零点和解算零点的区别

};

#endif //H7_BOARD_VMC_H
