//
// Created by 24158 on 2026/10/5.
//

#ifndef H7_BOARD_VMC_H
#define H7_BOARD_VMC_H

#include <math.h>


#define l1 0.21f //单位：米

#define l2 0.25f


enum Enum_Foot_Control_Mode
{
    VMC_Mode = 0,
    LQR_Mode,
};

class Class_VMC
{
public:




    void Init(float i_L_des = 0.25f,float i_K_L = 650.0f,float i_D_L = 150.5f,
        float i_G_Fine = 85.0f,
        float i_K_pe = 80.0f,float i_D_pe = 5.0f,
        Enum_Foot_Control_Mode Control_mode= LQR_Mode,float i_K_p_VMC = 80.0f,float i_D_p_VMC = 5.0f);//此处值基于纯17.8kg底盘进行初始化

    void VMC_Motor_tau_map();

    void Kinematic_Solute();

    void VMC_Calculate();




    void Set_theta1(float theta){theta1 = theta ;};
    void Set_theta2(float theta){theta2 = theta ;};
    void Set_L_des(float L){L_des = L;};
    void Set_theta1_dot(float theta){theta1_dot = theta;};
    void Set_theta2_dot(float theta){theta2_dot = theta;};
    void Set_leg_swing_torque_LQR(float swing_torque){leg_swing_torque_LQR = swing_torque;};
    void Set_delta_leg_rl_phi(float delta){delta_leg_rl_phi = delta;};
    void Set_delta_leg_rl_phi_dot(float delta){delta_leg_rl_phi_dot = delta;};
    void Set_phi_des_VMC_Mode(float phi){phi_des_VMC_Mode = phi;};

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
    float Get_phi_dot(){return phi_dot;};
    float Get_delta_leg_rl_phi(){return delta_leg_rl_phi;};
    float Get_delta_leg_rl_phi_dot(){return delta_leg_rl_phi_dot;};
    float Get_leg_swing_torque_VMC(){return leg_swing_torque_VMC;};
    float Get_leg_swing_torque_delta_fine(){return leg_swing_torque_delta_fine;};
    float Get_leg_swing_torque_LQR(){return leg_swing_torque_LQR;};

    float Get_Motor_tau1(){return tau_1;};
    float Get_Motor_tau2(){return tau_2;};


    protected://HACK：以下变量的类型区别需要再进行区分
    float Jacobian_Matrix[2][2] = {0};




    //***传入的***//
    float theta1 = 0.0f;//需要以传入的是零点为x负轴，逆时针为正的真实杆物理角（补偿后）
    float theta2 = 0.0f ;//零点为x正轴，逆时针为正
    float theta_fine = 0.16802f;//目前串联腿构型的,0.16802f为水平轴到当前电机0点的偏移补偿
    float theta1_dot = 0.0f;//关节电机角速度
    float theta2_dot = 0.0f;

    float phi_des_VMC_Mode = 0.0f;//用于VMC控制摆角


    //须初始化项//
    //腿长虚拟力计算
    float L_des = 0.25f;//目标腿长,单位是米 最短时为0.105m，最长时0.33m左右
    float K_L = 650.0f;//弹簧项
    float D_L = 150.5f;//阻尼项
    float G_Fine = 85.0f;//重力前馈TODO：是否需要考虑再不同倾角下的力矩输出？HACK：可以暂时直接利用cos映射一下分量XXX：注意做离地检测防止离地时堵转

    float K_pe = 80.0f;
    float D_pe = 5.0f;
    //两腿差角补偿KD项

    float K_p_VMC = 80.0f;
    float D_p_VMC = 5.0f;
    Enum_Foot_Control_Mode Foot_Control_Mode = LQR_Mode;
    ////

    //***内部算出的***//
    float L_dot = 0.0f;//等效腿长变化速率
    float phi_dot = 0.0f;
    float delta_leg_rl_phi = 0.0f;//左右腿倾角差值，实现两腿严格对称用
    float delta_leg_rl_phi_dot = 0.0f;

    //足端轮轴坐标——以髋关节为原点建系
    float x_d = 0.0f;
    float y_d = -L_des;
    float L = L_des;//等效腿长——髋关节和轮轴连杆长度
    float phi = 0.0f;//等效倾角（等效腿与y轴负半轴夹角）
    //初始化值防止乱飞

    //注意这是单条腿的
    float F_L = 0.0f;//虚拟腿长力
    float leg_swing_torque_delta_fine = 0.0f;
    //左右phi角差补偿力计算

    float leg_swing_torque_LQR = 0.0f;//等效摆腿力矩
    //目前实测力矩在5.0f左右就可以实现轮抬到到接近水平位置
    //TODO：同时方向为顺时针为负，逆时针为正，修改最终力输出时J[1][x]项的符号以匹配？？

    float leg_swing_torque_VMC = 0.0f;//用于VMC控制摆角

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
