//
// Created by 24158 on 2026/10/5.
//

#ifndef H7_BOARD_VMC_H
#define H7_BOARD_VMC_H

#include <math.h>

#define l1 0.21 //单位：米

#define l2 0.25

class Class_VMC
{
public:
    void Init();

    void VMC_Motor_tau_map(float F_L,float tau_leg);

    void Set_x_d(float x){x_d = x;};
    void Set_y_d(float y){y_d = y;};

    float Get_Motor_tau1(){return tau_1;};
    float Get_Motor_tau2(){return tau_2;};


    protected:
    float Jacobian_Matrix[2][2];

    //足端轮轴坐标——以髋关节为原点建系
    float x_d;
    float y_d;
    float L;//等效腿长——髋关节和轮轴连杆长度


    float theta1;
    float theta2;
//HACK:貌似可以直接获取这里的角度，然后算出x和y？

    //定义中间变量
    float A1;
    float A2;
    float B1;
    float B2;
    float delta;
    float K1;
    float K2;

    void Jacobian_Matrix_Calculate();

    //计算得到的关节力矩输出值
    float tau_1;
    float tau_2;
//TODO：明确对应的电机XXX：注意目前电机零点和解算零点的区别

};

#endif //H7_BOARD_VMC_H
