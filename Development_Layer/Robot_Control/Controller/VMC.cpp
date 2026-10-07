//
// Created by 24158 on 2026/10/5.
//

#include "VMC.h"


void Class_VMC::Init()
{

}

void Class_VMC::Jacobian_Matrix_Calculate()
{
    //计算当前腿长
    L = sqrt(x_d * x_d + y_d * y_d);


    A1 = x_d + l1*cos(theta1);
    A2 = x_d - l1*cos(theta2);
    B1 = y_d + l1*sin(theta1);
    B2 = y_d - l1*sin(theta2);

    delta = A1*B2 - A2*B1;

    K1 = A1*sin(theta1) - B1*cos(theta1) ;
    K2 = - A2*sin(theta2) + B2*cos(theta2) ;


    Jacobian_Matrix[0][0] = (l1*K1*(x_d*B2 - y_d*A2))/(delta * L);
    Jacobian_Matrix[0][1] = (l1*K2*(y_d*A1 - x_d*B1))/(delta * L);

    Jacobian_Matrix[1][0] = (l1*K1*(y_d*B2 + x_d*A2))/(delta * L * L);
    Jacobian_Matrix[1][1] = (l1*K2*(y_d*B1 + x_d*A1))/(delta * L * L);

}

void Class_VMC::VMC_Motor_tau_map(float F_L,float tau_leg)
{
    tau_1 = Jacobian_Matrix[0][0] * F_L + Jacobian_Matrix[1][0] * tau_leg;
    tau_2 = Jacobian_Matrix[0][1] * F_L + Jacobian_Matrix[1][1] * tau_leg;


}