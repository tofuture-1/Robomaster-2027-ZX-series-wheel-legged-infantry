//
// Created by 谭恩泽 on 2025/10/23.
//

#ifndef C_BOARD_LQR_H
#define C_BOARD_LQR_H


#define R_w 0.058f
#include "drv_math.h"

class Class_LQR
{
public:
//HACK：单位归一化的问题
  void Set_wheel_S(float wheel_S){p = wheel_S;};//HACK:临时清0用
  void Set_wheel_speed(float wheel_speed){p_dot = wheel_speed * R_w;};//HACK:临时用轮速换算车速
  void Set_theta(float theta_Pitch){theta = (theta_Pitch-180.0f) * 0.0174532f;};//3.1415926f/180.0f = 0.0174532f换算成弧度//HACK：角度对重心的补偿在K计算输出处实现了
  void Set_theta_dot(float theta_Pitch_dot){theta_dot = theta_Pitch_dot;};
  void Set_phi(float leg_phi){phi = leg_phi;}//HACK：注意补偿值方向，另外回位情况与QR矩阵权重有关，补偿-7.5度：此处+ 0.087222f*1.5
  void Set_phi_dot(float leg_phi_dot){phi_dot = leg_phi_dot;}

  void Set_p_dot_fine(float p_dot_f){p_dot_fine = p_dot_f;}//设置前后速度补偿;

  float Get_tau_w(){return tau_w;};
  float Get_tau_leg(){return tau_leg;};

  void LQR_Calculate();

protected:


  float p=0;//整车水平位置
  float p_dot=0;//整车水平速度
  float theta = 0;//车身俯仰角：目前IMU后倾时Pitch角减小，前倾（电池架下沉）增大
  float theta_dot=0;//车身俯仰角速度
  float phi = 0;//腿等效倾角
  float phi_dot=0;//腿等效角速度
  float D_t = 0.001f;//p整车水平位置积分的计算周期HACK：此部分应该参考PID.cpp中的Init进行初始化

  float theta_fine = 0.9f * 0.0174532f;//Pitch倾角对重心的补偿，-x * 0.0174532f表示向后倾x°为目标值,此项结合K矩阵中乘的符号决定
  float phi_fine = 0.0f;//利用倾角差值做摆角补偿，-x表示向后摆x rad,此项结合K矩阵中乘的符号决定

  float p_dot_fine = 0.0f;


  // float K[2][6] = {
  //   {-2.7676f, -11.3241f, -116.3097f, -10.1232f, 42.3300f, 1.7182f},
  //   {1.2491f, 5.1428f, 67.4651f, 6.0892f, 46.6200f, 2.5693f}
  // };相对可以的一版 v0.1

  // float K[2][6] = {
  //   {-7.5143f, -15.2116f, -52.2882f, -9.8807f, 46.1258f, 2.0814f},
  //   {3.5268f, 7.2811f, 27.8786f, 5.3189f, 23.2288f, 1.6173f}
  // };//0.17m腿长时用

  float K[2][6] = {
    {-7.0993f, -13.8329f, -50.0524f, -8.1849f, 62.7009f, 4.9869f},
    {3.3200f, 6.5570f, 25.4386f, 4.2249f, 25.3344f, 2.6726f}
  };//0.25m腿长时 v0.2_0.25m

  //TODO:完成变腿长LQR增益矩阵计算


  //261109车体后倾时IMU读出的Pitch轴角度减小
  float tau_w = 0.0f;
  float tau_leg = 0.0f;




};
//Matlab中使用的初版参数
// %% ------------------------- 1. 物理参数 -------------------------
// % 根据实际机器人修改这些值
// M       = 17.368;     % 车身质量 [kg]  check 16.7纯底盘+0.668的电池
// m_w     = 1.100;      % 轮子质量 [kg] check单个0.550kg
// m_leg   = 1.3256;      % 腿的等效质量 [kg] 单腿貌似0.6628kg check？
// I_c     = 0.29319283;      % 车身绕质心转动惯量 [kg·m^2] check
// I_leg   = 0.001726;     % 腿绕自身质心转动惯量 [kg·m^2] 暂时直接等效成直杆计算
// J_w     = 0.0037004;     % 轮子转动惯量 [kg·m^2] check  用上述双轮质量等效成均匀圆柱计算
// l_c     = 0.30;     % 车身质心到轮轴距离 [m] 简化
// L       = 0.25;     % 等效连杆长度（髋关节到轮轴）[m] 暂定
// r       = 0.058;     % 轮子半径 [m] check
// g       = 9.81;     % 重力加速度 [m/s^2]
//
// l_leg   = L / 2;    % 腿等效质心到轮轴距离 [m]



#endif //C_BOARD_LQR_H

