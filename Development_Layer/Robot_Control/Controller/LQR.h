//
// Created by 谭恩泽 on 2025/10/23.
//

#ifndef C_BOARD_LQR_H
#define C_BOARD_LQR_H


class Class_LQR
{
public:

protected:


  float p=0;//整车水平位置
  float p_dot=0;//整车水平速度
  float theta = 0;//车身俯仰角：目前IMU后倾时Pitch角减小，前倾（电池架下沉）增大
  float theta_dot=0;//车身俯仰角速度
  float phi = 0;//腿等效倾角
  float phi_dot=0;//腿等效角速度

  float K[2][6] = {
    {     -7.3096,     -16.8168,    -128.7754,     -30.5374,       0.9836,       0.8053 },
    {     -1.2070,      -2.7738,     -24.3661,      -6.1989,      23.7104,      11.3937 }
  };

  float tau_w;
  float tau_leg;

  void LQR_Calculate();


};



#endif //C_BOARD_LQR_H

