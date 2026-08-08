//
// Created by 谭恩泽 on 2026/4/18.
//

#ifndef H7_DEMO_DM_IMU_H
#define H7_DEMO_DM_IMU_H

#include "drv_fdcan.h"
#include "drv_math.h"
#include "imu.h"

/**
 * @brief IMU状态枚举
 */
enum Enum_IMU_Ststus
{
    IMU_Status_DISABLE = 0,
    IMU_Status_ENABLE = 1,
};

/**
 * @brief 四元数数据结构
 */
struct Struct_IMU_Data_Quaternion
{
    float Q_w;
    float Q_x;
    float Q_y;
    float Q_z;
};

/**
 * @brief 加速度计数据结构
 */
struct Struct_IMU_Data_Accelerate
{
    float Accelerate_X;
    float Accelerate_Y;
    float Accelerate_Z;
    float Temperature;
};

/**
 * @brief 陀螺仪数据结构
 */
struct Struct_IMU_Data_Gyro
{
    float Omega_X;
    float Omega_Y;
    float Omega_Z;
};

/**
 * @brief 欧拉角数据结构
 */
struct Struct_IMU_Data_Euler_Angle
{
    float Angle_Roll;
    float Angle_Pitch;
    float Angle_Yaw;
};

enum Reg_Id
{
     REBOOT_IMU=0,
     ACCEL_DATA,
     GYRO_DATA,
     EULER_DATA,
     QUAT_DATA,
     SET_ZERO,
     ACCEL_CALI,
     GYRO_CALI,
     MAG_CALI,
     CHANGE_COM,
     SET_DELAY,
     CHANGE_ACTIVE,
     SET_BAUD,
     SET_CAN_ID,
     SET_MST_ID,
     DATA_OUTPUT_SELECTION,
     SAVE_PARAM=254,
     RESTORE_SETTING=255
};

enum Enum_IMU_Conmunicate_Method
{
    DM_IMU_Control_RS485 = 0,
    DM_IMU_Control_USB,
    DM_IMU_Control_CAN,
    C_Board_IMU_Control_SPI,
};

class Class_DM_IMU
{

public:

     /**
      * @brief 初始化达妙IMU通信对象，并完成角速度Kalman滤波器的初始配置。
      * @param hfdcan IMU所在CAN总线句柄。
      * @param __Now_IMU_Status IMU初始在线状态。
      * @param __IMU_Method IMU通信方式。
      * @param _CAN_ID IMU命令发送CAN ID。
      * @param _Mst_ID IMU回传主机ID。
      */
     void Init(FDCAN_HandleTypeDef *hfdcan, Enum_IMU_Ststus __Now_IMU_Status, Enum_IMU_Conmunicate_Method __IMU_Method ,uint8_t _CAN_ID,uint8_t _Mst_ID);

     void DM_IMU_RequestData (FDCAN_HandleTypeDef *hfdcan , uint8_t REG_ID ,uint8_t AC , uint32_t *Data);

          // 多圈相关变量
     float Total_Angle_Yaw = 0.0f;

     float Pre_Angle_Yaw = 0.0f;

     int32_t Yaw_Round_Count = 0;

     bool Is_First_Loop = true;

     inline Enum_IMU_Ststus Get_Status();

     inline float Get_Accelerate_X();

     inline float Get_Accelerate_Y();

     inline float Get_Accelerate_Z();

     inline float Get_Accelerate_Temperature();

     inline float Get_Omega_X();

     inline float Get_Omega_Y();

     inline float Get_Omega_Z();

     /**
      * @brief 获取X轴Kalman滤波后的角速度。
      * @return float X轴滤波后角速度，单位与原始角速度保持一致。
      */
     inline float Get_Filter_Omega_X();

     /**
      * @brief 获取Y轴Kalman滤波后的角速度。
      * @return float Y轴滤波后角速度，单位与原始角速度保持一致。
      */
     inline float Get_Filter_Omega_Y();

     /**
      * @brief 获取Z轴Kalman滤波后的角速度。
      * @return float Z轴滤波后角速度，单位与原始角速度保持一致。
      */
     inline float Get_Filter_Omega_Z();

     inline float Get_Angle_Roll();

     inline float Get_Angle_Pitch();

     inline float Get_Angle_Yaw();

     inline float Get_Total_Angle_Yaw();

     inline float Get_Q_0();

     inline float Get_Q_1();

     inline float Get_Q_2();

     inline float Get_Q_3();

     inline void Set_Reg_ID(Reg_Id ID);

     inline void Set_CAN_ID(uint8_t _CAN_ID);

     inline void Set_MST_ID(uint8_t _MST_ID);

     void RTOS_1ms_IMU_Quest_Callback();

     void RTOS_100ms_IMU_Alive_Callback();

     void CAN_RxCpltCallback(uint8_t *Rx_Data);
private:

     //变量
     // 绑定的CAN
     Struct_FDCAN_Manage_Object *FDCAN_Manage_Object;
     //使能
     Enum_IMU_Ststus Now_IMU_Ststus;
    //通信方式
     Enum_IMU_Conmunicate_Method DM_IMU_Control_Method;
    //四元数
     Struct_IMU_Data_Quaternion Quaternion = {0};
    //加速度计
     Struct_IMU_Data_Accelerate Accelerate = {0};
    //加速度计
     Struct_IMU_Data_Gyro Gyro = {0};
     //Kalman滤波后的角速度
     Struct_IMU_Data_Gyro Filter_Gyro = {0};
     //欧拉角
     Struct_IMU_Data_Euler_Angle Euler_Angle = {0};


     uint8_t Can_IMU_Original_Data[8] = {0};
     //达妙IMU
     const float ACCEL_CAN_MAX = 235.2f;
     const float ACCEL_CAN_MIN = -235.2f;
     const float GYRO_CAN_MAX	= 34.88f;
     const float GYRO_CAN_MIN	= -34.88f;
     const float PITCH_CAN_MAX = 90.0f;
     const float PITCH_CAN_MIN = -90.0f;
     const float ROLL_CAN_MAX	= 180.0f;
     const float ROLL_CAN_MIN	= -180.0f;
     const float YAW_CAN_MAX = 180.0f;
     const float  YAW_CAN_MIN = -180.0f;
     const float TEMP_MIN = 0.0f;
     const float TEMP_MAX = 60.0f;
     const float Quaternion_MIN = -1.0f;
     const float Quaternion_MAX = 1.0f;

     //寄存器id
     Reg_Id IMU_REG_ID;
     //接收标志
     uint32_t Flag = 0;
     //上次接收标志
     uint32_t Pre_Flag = 0;
     //可以更改IMU的can总线ID
     uint8_t CAN_ID = 0x00;
     uint8_t MST_ID = 0x00;

     const uint8_t CMD_READ = 0;

     const uint8_t CMD_WRITE = 1;
     //方法
     void Data_Process();

     void DM_IMU_UpdateQuaternion(uint8_t* pData);

     void DM_IMU_UpdateGyro(uint8_t* pData);

     void DM_IMU_UpdateEuler(uint8_t* pData);

     void DM_IMU_UpdataAccel(uint8_t* pData);
};

inline void Class_DM_IMU::Set_Reg_ID(Reg_Id ID)
{
    IMU_REG_ID = ID;
}

inline Enum_IMU_Ststus Class_DM_IMU::Get_Status()
{
    return (Now_IMU_Ststus);
}

inline float Class_DM_IMU::Get_Accelerate_X()
{
    return (Accelerate.Accelerate_X);
}

inline float Class_DM_IMU::Get_Accelerate_Y()
{
    return (Accelerate.Accelerate_Y);
}

inline float Class_DM_IMU::Get_Accelerate_Z()
{
    return (Accelerate.Accelerate_Z);
}

inline float Class_DM_IMU::Get_Accelerate_Temperature()
{
    return (Accelerate.Temperature);
}

inline float Class_DM_IMU::Get_Omega_X()
{
    return (Gyro.Omega_X);
}

inline float Class_DM_IMU::Get_Omega_Y()
{
    return (Gyro.Omega_Y);
}

inline float Class_DM_IMU::Get_Omega_Z()
{
    return (Gyro.Omega_Z);
}

inline float Class_DM_IMU::Get_Angle_Roll()
{
    return (Euler_Angle.Angle_Roll);
}

inline float Class_DM_IMU::Get_Angle_Pitch()
{
    return (Euler_Angle.Angle_Pitch);
}

inline float Class_DM_IMU::Get_Angle_Yaw()
{
    return (Euler_Angle.Angle_Yaw);
}

inline float Class_DM_IMU::Get_Total_Angle_Yaw()
{
    return Total_Angle_Yaw;
}

inline float Class_DM_IMU::Get_Q_0()
{
    return (Quaternion.Q_w);
}

inline float Class_DM_IMU::Get_Q_1()
{
    return (Quaternion.Q_x);
}

inline float Class_DM_IMU::Get_Q_2()
{
    return (Quaternion.Q_y);
}

inline float Class_DM_IMU::Get_Q_3()
{
    return (Quaternion.Q_z);
}

inline void Class_DM_IMU::Set_CAN_ID(uint8_t _CAN_ID)
{
    this->CAN_ID = _CAN_ID;
}

inline void Class_DM_IMU::Set_MST_ID(uint8_t _Mst_ID)
{
    this->MST_ID = _Mst_ID;
}


#endif //H7_DEMO_DM_IMU_H
