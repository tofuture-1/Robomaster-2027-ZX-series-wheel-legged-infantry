//
// Created by 谭恩泽 on 2026/4/18.
//

#include "dm_imu.h"


/**
 * @brief 初始化达妙IMU对象，并建立角速度Kalman滤波器的初始状态。
 * @param hfdcan IMU所挂载的CAN总线句柄。
 * @param __Now_IMU_Status IMU初始在线状态。
 * @param __IMU_Method IMU通信方式。
 * @param _CAN_ID IMU命令发送CAN ID。
 * @param _Mst_ID IMU回传主机ID。
 */
void Class_DM_IMU::Init(FDCAN_HandleTypeDef *hfdcan,
                        Enum_IMU_Ststus __Now_IMU_Status,
                        Enum_IMU_Conmunicate_Method __IMU_Method,
                        uint8_t _CAN_ID, uint8_t _Mst_ID) {
  if (hfdcan->Instance == FDCAN1) {
    FDCAN_Manage_Object = &FDCAN1_Manage_Object;
  } else if (hfdcan->Instance == FDCAN2) {
    FDCAN_Manage_Object = &FDCAN2_Manage_Object;
  } else if (hfdcan->Instance == FDCAN3) {
    FDCAN_Manage_Object = &FDCAN3_Manage_Object;
  }
  Now_IMU_Ststus = __Now_IMU_Status;
  DM_IMU_Control_Method = __IMU_Method;

  // 默认欧拉角
  Set_Reg_ID(EULER_DATA);
  Set_CAN_ID(_CAN_ID);
  Set_MST_ID(_Mst_ID);

  // 初始化多圈变量
  Is_First_Loop = true;
  Yaw_Round_Count = 0;
  Total_Angle_Yaw = 0.0f;
  Pre_Angle_Yaw = 0.0f;

  Filter_Gyro = {0};
}

void Class_DM_IMU::DM_IMU_RequestData(FDCAN_HandleTypeDef *hfdcan,
                                      uint8_t REG_ID, uint8_t AC,
                                      uint32_t *Data) {
  FDCAN_TxHeaderTypeDef Txheader;
  uint8_t buf[8] = {0xCC, REG_ID, AC, 0xDD, 0, 0, 0, 0};
  memcpy(buf + 4, Data, 4);

  // 配置发送头
  Txheader.DataLength = 8;
  Txheader.IdType = FDCAN_STANDARD_ID;
  Txheader.TxFrameType = FDCAN_DATA_FRAME;
  Txheader.Identifier = this->CAN_ID;
  Txheader.FDFormat = FDCAN_CLASSIC_CAN;
  Txheader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  Txheader.BitRateSwitch = FDCAN_BRS_OFF;
  Txheader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

  Txheader.MessageMarker = 0x00;

  // 发送CAN报文
  HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &Txheader, buf);
}

void Class_DM_IMU::RTOS_1ms_IMU_Quest_Callback() {
  DM_IMU_RequestData(FDCAN_Manage_Object->FDCAN_Handler, IMU_REG_ID, CMD_READ,
                     0);
}

void Class_DM_IMU::DM_IMU_UpdataAccel(uint8_t *pData) {
  uint16_t accel[3];

  accel[0] = pData[3] << 8 | pData[2];
  accel[1] = pData[5] << 8 | pData[4];
  accel[2] = pData[7] << 8 | pData[6];

  Accelerate.Temperature = (float)pData[1];
  Accelerate.Accelerate_X =
      uint_to_float(accel[0], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
  Accelerate.Accelerate_Y =
      uint_to_float(accel[1], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
  Accelerate.Accelerate_Z =
      uint_to_float(accel[2], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
}

/**
 * @brief 解析达妙IMU陀螺仪数据，并同步更新Kalman滤波后的角速度。
 * @param pData IMU原始8字节CAN数据区指针。
 */
void Class_DM_IMU::DM_IMU_UpdateGyro(uint8_t *pData) {
  uint16_t gyro[3];

  gyro[0] = pData[3] << 8 | pData[2];
  gyro[1] = pData[5] << 8 | pData[4];
  gyro[2] = pData[7] << 8 | pData[6];

  Gyro.Omega_X = uint_to_float(gyro[0], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
  Gyro.Omega_Y = uint_to_float(gyro[1], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
  Gyro.Omega_Z = uint_to_float(gyro[2], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
}

void Class_DM_IMU::DM_IMU_UpdateEuler(uint8_t *pData) {
  int euler[3];

  euler[0] = pData[3] << 8 | pData[2];
  euler[1] = pData[5] << 8 | pData[4];
  euler[2] = pData[7] << 8 | pData[6];

  Euler_Angle.Angle_Pitch =
      uint_to_float(euler[0], PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
  // 这里获取的是单圈角度 [-PI, PI]
  float now_yaw = uint_to_float(euler[1], YAW_CAN_MIN, YAW_CAN_MAX, 16);

  // 多圈解算逻辑
  if (Is_First_Loop) {
    Pre_Angle_Yaw = now_yaw;
    Is_First_Loop = false;
    Yaw_Round_Count = 0;
  }

  Euler_Angle.Angle_Yaw = now_yaw;

  // 过零点检测
  if (now_yaw - Pre_Angle_Yaw < -180.0f) {
    // 从PI跳变到-PI，正转一圈
    Yaw_Round_Count++;
  } else if (now_yaw - Pre_Angle_Yaw > 180.0f) {
    // 从-PI跳变到PI，反转一圈
    Yaw_Round_Count--;
  }
  Pre_Angle_Yaw = now_yaw;

  // 计算连续的总角度
  Total_Angle_Yaw = now_yaw + (float)Yaw_Round_Count * 360.0f;

  Euler_Angle.Angle_Roll =
      uint_to_float(euler[2], ROLL_CAN_MIN, ROLL_CAN_MAX, 16);
}

void Class_DM_IMU::DM_IMU_UpdateQuaternion(uint8_t *pData) {

  int w = pData[1] << 6 | ((pData[2] & 0xF8) >> 2);
  int x = (pData[2] & 0x03) << 12 | (pData[3] << 4) | ((pData[4] & 0xF0) >> 4);
  int y = (pData[4] & 0x0F) << 10 | (pData[5] << 2) | (pData[6] & 0xC0) >> 6;
  int z = (pData[6] & 0x3F) << 8 | pData[7];

  Quaternion.Q_w = uint_to_float(w, Quaternion_MIN, Quaternion_MAX, 14);
  Quaternion.Q_x = uint_to_float(x, Quaternion_MIN, Quaternion_MAX, 14);
  Quaternion.Q_y = uint_to_float(y, Quaternion_MIN, Quaternion_MAX, 14);
  Quaternion.Q_z = uint_to_float(z, Quaternion_MIN, Quaternion_MAX, 14);
}

void Class_DM_IMU::Data_Process() {
  memcpy(Can_IMU_Original_Data, FDCAN_Manage_Object->Rx_Buffer.Data, 8);

  if (Can_IMU_Original_Data[0] == 1) {
    DM_IMU_UpdataAccel(Can_IMU_Original_Data);
  } else if (Can_IMU_Original_Data[0] == 2) {
    DM_IMU_UpdateGyro(Can_IMU_Original_Data);
  } else if (Can_IMU_Original_Data[0] == 3) {
    DM_IMU_UpdateEuler(Can_IMU_Original_Data);
  } else if (Can_IMU_Original_Data[0] == 4) {
    DM_IMU_UpdateQuaternion(Can_IMU_Original_Data);
  }
}

void Class_DM_IMU::CAN_RxCpltCallback(uint8_t *Rx_Data) {
  Flag += 1;

  Data_Process();
}

void Class_DM_IMU::RTOS_100ms_IMU_Alive_Callback() {
  // 判断该时间段内是否接收过电机数据
  if (Flag == Pre_Flag) {
    // 电机断开连接
    Now_IMU_Ststus = IMU_Status_DISABLE;
  } else {
    // 电机保持连接
    Now_IMU_Ststus = IMU_Status_ENABLE;
  }
  Pre_Flag = Flag;
}
