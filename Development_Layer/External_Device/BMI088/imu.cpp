#include "imu.h"
#include <cstring>


/**
 * @brief IMU初始化
 */
void Class_Board_IMU::Init(SPI_HandleTypeDef *hspi)
{
    this->hspi = hspi;
    this->dt = 0.001f;
    BMI088_Init(hspi);

    // 原参数 R=1e7，加速度修正增益过小，三轴积分漂移；调小 R 增强重力修正
    // IMU_QuaternionEKF_Init(10.0f, 0.001f, 10000000.0f, 1.0f, 0.0f);
    IMU_QuaternionEKF_Init(81.5f, 0.0010f, 1295000.0f, 1.0f, 0.000f);
    INS.AccelLPF = 0.0085f;

    offset_flag = 1;

    // // 原固定零偏（手动标定值），与器件/温度偏差时会导致三轴积分漂移
    // bmi088_data.GyroOffset[0] = 0.003191390214f;
    // bmi088_data.GyroOffset[1] = -0.00133049982f;
    // bmi088_data.GyroOffset[2] = -0.003365303324f;
    // bmi088_data.AccelScale = 1.0f;
    // 开机自动标定：机器人需静置约 2 秒，自动更新零偏与 AccelScale
    BMI088_Calibrate(&bmi088_data, hspi, 2000);

    IMU_PID_Heater.Init(2.0f,0.3f,2.0f,0.067f,0.0f,5.0f,20.0f,0.1,0.125f);


}

/**
 * @brief 姿态解算更新
 */
void Class_Board_IMU::Update(void)
{
    const float gravity[3] = {0.0f, 0.0f, 9.81f};
    const float xb[3] = {1.0f, 0.0f, 0.0f};
    const float yb[3] = {0.0f, 1.0f, 0.0f};
    const float zb[3] = {0.0f, 0.0f, 1.0f};

    INS.Accel[IMU_X] = bmi088_data.Accel[IMU_X] * bmi088_data.AccelScale;
    INS.Accel[IMU_Y] = bmi088_data.Accel[IMU_Y] * bmi088_data.AccelScale;
    INS.Accel[IMU_Z] = bmi088_data.Accel[IMU_Z] * bmi088_data.AccelScale;
    // 读取原始陀螺仪数据后，在类内进行零偏补偿
    INS.Gyro[IMU_X] = bmi088_data.Gyro[IMU_X] - bmi088_data.GyroOffset[IMU_X];
    INS.Gyro[IMU_Y] = bmi088_data.Gyro[IMU_Y] - bmi088_data.GyroOffset[IMU_Y];
    INS.Gyro[IMU_Z] = bmi088_data.Gyro[IMU_Z] - bmi088_data.GyroOffset[IMU_Z];

    INS.Temperature = bmi088_data.Temp;
    // 更新静态检测
    Update_Stationary_Detection();

    IMU_QuaternionEKF_Update(INS.Gyro[IMU_X], INS.Gyro[IMU_Y], INS.Gyro[IMU_Z],
                              INS.Accel[IMU_X], INS.Accel[IMU_Y], INS.Accel[IMU_Z], dt);
    memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

    // 更新陀螺仪零偏估计
    Update_Gyro_Bias_Estimator();

    BodyFrameToEarthFrame(xb, INS.xn, INS.q);
    BodyFrameToEarthFrame(yb, INS.yn, INS.q);
    BodyFrameToEarthFrame(zb, INS.zn, INS.q);

    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
    for (uint8_t i = 0; i < 3; i++)
    {
        INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt)
                                + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
    }
    BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q);

    INS.Yaw = QEKF_INS.Yaw;
    INS.Roll = QEKF_INS.Roll;
    INS.Pitch = QEKF_INS.Pitch;//直连
    INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
}


void Class_Board_IMU::IMU_Heater_SetPower(uint8_t percentage)
{

    if (percentage >100) percentage = 100;
    if (percentage < 0) percentage = 0;//做上下限制

    uint32_t compare_value = (uint32_t)((float)percentage / 100.0f * 2750.0f);

    if (compare_value > 2749) compare_value = 2749;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, compare_value);
}

void Class_Board_IMU::IMU_Heater_Control(uint16_t Target_temp)
{
    IMU_PID_Heater.Set_Target(Target_temp);
    IMU_PID_Heater.Set_Now(Get_Temp());

    IMU_PID_Heater.PID_Process();


    if (Get_Temp() < IMU_HEARTER_MAX_TEMPERATURE )
    {
        IMU_Heater_SetPower(IMU_PID_Heater.Get_Out());
        if (Get_Temp() >= Target_temp )
        {
            IMU_PID_Heater.Set_Integral_Error(0);//到达目标后清空积分误差，避免积分负饱和
        }
    }else
    {
        IMU_Heater_SetPower(0);//多一层过温保护

    }
    //封装好并实现相对稳定的控制算法2609302010

}
/**
 * @brief IMU采样任务，读取传感器原始数据
 */
void Class_Board_IMU::RTOS_IMU_1ms_Sampling_Task(void)
{
    BMI088_Read_IMU(hspi, &bmi088_data);
}

/**
 * @brief EKF更新回调函数
 */
void Class_Board_IMU::RTOS_IMU_1ms_EKF_Callback(void)
{
    Update();
}

/**
 * @brief 计算三维向量模
 */
float Class_Board_IMU::Norm3(const float *v)
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/**
 * @brief 限制数值范围
 */
void Class_Board_IMU::Math_Constrain(float *val, float min, float max)
{
    if (*val < min) *val = min;
    else if (*val > max) *val = max;
}

/**
 * @brief 静态检测更新
 * 检测IMU是否处于静止状态，包含进入/退出确认机制
 */
void Class_Board_IMU::Update_Stationary_Detection(void)
{
     gyro_norm = Norm3(INS.Gyro);
     accel_norm = Norm3(INS.Accel);
     accel_error = fabsf(accel_norm - gravity_ref);

    const bool gyro_stationary = (gyro_norm < stationary_gyro_enter_threshold);
    const bool accel_stationary = (accel_error < stationary_accel_enter_threshold);
    const bool enter_condition = gyro_stationary && accel_stationary;

    const bool gyro_exit = (gyro_norm > stationary_gyro_exit_threshold);
    const bool accel_exit = (accel_error > stationary_accel_exit_threshold);
    const bool exit_condition = gyro_exit || accel_exit;

    if (!is_stationary)
    {
        // 非静止状态，判断是否进入静止状态
        if (enter_condition)
        {
            if (stationary_enter_counter < stationary_enter_confirm_count)
            {
                stationary_enter_counter++;
            }
            if (stationary_enter_counter >= stationary_enter_confirm_count)
            {
                is_stationary = true;
                stationary_exit_counter = 0U;
            }
        }
        else
        {
            stationary_enter_counter = 0U;
        }
    }
    else
        {
            // 静止状态，判断是否退出静止状态
            if (exit_condition)
            {
                if (stationary_exit_counter < stationary_exit_confirm_count)
                {
                    stationary_exit_counter++;
                }
                if (stationary_exit_counter >= stationary_exit_confirm_count)
                {
                    is_stationary = false;
                    stationary_exit_counter = 0U;
                }
            }
            else
            {
                stationary_exit_counter = 0U;
            }
        }
}

/**
 * @brief 陀螺仪零偏估计器更新
 * 静态时自动校准陀螺仪零偏，使用低通滤波形式直接估计零偏
 */
void Class_Board_IMU::Update_Gyro_Bias_Estimator(void)
{
    if (!QEKF_INS.ConvergeFlag || !is_stationary)
    {
        return;
    }

    const float measured_bias[3] = {
        bmi088_data.Gyro[IMU_X],
        bmi088_data.Gyro[IMU_Y],
        bmi088_data.Gyro[IMU_Z],
    };

    // 如果测量值异常，重置零偏
    if (Norm3(measured_bias) > gyro_bias_update_norm_threshold)
    {
            gyro_bias_init_count = 0U;
            gyro_bias_acc_x = 0.0f;
            gyro_bias_acc_y = 0.0f;
            gyro_bias_acc_z = 0.0f;
        return;
    }

    // 初始阶段：累加采样
    if (gyro_bias_init_count < gyro_bias_init_count_target)
    {
        gyro_bias_acc_x += measured_bias[0];
        gyro_bias_acc_y += measured_bias[1];
        gyro_bias_acc_z += measured_bias[2];
        gyro_bias_init_count++;
        if (gyro_bias_init_count >= gyro_bias_init_count_target)
        {
            // 计算平均残余零偏并约束
            float avg_bias_x = gyro_bias_acc_x / (float)gyro_bias_init_count_target;
            float avg_bias_y = gyro_bias_acc_y / (float)gyro_bias_init_count_target;
            float avg_bias_z = gyro_bias_acc_z / (float)gyro_bias_init_count_target;

            Math_Constrain(&avg_bias_x, -gyro_bias_max, gyro_bias_max);
            Math_Constrain(&avg_bias_y, -gyro_bias_max, gyro_bias_max);
            Math_Constrain(&avg_bias_z, -gyro_bias_max, gyro_bias_max);

            // 残余零偏累加到GyroOffset
            bmi088_data.GyroOffset[IMU_X] = avg_bias_x;
            bmi088_data.GyroOffset[IMU_Y] = avg_bias_y;
            bmi088_data.GyroOffset[IMU_Z] = avg_bias_z;

            // 限制零偏范围
            Math_Constrain(&bmi088_data.GyroOffset[IMU_X], -gyro_bias_max, gyro_bias_max);
            Math_Constrain(&bmi088_data.GyroOffset[IMU_Y], -gyro_bias_max, gyro_bias_max);
            Math_Constrain(&bmi088_data.GyroOffset[IMU_Z], -gyro_bias_max, gyro_bias_max);

            // 重置累加器
            gyro_bias_init_count = 0U;
            gyro_bias_acc_x = 0.0f;
            gyro_bias_acc_y = 0.0f;
            gyro_bias_acc_z = 0.0f;
        }
        return;
    }

    // 稳态阶段：低通滤波估计零偏
    for (uint8_t axis = 0; axis < 3U; axis++)
    {
        bmi088_data.GyroOffset[axis] = gyro_bias_stationary_alpha * measured_bias[axis] + 
                                        (1.0f - gyro_bias_stationary_alpha) * bmi088_data.GyroOffset[axis];
        Math_Constrain(&bmi088_data.GyroOffset[axis], -gyro_bias_max, gyro_bias_max);
    }
}

/**
 * @brief 机体坐标转换为大地坐标
 * @param vecBF 机体系向量
 * @param vecEF 导航系向量
 * @param q 四元数
 */
void Class_Board_IMU::BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief 大地坐标转换为机体坐标
 * @param vecEF 导航系向量
 * @param vecBF 机体系向量
 * @param q 四元数
 */
void Class_Board_IMU::EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

