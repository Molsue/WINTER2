#include "imu.h"
#include "i2c.h"
#include "arm_math.h"

#define MPU6050_ADDR            (0x68 << 1)
#define MPU6050_REG_PWR_MGMT1   0x6B
#define MPU6050_REG_GYRO_CFG    0x1B
#define MPU6050_REG_ACCEL_CFG   0x1C
#define MPU6050_REG_GYRO_ZH     0x47

#define GYRO_SCALE_DPS_PER_LSB  131.0f   // ±250dps
#define GYRO_BIAS_LPF_ALPHA     0.01f    // 低通更新系数

static IMU_State_t g_imu = {0};

static int16_t IMU_ReadGyroZRaw(void)
{
    uint8_t buf[2] = {0};
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_REG_GYRO_ZH, I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
    return (int16_t)((buf[0] << 8) | buf[1]);
}

void IMU_Init(void)
{
    uint8_t data = 0;

    // 唤醒 MPU6050
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_REG_PWR_MGMT1, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // 设置陀螺仪量程 ±250dps
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_REG_GYRO_CFG, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // 设置加速度量程 ±2g (这里不直接使用加速度，只保留默认)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_REG_ACCEL_CFG, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // 若你使用 INT 引脚，可在这里初始化 EXTI
    // EXTI + MPU6050 数据就绪中断示例（需在 CubeMX 配置引脚）：
    // HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    // HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void IMU_Update(float dt_s, uint8_t is_stationary)
{
    int16_t gz_raw = IMU_ReadGyroZRaw();
    g_imu.gyro_z_dps = (float)gz_raw / GYRO_SCALE_DPS_PER_LSB;

    if (is_stationary)
    {
        // 估计陀螺零偏，减少积分漂移
        g_imu.gyro_bias_dps = (1.0f - GYRO_BIAS_LPF_ALPHA) * g_imu.gyro_bias_dps
                            + GYRO_BIAS_LPF_ALPHA * g_imu.gyro_z_dps;
    }

    float gyro_dps_corrected = g_imu.gyro_z_dps - g_imu.gyro_bias_dps;
    float gyro_rad_s = gyro_dps_corrected * (PI / 180.0f);

    g_imu.yaw_rad += gyro_rad_s * dt_s;

    if (g_imu.yaw_rad > PI)  g_imu.yaw_rad -= 2.0f * PI;
    if (g_imu.yaw_rad < -PI) g_imu.yaw_rad += 2.0f * PI;
}

IMU_State_t IMU_GetState(void)
{
    return g_imu;
}
