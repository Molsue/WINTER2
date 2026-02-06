#ifndef __IMU_H__
#define __IMU_H__

#include "main.h"

typedef struct
{
    float yaw_rad;         // 融合后的航向角 (rad)
    float gyro_z_dps;      // 原始陀螺Z轴角速度 (deg/s)
    float gyro_bias_dps;   // 陀螺零偏估计
} IMU_State_t;

void IMU_Init(void);
void IMU_Update(float dt_s, uint8_t is_stationary);
IMU_State_t IMU_GetState(void);

#endif /* __IMU_H__ */
