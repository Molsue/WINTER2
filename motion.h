#ifndef __MOTION_H__
#define __MOTION_H__

#include "main.h"

typedef struct
{
    float x;
    float y;
    float yaw;
} RobotPose_t;

typedef struct
{
    float x;
    float y;
    float v;      // 该点期望速度 (m/s)
    uint8_t stop; // 1 表示到达该点后停车
} PathPoint_t;

typedef struct
{
    float wheel_radius_m;      // 轮子半径 (可调)
    float wheel_base_m;        // 轮距 (可调)
    float lookahead_m;         // 前视距离 (可调)
    float max_linear_mps;      // 最大线速度
    float max_angular_rps;     // 最大角速度
    float pos_tolerance_m;     // 到点位置阈值
    float speed_ramp_mps2;     // 速度斜坡限制
} MotionConfig_t;

typedef enum
{
    MOTION_IDLE = 0,
    MOTION_RUNNING = 1,
    MOTION_REACHED = 2
} MotionStatus_t;

void Motion_Init(void);
void Motion_SetPath(const PathPoint_t *points, uint16_t count);
MotionStatus_t Motion_Update(float dt_s, float yaw_imu_rad);
RobotPose_t Motion_GetPose(void);
uint8_t Motion_IsStationary(void);

#endif /* __MOTION_H__ */
