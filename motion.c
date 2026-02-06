#include "motion.h"
#include "pid.h"
#include "arm_math.h"
#include <math.h>

typedef struct
{
    MotionConfig_t cfg;
    RobotPose_t pose;
    float yaw_odom;
    float target_v;
    float target_w;
    float filtered_v;
    const PathPoint_t *path;
    uint16_t path_count;
    uint16_t path_index;
    MotionStatus_t status;
    arm_pid_instance_f32 yaw_pid;
} MotionState_t;

static MotionState_t g_motion;

static float WrapAngle(float rad)
{
    if (rad > PI)  rad -= 2.0f * PI;
    if (rad < -PI) rad += 2.0f * PI;
    return rad;
}

static float Clamp(float v, float min_v, float max_v)
{
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static float RPM_To_MPS(float rpm, float wheel_radius_m)
{
    return (rpm / 60.0f) * 2.0f * PI * wheel_radius_m;
}

static float MPS_To_RPM(float v_mps, float wheel_radius_m)
{
    return (v_mps / (2.0f * PI * wheel_radius_m)) * 60.0f;
}

static void Motion_ResetPose(void)
{
    g_motion.pose.x = 0.0f;
    g_motion.pose.y = 0.0f;
    g_motion.pose.yaw = 0.0f;
    g_motion.yaw_odom = 0.0f;
}

void Motion_Init(void)
{
    g_motion.cfg.wheel_radius_m = 0.024f;   // 轮子半径 (可调)
    g_motion.cfg.wheel_base_m = 0.132f;      // 轮距 (可调)
    g_motion.cfg.lookahead_m = 0.20f;       // 前视距离 (可调)
    g_motion.cfg.max_linear_mps = 0.4f;     // 最大线速度
    g_motion.cfg.max_angular_rps = 2.5f;    // 最大角速度
    g_motion.cfg.pos_tolerance_m = 0.05f;   // 到点判定阈值
    g_motion.cfg.speed_ramp_mps2 = 0.5f;    // 速度斜坡限制

    g_motion.yaw_pid.Kp = 2.0f;  // 航向修正 PID (可调)
    g_motion.yaw_pid.Ki = 0.0f;
    g_motion.yaw_pid.Kd = 0.0f;
    arm_pid_init_f32(&g_motion.yaw_pid, 1);

    g_motion.path = NULL;
    g_motion.path_count = 0;
    g_motion.path_index = 0;
    g_motion.status = MOTION_IDLE;
    g_motion.target_v = 0.0f;
    g_motion.target_w = 0.0f;
    g_motion.filtered_v = 0.0f;

    Motion_ResetPose();
}

void Motion_SetPath(const PathPoint_t *points, uint16_t count)
{
    g_motion.path = points;
    g_motion.path_count = count;
    g_motion.path_index = 0;
    g_motion.status = (count > 0) ? MOTION_RUNNING : MOTION_IDLE;
    g_motion.target_v = 0.0f;
    g_motion.target_w = 0.0f;
    g_motion.filtered_v = 0.0f;
}

MotionStatus_t Motion_Update(float dt_s, float yaw_imu_rad)
{
    if (g_motion.status == MOTION_IDLE || g_motion.path == NULL)
    {
        motor_L.target_speed = 0.0f;
        motor_R.target_speed = 0.0f;
        return g_motion.status;
    }

    // 使用电机实际速度更新里程计
    float v_l = RPM_To_MPS(motor_L.current_speed, g_motion.cfg.wheel_radius_m);
    float v_r = RPM_To_MPS(motor_R.current_speed, g_motion.cfg.wheel_radius_m);
    float v = 0.5f * (v_l + v_r);
    float w_odom = (v_r - v_l) / g_motion.cfg.wheel_base_m;

    g_motion.yaw_odom = WrapAngle(g_motion.yaw_odom + w_odom * dt_s);

    // 互补融合：陀螺仪为主，轮速为辅，降低漂移
    const float alpha = 0.98f;
    g_motion.pose.yaw = WrapAngle(alpha * yaw_imu_rad + (1.0f - alpha) * g_motion.yaw_odom);

    g_motion.pose.x += v * arm_cos_f32(g_motion.pose.yaw) * dt_s;
    g_motion.pose.y += v * arm_sin_f32(g_motion.pose.yaw) * dt_s;

    // --- 轨迹跟踪 ---
    PathPoint_t target = g_motion.path[g_motion.path_index];
    float dx = target.x - g_motion.pose.x;
    float dy = target.y - g_motion.pose.y;
    float dist = 0.0f;
    arm_sqrt_f32(dx * dx + dy * dy, &dist);

    if (dist < g_motion.cfg.pos_tolerance_m)
    {
        if (target.stop || g_motion.path_index >= g_motion.path_count - 1)
        {
            g_motion.status = MOTION_REACHED;
            motor_L.target_speed = 0.0f;
            motor_R.target_speed = 0.0f;
            return g_motion.status;
        }
        g_motion.path_index++;
        target = g_motion.path[g_motion.path_index];
        dx = target.x - g_motion.pose.x;
        dy = target.y - g_motion.pose.y;
        arm_sqrt_f32(dx * dx + dy * dy, &dist);
    }

    float lookahead = g_motion.cfg.lookahead_m;
    if (dist < lookahead)
    {
        lookahead = dist + 1e-3f;
    }

    float target_angle = atan2f(dy, dx);
    float alpha_angle = WrapAngle(target_angle - g_motion.pose.yaw);
    float curvature = 2.0f * arm_sin_f32(alpha_angle) / lookahead;

    float v_cmd = Clamp(target.v, -g_motion.cfg.max_linear_mps, g_motion.cfg.max_linear_mps);
    float w_cmd = curvature * v_cmd;

    // 航向误差 PID 微调，减少累计误差
    float yaw_error = alpha_angle;
    w_cmd += arm_pid_f32(&g_motion.yaw_pid, yaw_error);

    w_cmd = Clamp(w_cmd, -g_motion.cfg.max_angular_rps, g_motion.cfg.max_angular_rps);

    // 速度斜坡限制，避免加速过猛
    float v_diff = v_cmd - g_motion.filtered_v;
    float v_step = g_motion.cfg.speed_ramp_mps2 * dt_s;
    g_motion.filtered_v += Clamp(v_diff, -v_step, v_step);

    float v_left = g_motion.filtered_v - w_cmd * g_motion.cfg.wheel_base_m * 0.5f;
    float v_right = g_motion.filtered_v + w_cmd * g_motion.cfg.wheel_base_m * 0.5f;

    motor_L.target_speed = MPS_To_RPM(v_left, g_motion.cfg.wheel_radius_m);
    motor_R.target_speed = MPS_To_RPM(v_right, g_motion.cfg.wheel_radius_m);

    return g_motion.status;
}

RobotPose_t Motion_GetPose(void)
{
    return g_motion.pose;
}

uint8_t Motion_IsStationary(void)
{
    float v_l = fabsf(motor_L.current_speed);
    float v_r = fabsf(motor_R.current_speed);
    return (v_l < 1.0f && v_r < 1.0f);
}
