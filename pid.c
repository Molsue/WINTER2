#include "pid.h"
#include "motor.h"
#include "tim.h"
#include <math.h>

Motor_Ctrl_t motor_L;
Motor_Ctrl_t motor_R;

static float PID_Compute(Motor_Ctrl_t *motor, float error)
{
    // 调用 DSP PID 计算输出
    float output = arm_pid_f32(&motor->pid_instance, error);

    // 积分限幅（防止累加失控）
    if (motor->pid_instance.state[0] > motor->integral_limit) {
        motor->pid_instance.state[0] = motor->integral_limit;
    } else if (motor->pid_instance.state[0] < -motor->integral_limit) {
        motor->pid_instance.state[0] = -motor->integral_limit;
    }

    // 输出限幅 + 反向修正（简单抗积分饱和）
    if (output > motor->output_limit) {
        float excess = output - motor->output_limit;
        output = motor->output_limit;
        motor->pid_instance.state[0] -= excess;
    } else if (output < -motor->output_limit) {
        float excess = output + motor->output_limit;
        output = -motor->output_limit;
        motor->pid_instance.state[0] -= excess;
    }

    return output;
}

/**
  * @brief  初始化PID参数
  */
void Motor_PID_Init(void)
{
    // --- 左电机 PID 参数配置 ---
    motor_L.pid_instance.Kp = 15.0f;  // 比例系数 (可调)
    motor_L.pid_instance.Ki = 0.0f;   // 积分系数 (可调)
    motor_L.pid_instance.Kd = 0.0f;   // 微分系数 (速度环通常为0)

    motor_L.output_limit = 7199.0f;   // PWM限幅 (与ARR一致)
    motor_L.integral_limit = 3000.0f; // 积分限幅 (可调)
    motor_L.speed_lpf_alpha = 0.3f;   // 速度低通 (可调)
    motor_L.deadband_rpm = 0.0f;      // 死区 (可调)
    motor_L.speed_filtered = 0.0f;

    // 初始化 DSP PID 内部结构 (清除历史误差)
    arm_pid_init_f32(&motor_L.pid_instance, 1);

    // --- 右电机 PID 参数配置 ---
    motor_R.pid_instance.Kp = 15.0f;
    motor_R.pid_instance.Ki = 0.0f;
    motor_R.pid_instance.Kd = 0.0f;

    motor_R.output_limit = 7199.0f;
    motor_R.integral_limit = 3000.0f;
    motor_R.speed_lpf_alpha = 0.3f;
    motor_R.deadband_rpm = 0.0f;
    motor_R.speed_filtered = 0.0f;

    arm_pid_init_f32(&motor_R.pid_instance, 1);
}

/**
  * @brief  周期性控制任务 (建议 10ms 调用一次)
  */
void Motor_Loop_10ms(void)
{
    // 1. 读取编码器实际速度（RPM）
    int16_t raw_pulse_L = Encoder_Reader(1);
    int16_t raw_pulse_R = Encoder_Reader(2);

    float speed_L = raw_pulse_L * PULSE_TO_RPM_FACTOR;
    float speed_R = raw_pulse_R * PULSE_TO_RPM_FACTOR;

    // 2. 速度低通滤波（减少抖动）
    motor_L.speed_filtered = motor_L.speed_lpf_alpha * speed_L
                             + (1.0f - motor_L.speed_lpf_alpha) * motor_L.speed_filtered;
    motor_R.speed_filtered = motor_R.speed_lpf_alpha * speed_R
                             + (1.0f - motor_R.speed_lpf_alpha) * motor_R.speed_filtered;

    motor_L.current_speed = motor_L.speed_filtered;
    motor_R.current_speed = motor_R.speed_filtered;

    // 3. 目标接近0时，复位积分，避免输出漂移
    if (fabsf(motor_L.target_speed) < motor_L.deadband_rpm) {
        arm_pid_reset_f32(&motor_L.pid_instance);
        motor_L.output_pwm = 0.0f;
    }

    if (fabsf(motor_R.target_speed) < motor_R.deadband_rpm) {
        arm_pid_reset_f32(&motor_R.pid_instance);
        motor_R.output_pwm = 0.0f;
    }

    // 4. 计算误差 = 目标 - 实际
    float error_L = motor_L.target_speed - motor_L.current_speed;
    float error_R = motor_R.target_speed - motor_R.current_speed;

    // 5. PID 输出计算 + 限幅
    if (fabsf(motor_L.target_speed) >= motor_L.deadband_rpm) {
        motor_L.output_pwm = PID_Compute(&motor_L, error_L);
    }

    if (fabsf(motor_R.target_speed) >= motor_R.deadband_rpm) {
        motor_R.output_pwm = PID_Compute(&motor_R, error_R);
    }

    // 6. 数据发送
    // ---------------- 处理左电机 ----------------
    uint8_t dir_L = (motor_L.output_pwm >= 0) ? 1 : 0;
    int32_t speed_pwm_L = (int32_t)fabsf(motor_L.output_pwm);

    if (speed_pwm_L > (int32_t)motor_L.output_limit) speed_pwm_L = (int32_t)motor_L.output_limit;
    Motor_Control(1, speed_pwm_L, dir_L);

    // ---------------- 处理右电机 ----------------
    uint8_t dir_R = (motor_R.output_pwm >= 0) ? 1 : 0;
    int32_t speed_pwm_R = (int32_t)fabsf(motor_R.output_pwm);

    if (speed_pwm_R > (int32_t)motor_R.output_limit) speed_pwm_R = (int32_t)motor_R.output_limit;
    Motor_Control(2, speed_pwm_R, dir_R);
}
