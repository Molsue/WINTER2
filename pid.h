#ifndef __PID_CTRL_H
#define __PID_CTRL_H

#include "main.h"
#include "arm_math.h"

typedef struct {
    // DSP库自带的PID实例结构体
    arm_pid_instance_f32 pid_instance;

    // 控制变量
    float target_speed;    // 目标速度 (RPM)
    float current_speed;   // 当前反馈速度 (RPM)
    float output_pwm;      // PID输出PWM值 (-limit~+limit)

    // 额外参数（可调）
    float output_limit;    // PWM输出限幅 (如 7199)
    float integral_limit;  // 积分限幅 (防止饱和积累)
    float speed_lpf_alpha; // 速度低通系数 0~1 (越大越敏感)
    float deadband_rpm;    // 速度死区 (RPM)
    float speed_filtered;  // 速度低通后的值
} Motor_Ctrl_t;

// 定义左右两个电机对象
extern Motor_Ctrl_t motor_L;
extern Motor_Ctrl_t motor_R;

// 供外部调用的函数
void Motor_PID_Init(void);
void Motor_Loop_10ms(void); // 放在定时器或FreeRTOS任务里调用

// 转换参数
// --- 电机物理参数配置 ---
#define ENCODER_PPR  13.0f   // 编码器线数
#define GEAR_RATIO   20.0f   // 减速比
#define ENCODER_MULT 4.0f    // 定时器倍频 (TI1+TI2模式为4)
#define CONTROL_DT   0.01f   // 控制周期 10ms = 0.01s

// 一圈对应的总脉冲数 = 13 * 20 * 4 = 1040
#define PULSE_PER_ROUND (ENCODER_PPR * GEAR_RATIO * ENCODER_MULT)

// 转换系数：将 (脉冲/dt) 转换为 RPM
// RPM = count / PULSE_PER_ROUND * 60 / dt
#define PULSE_TO_RPM_FACTOR (60.0f / (PULSE_PER_ROUND * CONTROL_DT))

#endif
