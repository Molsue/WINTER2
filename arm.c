#include "arm.h"
#include "tim.h"
#include "cmsis_os.h"

#define SERVO_MIN_PULSE_US  500.0f
#define SERVO_MAX_PULSE_US  2500.0f
#define SERVO_PERIOD_US     20000.0f

static uint16_t Servo_DegToCompare(float deg)
{
    if (deg < 0.0f) deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;

    float pulse_us = SERVO_MIN_PULSE_US + (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * (deg / 180.0f);
    float duty = pulse_us / SERVO_PERIOD_US;
    return (uint16_t)(duty * (float)(__HAL_TIM_GET_AUTORELOAD(&htim8) + 1));
}

void Arm_Init(void)
{
    // 舵机 PWM 已在 main.c 中启动
    ArmPose_t home = {
        .clamp_deg = 250.0f,
        .horiz_deg = 90.0f,
        .vert_deg = 90.0f,
        .base_deg = 90.0f
    };
    Arm_SetPose(&home);
}

void Arm_SetPose(const ArmPose_t *pose)
{
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, Servo_DegToCompare(pose->clamp_deg));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, Servo_DegToCompare(pose->horiz_deg));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, Servo_DegToCompare(pose->vert_deg));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, Servo_DegToCompare(pose->base_deg));
}

void Arm_Sequence_Pick(void)
{
    // 夹爪拾取动作：需要根据你的机械结构调参
    ArmPose_t step1 = { .clamp_deg = 30.0f, .horiz_deg = 90.0f, .vert_deg = 120.0f, .base_deg = 90.0f };
    ArmPose_t step2 = { .clamp_deg = 30.0f, .horiz_deg = 110.0f, .vert_deg = 120.0f, .base_deg = 90.0f };
    ArmPose_t step3 = { .clamp_deg = 90.0f, .horiz_deg = 110.0f, .vert_deg = 120.0f, .base_deg = 90.0f };
    ArmPose_t step4 = { .clamp_deg = 90.0f, .horiz_deg = 90.0f, .vert_deg = 90.0f, .base_deg = 90.0f };

    Arm_SetPose(&step1);
    osDelay(400);
    Arm_SetPose(&step2);
    osDelay(400);
    Arm_SetPose(&step3);
    osDelay(400);
    Arm_SetPose(&step4);
    osDelay(400);
}

void Arm_Sequence_Place(void)
{
    // 放置动作：需要根据你的机械结构调参
    ArmPose_t step1 = { .clamp_deg = 90.0f, .horiz_deg = 120.0f, .vert_deg = 110.0f, .base_deg = 90.0f };
    ArmPose_t step2 = { .clamp_deg = 30.0f, .horiz_deg = 120.0f, .vert_deg = 110.0f, .base_deg = 90.0f };
    ArmPose_t step3 = { .clamp_deg = 30.0f, .horiz_deg = 90.0f, .vert_deg = 90.0f, .base_deg = 90.0f };

    Arm_SetPose(&step1);
    osDelay(400);
    Arm_SetPose(&step2);
    osDelay(400);
    Arm_SetPose(&step3);
    osDelay(400);
}

void Arm_Sequence_Stack(void)
{
    // 堆叠动作：需要根据你的机械结构调参
    ArmPose_t step1 = { .clamp_deg = 90.0f, .horiz_deg = 120.0f, .vert_deg = 130.0f, .base_deg = 90.0f };
    ArmPose_t step2 = { .clamp_deg = 30.0f, .horiz_deg = 120.0f, .vert_deg = 130.0f, .base_deg = 90.0f };
    ArmPose_t step3 = { .clamp_deg = 30.0f, .horiz_deg = 90.0f, .vert_deg = 90.0f, .base_deg = 90.0f };

    Arm_SetPose(&step1);
    osDelay(400);
    Arm_SetPose(&step2);
    osDelay(400);
    Arm_SetPose(&step3);
    osDelay(400);
}
