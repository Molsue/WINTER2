#ifndef __ARM_CTRL_H__
#define __ARM_CTRL_H__

#include "main.h"

typedef struct
{
    float clamp_deg;     // 夹爪开合角度
    float horiz_deg;     // 水平运动舵机角度
    float vert_deg;      // 垂直运动舵机角度
    float base_deg;      // 底座旋转角度
} ArmPose_t;

void Arm_Init(void);
void Arm_SetPose(const ArmPose_t *pose);
void Arm_Sequence_Pick(void);
void Arm_Sequence_Place(void);
void Arm_Sequence_Stack(void);

#endif /* __ARM_CTRL_H__ */
