#ifndef __MOTOR_H__
#define __MOTOR_H__

// 必须包含 main.h，否则 uint8_t 和 HAL 库宏无法识别
#include "main.h" 

//定义马达
#define MOTOR_L  1  // 对应 PB12/PB13 + TIM1_CH1 左侧马达（BlN）+TIM3（编码器）
#define MOTOR_R  2  // 对应 PB14/PB15 + TIM1_CH2 右侧马达（AlN）+TIM4（编码器）

// 定义方向
#define Forward  1  // 正转 (Forward)
#define Reverse  0  // 反转 (Reverse)

// 函数声明
void Motor_Control(uint8_t motor_id, int32_t speed, uint8_t direction);
int16_t Encoder_Reader(uint8_t motor_id);

#endif /* __MOTOR_H__ */
