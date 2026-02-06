
#include "tim.h"
#include "motor.h"


/**
 * @brief  电机控制函数
 * @param  motor_id:  选择电机 (MOTOR_L（1） 或 MOTOR_R（2）)
 * @param  speed:     PWM占空比数值 (0 - 7199)
 * @param  direction: 旋转方向 (1=正转, 0=反转)
 */
void Motor_Control(uint8_t motor_id, int32_t speed, uint8_t direction)
{
    // 1. 安全限幅：防止PWM值超过定时器周期的最大值
    if (speed > 7199) speed = 7199;
    if (speed < 0)    speed = 0;

    // 2. 根据 motor_id 选择控制逻辑
    if (motor_id == MOTOR_L)
    {
        /* --- 控制马达 Left (PB12, PB13, TIM1_CH1) --- */
        
        // 设置方向 
        if (direction == Forward) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        } else if (direction == Reverse) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        } else {
					  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
				}
        
        // 设置速度
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
    }
    else if (motor_id == MOTOR_R)
    {
        /* --- 控制马达 Right (PB14, PB15, TIM1_CH2) --- */
        
        // 设置方向
        if (direction == Forward) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        } else if (direction == Reverse) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        } else {
					  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
				}
        
        // 设置速度
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
    }
}



/**
 * @brief  编码器读取函数
 * @param  motor_id:  选择电机 (MOTOR_L（1） 或 MOTOR_R（2）)
 * @param  speed:     计数器值CNT指针读数（int16有正负）
 * @param  return:    CNT值
 */
int16_t Encoder_Reader(uint8_t motor_id)
{
    int16_t speed;

    switch(motor_id)
    {
        case 1: // 处理左电机 (TIM3)
        {
            // 1. 读取计数器值，强制转换为 int16_t 以利用溢出特性处理负数
            speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
            
            // 2. 软件方向修正 
            speed = -speed; 

            // 3. 清零计数器，准备下一次测量 (M法测速)
            __HAL_TIM_SET_COUNTER(&htim3, 0);
            
            break;
        }

        case 2: // 处理右电机 (TIM4)
        {
            speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
            
            // 2. 软件方向修正 
            // speed = -speed;

            __HAL_TIM_SET_COUNTER(&htim4, 0);
            
            break;
        }

        default:
            return 0; // ID 错误返回 0
    }

    return speed;
}
