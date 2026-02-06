/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "motion.h"
#include "imu.h"
#include "arm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//typedef struct {
//    volatile float target_speed_L; // ����Ŀ�� RPM
//    volatile float target_speed_R; // ����Ŀ�� RPM
//    // ������������չ״̬��float current_angle; ��
//} Robot_State_t;

//Robot_State_t g_robot = {0.0f, 0.0f}; // ��ʼ��Ϊ0
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Task_ChassisHandle;
osThreadId Task_INSHandle;
osThreadId Task_RobotHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartChassisTask(void const * argument);
void StartINSTask(void const * argument);
void StartRobotTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Task_Chassis */
  osThreadDef(Task_Chassis, StartChassisTask, osPriorityRealtime, 0, 512);
  Task_ChassisHandle = osThreadCreate(osThread(Task_Chassis), NULL);

  /* definition and creation of Task_INS */
  osThreadDef(Task_INS, StartINSTask, osPriorityAboveNormal, 0, 512);
  Task_INSHandle = osThreadCreate(osThread(Task_INS), NULL);

  /* definition and creation of Task_Robot */
  osThreadDef(Task_Robot, StartRobotTask, osPriorityNormal, 0, 512);
  Task_RobotHandle = osThreadCreate(osThread(Task_Robot), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
* @brief Function implementing the Task_Chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void const * argument)
{
  /* USER CODE BEGIN StartChassisTask */
  // 1. ��ʼ�� PID ��������ʷ����
  Motor_PID_Init();


  // 2. ׼��������ʱ (��֤�ϸ�� 100Hz Ƶ��)
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 10; // 10ms
  xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
//    // --- Step A: ��ȫ�ֱ�����ȡĿ���ٶ� ---
//    // ����ʵ���ˡ����ԡ������ȡ��Ĵ���
//    motor_L.target_speed = g_robot.target_speed_L;
//    motor_R.target_speed = g_robot.target_speed_R;

    // --- Step B: ִ�� PID ���ļ��� ---
    // �������������ˣ��������� -> ��PID -> ���PWM
    Motor_Loop_10ms(); 


    // --- Step C: ������ʱ ---
    // ��ʹ����ļ��㻨�� 2ms������Ҳֻ����ʱ 8ms��ȷ���������� 10ms
    osDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartINSTask */
/**
* @brief Function implementing the Task_INS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartINSTask */
void StartINSTask(void const * argument)
{
  /* USER CODE BEGIN StartINSTask */
   IMU_Init();

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 5; // 5ms (200Hz)

  for(;;)
  {
    IMU_Update(0.005f, Motion_IsStationary());
    osDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartINSTask */
}

/* USER CODE BEGIN Header_StartRobotTask */
/**
* @brief Function implementing the Task_Robot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRobotTask */
void StartRobotTask(void const * argument)
{
  /* USER CODE BEGIN StartRobotTask */

  // --- 轨迹点 (单位: 米) ---
  // 可调：通过修改点坐标与每个点的速度，实现圆滑轨迹
  static const PathPoint_t path_to_pick1[] = {
      {0.0f, 0.0f, 0.20f, 0},
      {0.4f, 0.0f, 0.25f, 0},
      {0.8f, 0.2f, 0.20f, 1},
  };

  static const PathPoint_t path_to_place1[] = {
      {0.8f, 0.2f, 0.20f, 0},
      {0.4f, 0.4f, 0.25f, 0},
      {0.0f, 0.6f, 0.20f, 1},
  };

  static const PathPoint_t path_to_pick2[] = {
      {0.0f, 0.6f, 0.25f, 0},
      {0.4f, 0.8f, 0.25f, 0},
      {0.8f, 0.6f, 0.20f, 1},
  };

  static const PathPoint_t path_to_stack[] = {
      {0.8f, 0.6f, 0.20f, 0},
      {0.4f, 0.4f, 0.20f, 0},
      {0.0f, 0.6f, 0.15f, 1},
  };

  typedef enum
  {
    ROBOT_TO_PICK1 = 0,
    ROBOT_PICK1,
    ROBOT_TO_PLACE1,
    ROBOT_PLACE1,
    ROBOT_TO_PICK2,
    ROBOT_PICK2,
    ROBOT_TO_STACK,
    ROBOT_STACK,
    ROBOT_DONE
  } RobotState_t;

  Motion_Init();
  Arm_Init();

  RobotState_t state = ROBOT_TO_PICK1;
  uint8_t state_enter = 1;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 20; // 20ms

  for(;;)
  {
    IMU_State_t imu = IMU_GetState();

    switch (state)
    {
      case ROBOT_TO_PICK1:
				
        if (state_enter)
        {
          Motion_SetPath(path_to_pick1, sizeof(path_to_pick1) / sizeof(path_to_pick1[0]));
          state_enter = 0;
        }
        if (Motion_Update(0.02f, imu.yaw_rad) == MOTION_REACHED)
        {
          state = ROBOT_PICK1;
          state_enter = 1;
        }
        break;

      case ROBOT_PICK1:
        motor_L.target_speed = 0.0f;
        motor_R.target_speed = 0.0f;
        Arm_Sequence_Pick();
        state = ROBOT_TO_PLACE1;
        state_enter = 1;
        break;

      case ROBOT_TO_PLACE1:
        if (state_enter)
        {
          Motion_SetPath(path_to_place1, sizeof(path_to_place1) / sizeof(path_to_place1[0]));
          state_enter = 0;
        }
        if (Motion_Update(0.02f, imu.yaw_rad) == MOTION_REACHED)
        {
          state = ROBOT_PLACE1;
          state_enter = 1;
        }
        break;

      case ROBOT_PLACE1:
        motor_L.target_speed = 0.0f;
        motor_R.target_speed = 0.0f;
        Arm_Sequence_Place();
        state = ROBOT_TO_PICK2;
        state_enter = 1;
        break;

      case ROBOT_TO_PICK2:
        if (state_enter)
        {
          Motion_SetPath(path_to_pick2, sizeof(path_to_pick2) / sizeof(path_to_pick2[0]));
          state_enter = 0;
        }
        if (Motion_Update(0.02f, imu.yaw_rad) == MOTION_REACHED)
        {
          state = ROBOT_PICK2;
          state_enter = 1;
        }
        break;

      case ROBOT_PICK2:
        motor_L.target_speed = 0.0f;
        motor_R.target_speed = 0.0f;
        Arm_Sequence_Pick();
        state = ROBOT_TO_STACK;
        state_enter = 1;
        break;

      case ROBOT_TO_STACK:
        if (state_enter)
        {
          Motion_SetPath(path_to_stack, sizeof(path_to_stack) / sizeof(path_to_stack[0]));
          state_enter = 0;
        }
        if (Motion_Update(0.02f, imu.yaw_rad) == MOTION_REACHED)
        {
          state = ROBOT_STACK;
          state_enter = 1;
        }
        break;

      case ROBOT_STACK:
        motor_L.target_speed = 0.0f;
        motor_R.target_speed = 0.0f;
        Arm_Sequence_Stack();
        state = ROBOT_DONE;
        state_enter = 1;
        break;

      case ROBOT_DONE:
      default:
        motor_L.target_speed = 0.0f;
        motor_R.target_speed = 0.0f;
        break;
    }

    osDelayUntil(&xLastWakeTime, xFrequency);
  }

  /* USER CODE END StartRobotTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

