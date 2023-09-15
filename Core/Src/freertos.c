/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usart.h"
#include <stdbool.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/Twist.h>
#include <diagnostic_msgs/msg/diagnostic_array.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
/* USER CODE BEGIN PTD */

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
/* Definitions for microROS */
osThreadId_t microROSHandle;
const osThreadAttr_t microROS_attributes = {
  .name = "microROS",
  .stack_size = 2000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RobotProcess */
osThreadId_t RobotProcessHandle;
uint32_t RobotProcessBuffer[ 128 ];
osStaticThreadDef_t RobotProcessControlBlock;
const osThreadAttr_t RobotProcess_attributes = {
  .name = "RobotProcess",
  .cb_mem = &RobotProcessControlBlock,
  .cb_size = sizeof(RobotProcessControlBlock),
  .stack_mem = &RobotProcessBuffer[0],
  .stack_size = sizeof(RobotProcessBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for RestartTask */
osThreadId_t RestartTaskHandle;
uint32_t RestartTaskBuffer[ 128 ];
osStaticThreadDef_t RestartTaskControlBlock;
const osThreadAttr_t RestartTask_attributes = {
  .name = "RestartTask",
  .cb_mem = &RestartTaskControlBlock,
  .cb_size = sizeof(RestartTaskControlBlock),
  .stack_mem = &RestartTaskBuffer[0],
  .stack_size = sizeof(RestartTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Synchronize_flag */
osMutexId_t Synchronize_flagHandle;
osStaticMutexDef_t Synchronize_flagControlBlock;
const osMutexAttr_t Synchronize_flag_attributes = {
  .name = "Synchronize_flag",
  .cb_mem = &Synchronize_flagControlBlock,
  .cb_size = sizeof(Synchronize_flagControlBlock),
};
/* Definitions for RestartSemaphore */
osSemaphoreId_t RestartSemaphoreHandle;
osStaticSemaphoreDef_t myBinarySem01ControlBlock;
const osSemaphoreAttr_t RestartSemaphore_attributes = {
  .name = "RestartSemaphore",
  .cb_mem = &myBinarySem01ControlBlock,
  .cb_size = sizeof(myBinarySem01ControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void ROS_init(void);

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END FunctionPrototypes */

void microROSTask(void *argument);
void Robot_processTask(void *argument);
void Restart_processTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of Synchronize_flag */
  Synchronize_flagHandle = osMutexNew(&Synchronize_flag_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of RestartSemaphore */
  RestartSemaphoreHandle = osSemaphoreNew(1, 1, &RestartSemaphore_attributes);

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
  /* creation of microROS */
  microROSHandle = osThreadNew(microROSTask, NULL, &microROS_attributes);

  /* creation of RobotProcess */
  RobotProcessHandle = osThreadNew(Robot_processTask, NULL, &RobotProcess_attributes);

  /* creation of RestartTask */
  RestartTaskHandle = osThreadNew(Restart_processTask, NULL, &RestartTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_microROSTask */
static rcl_publisher_t diagnostic_publisher;
static rcl_subscription_t geometry_subscriber;
/**
  * @brief  Function implementing the microROS thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_microROSTask */
void microROSTask(void *argument)
{
  /* USER CODE BEGIN microROSTask */
  /* Infinite loop */
	ROS_init();

	  // micro-ROS app

	  rclc_support_t support;
	  rcl_allocator_t allocator;
	  rcl_node_t node;

	  allocator = rcl_get_default_allocator();

	  //create init_options
	  rclc_support_init(&support, 0, NULL, &allocator);

	  // create node
	  rclc_node_init_default(&node, "CrawlerBot_node", "", &support);

	  // create publisher
	  rclc_publisher_init_default(
	    &diagnostic_publisher,
	    &node,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray),
	    "CrawlerRobot/diagnostic");

	  for(;;)
	  {
	    rcl_ret_t ret = rcl_publish(&diagnostic_publisher, &diagnostic_publisher, NULL);
	    if (ret != RCL_RET_OK)
	    {
	      printf("Error publishing (line %d)\n", __LINE__);
	    }

	    osDelay(10);
	  }
  /* USER CODE END microROSTask */
}

/* USER CODE BEGIN Header_Robot_processTask */
/**
* @brief Function implementing the RobotProcess thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Robot_processTask */
void Robot_processTask(void *argument)
{
  /* USER CODE BEGIN Robot_processTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Robot_processTask */
}

/* USER CODE BEGIN Header_Restart_processTask */
/**
* @brief Function implementing the RestartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Restart_processTask */
void Restart_processTask(void *argument)
{
  /* USER CODE BEGIN Restart_processTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Restart_processTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void ROS_init(void)
{
	  rmw_uros_set_custom_transport(
	    true,
	    (void *) &huart1,
	    cubemx_transport_open,
	    cubemx_transport_close,
	    cubemx_transport_write,
	    cubemx_transport_read);

	  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	  freeRTOS_allocator.allocate = microros_allocate;
	  freeRTOS_allocator.deallocate = microros_deallocate;
	  freeRTOS_allocator.reallocate = microros_reallocate;
	  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	      printf("Error on default allocators (line %d)\n", __LINE__);
	  }
}
/* USER CODE END Application */

