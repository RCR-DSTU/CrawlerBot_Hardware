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
#include <nav_msgs/msg/odometry.h>

#include "kinematics.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static struct {
	/*---------------------ROBOT-GENERAL-------------------------*/
	uint8_t start_flag;
	float target_left_crawler_speed;
	float target_right_crawler_speed;
	float target_skis_angle;
	Kinematic_t kinematic;
	Engine_t engines;
	/*---------------------ROBOT-SENSORS-------------------------*/
	Encoder_t left_crawler_encoder;
	Encoder_t right_crawler_encoder;
	Encoder_t skis_crawler_encoder;
	/*---------------------ROBOT-ERROR-FLAGS---------------------*/
	uint8_t error_count;
	uint8_t emergency_flag;
	/*---------------------ROBOT-PID-----------------------------*/
	Regulator_t regulator;
	/*---------------------ROS TIME SYNC-------------------------*/
	int64_t time_ms;
	int64_t time_ns;
	/*---------------------ROS OBJECTS---------------------------*/
	rcl_allocator_t allocator;
	rcl_publisher_t diagnostic_publisher;
	rcl_subscription_t geometry_subscriber;
	rcl_publisher_t odometry_publisher;
	rclc_support_t support;
	rcl_node_t node;
	rclc_executor_t executor;
	/*---------------------MESSAGES---------------------------*/
	nav_msgs__msg__Odometry odometry_msg;
	diagnostic_msgs__msg__DiagnosticArray diagnostic_msg;
	geometry_msgs__msg__Twist geometry_msg;
	/*--------------------ROS OTHER OBJECTS-------------------*/
	rcl_ret_t ret;
}robot;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){robot.start_flag = 0; return false;}}
#define MICRO_ROS_ALLOCATE	do { \
		rmw_uros_set_custom_transport(\
			true,\
			(void *) &huart1,\
			cubemx_transport_open,\
			cubemx_transport_close,\
			cubemx_transport_write,\
			cubemx_transport_read);\
		rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();\
		freeRTOS_allocator.allocate = microros_allocate;\
		freeRTOS_allocator.deallocate = microros_deallocate;\
		freeRTOS_allocator.reallocate = microros_reallocate;\
		freeRTOS_allocator.zero_allocate =  microros_zero_allocate;\
		if (!rcutils_set_default_allocator(&freeRTOS_allocator)) { robot.start_flag = 0; xTaskNotifyGive(RestartTaskHandle); }\
} while(0)
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
const osThreadAttr_t RobotProcess_attributes = {
  .name = "RobotProcess",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RestartTask */
osThreadId_t RestartTaskHandle;
const osThreadAttr_t RestartTask_attributes = {
  .name = "RestartTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Synchronize_flag */
osMutexId_t Synchronize_flagHandle;
const osMutexAttr_t Synchronize_flag_attributes = {
  .name = "Synchronize_flag"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void regulator_init(void);
static void kinematic_init(void);
static void engines_init(void);
static void sensors_init(void);
static void regulator_calc(PID_parameters_t *param);
static void regulator_get_data(PID_parameters_t *param);
static void regulator_on(PID_parameters_t *pid);
static void regulator_off(PID_parameters_t *pid);
static void set_voltage(Engine_parameters_t engine, double duty);
static bool ros_init(void);
static void ros_deinit(void);
static void ros_process(void);
static void ros_sync(void);
static void ros_diagnostic_process(void);

static void subscription_callback(const void* msgin);

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

static inline void general_init(void)
{
	regulator_init();
	kinematic_init();
	engines_init();
	sensors_init();
}
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
  general_init();
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_microROSTask */
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
	MICRO_ROS_ALLOCATE;
	bool state = ros_init();
	if(state != true) xTaskNotifyGive(RestartTaskHandle);
	/*!
	 * @info ROS application:
	 * 	1) Ping agent
	 * 	2)
	 */
	for(;;)
	{
		if(robot.start_flag == 1)
		{
			ros_sync();
			ros_process();
			ros_diagnostic_process();
		} else xTaskNotifyGive(RestartTaskHandle);
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
	 if(robot.start_flag == 1)
	 {
		 robot.kinematic.inverse(robot.kinematic.crawler);
	 }
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
	ulTaskNotifyTake(1, portMAX_DELAY);
	if(robot.start_flag == 0)
	{
		HAL_NVIC_SystemReset();
	}
  }
  /* USER CODE END Restart_processTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void regulator_init(void)
{
	robot.regulator.speed_left_regulator.p_k = 1.0;
	robot.regulator.speed_left_regulator.i_k = 0.0;
	robot.regulator.speed_left_regulator.d_k = 0.0;

	robot.regulator.speed_right_regulator.p_k = 1.0;
	robot.regulator.speed_right_regulator.i_k = 0.0;
	robot.regulator.speed_right_regulator.d_k = 0.0;

	robot.regulator.track_regulator.p_k = 1.0;
	robot.regulator.track_regulator.i_k = 0.0;
	robot.regulator.track_regulator.d_k = 0.0;

	robot.regulator.speed_left_regulator.error_end = 0.01;
	robot.regulator.speed_left_regulator.prev_error = 0.00;
	robot.regulator.speed_left_regulator.sum_error = 0.00;
	robot.regulator.speed_left_regulator.error = 0.00;
	robot.regulator.speed_left_regulator.current = 0.00;
	robot.regulator.speed_left_regulator.target = 0.00;
	robot.regulator.speed_left_regulator.max_error = 5.0;
	robot.regulator.speed_left_regulator.min_error = 0.01;
	robot.regulator.speed_left_regulator.pid_on = 0;
	robot.regulator.speed_left_regulator.pid_finish = 0;

	robot.regulator.speed_right_regulator.error_end = 0.01;
	robot.regulator.speed_right_regulator.prev_error = 0.00;
	robot.regulator.speed_right_regulator.sum_error = 0.00;
	robot.regulator.speed_right_regulator.error = 0.00;
	robot.regulator.speed_right_regulator.current = 0.00;
	robot.regulator.speed_right_regulator.target = 0.00;
	robot.regulator.speed_right_regulator.max_error = 5.0;
	robot.regulator.speed_right_regulator.min_error = 0.01;
	robot.regulator.speed_right_regulator.pid_on = 0;
	robot.regulator.speed_right_regulator.pid_finish = 0;

	robot.regulator.track_regulator.error_end = 0.01;
	robot.regulator.track_regulator.prev_error = 0.00;
	robot.regulator.track_regulator.sum_error = 0.00;
	robot.regulator.track_regulator.error = 0.00;
	robot.regulator.track_regulator.current = 0.00;
	robot.regulator.track_regulator.target = 0.00;
	robot.regulator.track_regulator.max_error = 5.0;
	robot.regulator.track_regulator.min_error = 0.01;
	robot.regulator.track_regulator.pid_on = 0;
	robot.regulator.track_regulator.pid_finish = 0;

	robot.regulator.get_new_data = &regulator_get_data;
	robot.regulator.step = &regulator_calc;
	robot.regulator.on = &regulator_on;
	robot.regulator.off = &regulator_off;
}

static void kinematic_init(void)
{
	robot.kinematic.crawler.target_moving[0] = 0.00;
	robot.kinematic.crawler.target_moving[1] = 0.00;
	robot.kinematic.crawler.target_moving[2] = 0.00;
	robot.kinematic.inverse = &reverse_kinematic;
	robot.kinematic.direct = &direct_kinematic;
	robot.kinematic.set = &set_kinematic_target;
}

static void engines_init(void)
{
	robot.engines.left_crawler_engine.period = &TIM2->ARR;
	robot.engines.left_crawler_engine.pwm = &TIM2->CCR1;
	robot.engines.left_crawler_engine.GPIOx = ENGINE1_DIR_GPIO_Port;
	robot.engines.left_crawler_engine.GPIO_Pin = ENGINE1_DIR_Pin;

	robot.engines.right_crawler_engine.period = &TIM2->ARR;
	robot.engines.right_crawler_engine.pwm = &TIM2->CCR2;
	robot.engines.right_crawler_engine.GPIOx = ENGINE2_DIR_GPIO_Port;
	robot.engines.right_crawler_engine.GPIO_Pin = ENGINE2_DIR_Pin;

	robot.engines.skis_crawler_engine.period = &TIM1->ARR;
	robot.engines.skis_crawler_engine.pwm = &TIM1->CCR1;
	robot.engines.skis_crawler_engine.GPIOx = ENGINE3_DIR_GPIO_Port;
	robot.engines.skis_crawler_engine.GPIO_Pin = ENGINE3_DIR_Pin;

	robot.engines.set_voltage = &set_voltage;
}

static void sensors_init(void)
{
	robot.left_crawler_encoder.counter = &TIM3->CNT;
	robot.right_crawler_encoder.counter = &TIM4->CNT;
	robot.skis_crawler_encoder.counter = &TIM5->CNT;

	robot.left_crawler_encoder.distanse = 0.00;
	robot.left_crawler_encoder.line_speed = 0.00;
	robot.left_crawler_encoder.encoder_data = 0;

	robot.right_crawler_encoder.distanse = 0.00;
	robot.right_crawler_encoder.line_speed = 0.00;
	robot.right_crawler_encoder.encoder_data = 0;

	robot.skis_crawler_encoder.distanse = 0.00;
	robot.skis_crawler_encoder.line_speed = 0.00;
	robot.skis_crawler_encoder.encoder_data = 0;
}

static void regulator_calc(PID_parameters_t *param)
{
	if(param->pid_on)
	{

	}


}

static void regulator_get_data(PID_parameters_t *param)
{

}

static void regulator_on(PID_parameters_t *pid)
{
	pid->pid_on = 1;
}

static void regulator_off(PID_parameters_t *pid)
{
	pid->pid_on = 0;
	pid->error = 0.00;
	pid->output = 0.00;
	pid->sum_error = 0.00;
	pid->target = 0.00;
	pid->prev_error = 0.00;
}

void subscription_callback(const void* msgin)
{
	const geometry_msgs__msg__Twist * msg = (geometry_msgs__msg__Twist *)msgin;
	robot.geometry_msg = *msg;
}

/*!
 * ROS_sync(void) - synchronization with micro-ROS agent
 *
 */
static void ros_sync(void)
{
	const int timeout_ms = 100;
	const uint8_t attempts = 1;

	rmw_ret_t ping_result = rmw_uros_ping_agent(timeout_ms, attempts);

	if(RMW_RET_OK != ping_result)
	{
		robot.start_flag = 0;
		xTaskNotifyGive(RestartTaskHandle);
	} else robot.start_flag = 1;
}

static void ros_process(void)
{
	robot.odometry_msg.pose.pose.position.x = 0.2;
	robot.odometry_msg.pose.pose.position.y = 0.2;
	robot.odometry_msg.pose.pose.position.z = 0.2;
    robot.ret = rcl_publish(&robot.odometry_publisher, &robot.odometry_msg, NULL);
    rclc_executor_spin_some(
      &robot.executor,
	  RCL_MS_TO_NS(25));
}

static void ros_diagnostic_process(void)
{
	const int timeout_ms = 50;
	rmw_uros_sync_session(timeout_ms);
	if(rmw_uros_epoch_synchronized())
	{
		robot.time_ms = rmw_uros_epoch_millis();
		robot.time_ns = rmw_uros_epoch_nanos();

		robot.diagnostic_msg.header.stamp.sec = (int32_t)(robot.time_ms / 1000);
		robot.diagnostic_msg.header.stamp.nanosec = (uint32_t)(robot.time_ns);
		robot.diagnostic_msg.header.frame_id.data = "[CRAWLER:GENERAL] NO ERRORS";
		robot.ret = rcl_publish(&robot.diagnostic_publisher, &robot.diagnostic_msg, NULL);
	}
}

static bool ros_init(void)
{
	  // micro-ROS app
	  if(RMW_RET_OK != rmw_uros_ping_agent(100, 1)) return false;
	  //create init_options
	  RCCHECK(rclc_support_init(&robot.support, 0, NULL, &allocator));

	  // create node
	  RCCHECK(rclc_node_init_default(&robot.node, "CrawlerBot_node", "", &robot.support));

	  // create publisher
	  RCCHECK(rclc_publisher_init_default(
					&robot.diagnostic_publisher,
					&robot.node,
					ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray),
					"CrawlerBot/diagnostic"));

	  RCCHECK(rclc_publisher_init_default(
					&robot.odometry_publisher,
					&robot.node,
					ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
					"CrawlerBot/odometry"));

	  RCCHECK(rclc_subscription_init_default(
					&robot.geometry_subscriber,
					&robot.node,
					ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
					"CrawlerBot/twist"));

	  robot.executor = rclc_executor_get_zero_initialized_executor();
	  RCCHECK(rclc_executor_init(
			  	    &robot.executor,
					&robot.support.context,
					1,
					&robot.allocator));

	  RCCHECK(rclc_executor_add_subscription(
				    &robot.executor,
				    &robot.geometry_subscriber,
				    &robot.geometry_msg,
				    &subscription_callback,
				    ON_NEW_DATA));
	  robot.start_flag = 1;
	  return true;
}

static void ros_deinit(void)
{
	  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&robot.support.context);
	  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

	  robot.ret = rcl_publisher_fini(&robot.diagnostic_publisher, &robot.node);
	  robot.ret = rcl_publisher_fini(&robot.odometry_publisher, &robot.node);
	  rclc_executor_fini(&robot.executor);
	  robot.ret = rcl_subscription_fini(&robot.geometry_subscriber, &robot.node);
	  robot.ret = rcl_node_fini(&robot.node);
	  rclc_support_fini(&robot.support);
}

static void set_voltage(Engine_parameters_t engine, double duty)
{
	if(duty > 1.0) duty = 1.0;
	if(duty < -1.0) duty = -1.0;
	if(duty >= 0.0)
	{
		*engine.pwm = (int32_t)(duty * (*engine.period));
		HAL_GPIO_WritePin(engine.GPIOx, engine.GPIO_Pin, GPIO_PIN_RESET);
	} else
	{
		*engine.pwm = (int32_t)(duty + (duty * (*engine.period)));
		HAL_GPIO_WritePin(engine.GPIOx, engine.GPIO_Pin, GPIO_PIN_SET);
	}
}
/* USER CODE END Application */

