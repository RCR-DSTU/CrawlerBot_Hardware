/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
	uint8_t pid_on;
	uint8_t pid_finish;
	double output;
	double error;
	double sum_error;
	double prev_error;
	double current;
	double target;
	double error_end;
	double max_error;
	double min_error;
	double p_k;
	double i_k;
	double d_k;
}PID_parameters_t;

typedef struct {
	volatile uint32_t *pwm;
	volatile uint32_t *period;
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
}Engine_parameters_t;

typedef struct {
	Engine_parameters_t left_crawler_engine;
	Engine_parameters_t right_crawler_engine;
	Engine_parameters_t skis_crawler_engine;
	void (*set_voltage)(Engine_parameters_t engine, double duty);
}Engine_t;

typedef struct {
	PID_parameters_t speed_left_regulator;
	PID_parameters_t speed_right_regulator;
	PID_parameters_t track_regulator;
	void (*get_new_data)(PID_parameters_t *param);
	void (*step)(PID_parameters_t *param);
	void (*output)(Engine_t engine);
	void (*on)(PID_parameters_t *pid);
	void (*off)(PID_parameters_t *pid);
}Regulator_t;

typedef struct {
	uint8_t kinematic_on;
	float target_moving[3];
	float current_moving[3];
	float kinematic_out[3];
}Kinematic_parameters_t;

typedef struct {
	Kinematic_parameters_t crawler;
	void (*set)(Kinematic_parameters_t kinematic, float Vx, float Vy, float angular);
	void (*inverse)(Kinematic_parameters_t kinematic);
	void (*direct)(Kinematic_parameters_t kinematic);
}Kinematic_t;

typedef struct {
	volatile uint32_t *counter;
	int16_t encoder_data;
	double line_speed;
	double distanse;
}Encoder_t;

extern Engine_t engines;
extern Kinematic_t kinematic;
extern Encoder_t left_crawler_encoder;
extern Encoder_t right_crawler_encoder;
extern Encoder_t skis_crawler_encoder;
extern Regulator_t regulator;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define PID_START(PID) PID.pid_on = 1;
#define PID_STOP(PID)  PID.pid_on = 0;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENGINE1_DIR_Pin GPIO_PIN_7
#define ENGINE1_DIR_GPIO_Port GPIOA
#define ENGINE2_DIR_Pin GPIO_PIN_0
#define ENGINE2_DIR_GPIO_Port GPIOB
#define ENGINE3_DIR_Pin GPIO_PIN_1
#define ENGINE3_DIR_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
