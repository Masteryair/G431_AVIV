
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#pragma once
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define NR 18
#define clk 170
#define tx_buf_size 27
#define Lad 6
#define dt_dgima 74
#define dt_delay 160
#define COMP_Factor 0.737
#define COMP_Bias 166
#define pi2 6.2832
#define curr_f  1.4208
#define IbuffL  20
#define LCAN 64
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum{NONE=0,UART_RCV,CAN_RCV,POT_RCV}COMM_TYPE;
typedef enum{MAXON=0,FAULHABER}MOTOR_TYPE;

typedef struct {
	COMM_TYPE com_recievd;
	int8_t uart_receiving;
	int8_t is_master;
} COMMUNICATION_STRUCT;

typedef struct {
	int8_t pressed;
	uint32_t time;
	uint32_t time0;
	uint32_t step_time;
	uint32_t step_time0;
	int16_t rep;


	float pot;
	int8_t mode;
	int8_t zeroed;
	int16_t value;
	int16_t step;
} BUTTON_STRUCT;

typedef struct {
    float int_l;
	float int_h;
	float int_t;
    float kp;
    float ki;
    float kd;
    float pos;
    float vel;
    float target;
    float err;
    float i_err;
    int8_t motion_comp;
    int8_t in_win;
} POSITION_STRUCT;

typedef struct {

	float pos_l;
	float pos_h;
	float pos;
	float pos_temp;
	float check;
	float tar_pos;
	float tar_vel;
	float tar_acc;
} PROFILE_STRUCT;

typedef struct {
    float kp;
    float ki;
    float e;
    float int_t;
    float int_l;
    float int_h;
    float int_max;
    float max;
    float p;
    float i;
    float tar;
    float meas;
    float filt;
    float factor;
    float PWM;
    uint16_t PWM16;
    uint16_t PW[3];
    int8_t dir;
    int8_t CP;
    uint16_t PWMAX;
    uint16_t PWMAX2;
    int16_t dbg;
    uint16_t meas_hd[10];
    uint16_t l_hd;
    uint16_t CCER;
    uint16_t CCERN;
} CURRENT_STRUCT;


typedef enum{NORMAL=0,CONST,PWM,SIN,RAMP,BIASED_RAMP,MUSIC,MUSICPWM,TORQUE,PWM3}CONTROL_MODE;

typedef struct {
    int8_t hall;
    int8_t hall_prev;
    float pos;
    float vel;
    uint16_t pwm[3];
    int8_t polar[3];//phase polarity PN
    float busv;
    int8_t on;
    int8_t stand;
    float win_cnt;
    CONTROL_MODE curr_mode;
    MOTOR_TYPE type;
} MOTOR_STRUCT;

typedef struct {
	float freq;
	uint32_t t0;
	float a;
	float b;
} BALAM_STRUCT;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAN_TERM_Pin GPIO_PIN_14
#define CAN_TERM_GPIO_Port GPIOC
#define BUTTON_Pin GPIO_PIN_10
#define BUTTON_GPIO_Port GPIOC
#define BEMF_Pin GPIO_PIN_5
#define BEMF_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

