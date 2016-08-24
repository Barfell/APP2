/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define E2_MOTOR_STEP_Pin GPIO_PIN_2
#define E2_MOTOR_STEP_GPIO_Port GPIOE
#define E1_MOTOR_STEP_Pin GPIO_PIN_3
#define E1_MOTOR_STEP_GPIO_Port GPIOE
#define E2_MOTOR_EN_Pin GPIO_PIN_4
#define E2_MOTOR_EN_GPIO_Port GPIOE
#define E1_MOTOR_DIR_Pin GPIO_PIN_5
#define E1_MOTOR_DIR_GPIO_Port GPIOE
#define T_CS_Pin GPIO_PIN_13
#define T_CS_GPIO_Port GPIOC
#define IO_FAN_5V_Pin GPIO_PIN_14
#define IO_FAN_5V_GPIO_Port GPIOC
#define IO_FAN_E_Pin GPIO_PIN_15
#define IO_FAN_E_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOC
#define T_MOSI_Pin GPIO_PIN_1
#define T_MOSI_GPIO_Port GPIOC
#define X_MIN_Pin GPIO_PIN_2
#define X_MIN_GPIO_Port GPIOC
#define X_MOTOR_EN_Pin GPIO_PIN_3
#define X_MOTOR_EN_GPIO_Port GPIOC
#define Z_MIN_Pin GPIO_PIN_0
#define Z_MIN_GPIO_Port GPIOA
#define TIM5_CH2_PWM_FAN_Pin GPIO_PIN_1
#define TIM5_CH2_PWM_FAN_GPIO_Port GPIOA
#define PWM_BEEP_Pin GPIO_PIN_2
#define PWM_BEEP_GPIO_Port GPIOA
#define X_MOTOR_DIR_Pin GPIO_PIN_3
#define X_MOTOR_DIR_GPIO_Port GPIOA
#define Y_MIN_Pin GPIO_PIN_4
#define Y_MIN_GPIO_Port GPIOA
#define ADC1_IN5_MAT_CHECK_Pin GPIO_PIN_5
#define ADC1_IN5_MAT_CHECK_GPIO_Port GPIOA
#define ADC1_IN6_TEMP_NOZZLE_Pin GPIO_PIN_6
#define ADC1_IN6_TEMP_NOZZLE_GPIO_Port GPIOA
#define ADC1_IN7_TEMP_BED_Pin GPIO_PIN_7
#define ADC1_IN7_TEMP_BED_GPIO_Port GPIOA
#define X_MOTOR_STEP_Pin GPIO_PIN_5
#define X_MOTOR_STEP_GPIO_Port GPIOC
#define T_SCK_Pin GPIO_PIN_0
#define T_SCK_GPIO_Port GPIOB
#define T_PEN_Pin GPIO_PIN_1
#define T_PEN_GPIO_Port GPIOB
#define T_MISO_Pin GPIO_PIN_2
#define T_MISO_GPIO_Port GPIOB
#define PWM_Z_VREF_Pin GPIO_PIN_10
#define PWM_Z_VREF_GPIO_Port GPIOB
#define PWM_XY_VREF_Pin GPIO_PIN_11
#define PWM_XY_VREF_GPIO_Port GPIOB
#define Y_MOTOR_DIR_Pin GPIO_PIN_12
#define Y_MOTOR_DIR_GPIO_Port GPIOB
#define Y_MOTOR_EN_Pin GPIO_PIN_13
#define Y_MOTOR_EN_GPIO_Port GPIOB
#define Y_MOTOR_STEP_Pin GPIO_PIN_11
#define Y_MOTOR_STEP_GPIO_Port GPIOD
#define PWM_HEAT_BED_Pin GPIO_PIN_12
#define PWM_HEAT_BED_GPIO_Port GPIOD
#define PWM_HEAT_NOZZLE_Pin GPIO_PIN_13
#define PWM_HEAT_NOZZLE_GPIO_Port GPIOD
#define IO_FAN_BOARD_Pin GPIO_PIN_6
#define IO_FAN_BOARD_GPIO_Port GPIOC
#define Z_MOTOR_STEP_Pin GPIO_PIN_7
#define Z_MOTOR_STEP_GPIO_Port GPIOC
#define PWM_E_VREF_Pin GPIO_PIN_8
#define PWM_E_VREF_GPIO_Port GPIOA
#define Z_MAX_Pin GPIO_PIN_15
#define Z_MAX_GPIO_Port GPIOA
#define INT_POWER_Pin GPIO_PIN_3
#define INT_POWER_GPIO_Port GPIOD
#define Z_MOTOR_EN_Pin GPIO_PIN_6
#define Z_MOTOR_EN_GPIO_Port GPIOD
#define LCD_LIGHT_UP_Pin GPIO_PIN_5
#define LCD_LIGHT_UP_GPIO_Port GPIOB
#define INT_SD_DETECT_Pin GPIO_PIN_6
#define INT_SD_DETECT_GPIO_Port GPIOB
#define Z_MOTOR_DIR_Pin GPIO_PIN_7
#define Z_MOTOR_DIR_GPIO_Port GPIOB
#define PWM_SW_EXT_Pin GPIO_PIN_9
#define PWM_SW_EXT_GPIO_Port GPIOB
#define E2_MOTOR_DIR_Pin GPIO_PIN_0
#define E2_MOTOR_DIR_GPIO_Port GPIOE
#define E1_MOTOR_EN_Pin GPIO_PIN_1
#define E1_MOTOR_EN_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
