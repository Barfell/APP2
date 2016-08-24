/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_host.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

IWDG_HandleTypeDef hiwdg;

SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

HCD_HandleTypeDef hhcd_USB_OTG_HS;

SRAM_HandleTypeDef hsram1;
osThreadId defaultTaskHandle;
osThreadId initTaskHandle;
osThreadId refDataTaskHandle;
osThreadId readUdiskTaskHandle;
osThreadId printTaskHandle;
osThreadId guiTaskHandle;
osThreadId respondGuiTaskHandle;
osMessageQId gcodeCommandHandle;
osSemaphoreId guiSendSemHandle;
osSemaphoreId guiWaitSemHandle;
osSemaphoreId readUdiskSemHandle;
osSemaphoreId RecUartCmdSemHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_FSMC_Init(void);
static void MX_CRC_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USB_OTG_HS_HCD_Init(void);
static void MX_TIM5_Init(void);
void StartDefaultTask(void const * argument);
void InitTask_Func(void const * argument);
void RefDataTask_Func(void const * argument);
void ReadUdiskTask_Func(void const * argument);
void PrintTask_Func(void const * argument);
void GuiTask_func(void const * argument);
void RespondGuiTask_func(void const * argument);
static void MX_NVIC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SDIO_SD_Init();
  MX_FSMC_Init();
  MX_CRC_Init();
  MX_IWDG_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_OTG_HS_HCD_Init();
  MX_TIM5_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of guiSendSem */
  osSemaphoreDef(guiSendSem);
  guiSendSemHandle = osSemaphoreCreate(osSemaphore(guiSendSem), 2);

  /* definition and creation of guiWaitSem */
  osSemaphoreDef(guiWaitSem);
  guiWaitSemHandle = osSemaphoreCreate(osSemaphore(guiWaitSem), 2);

  /* definition and creation of readUdiskSem */
  osSemaphoreDef(readUdiskSem);
  readUdiskSemHandle = osSemaphoreCreate(osSemaphore(readUdiskSem), 2);

  /* definition and creation of RecUartCmdSem */
  osSemaphoreDef(RecUartCmdSem);
  RecUartCmdSemHandle = osSemaphoreCreate(osSemaphore(RecUartCmdSem), 2);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of initTask */
  osThreadDef(initTask, InitTask_Func, osPriorityRealtime, 0, 640);
  initTaskHandle = osThreadCreate(osThread(initTask), NULL);

  /* definition and creation of refDataTask */
  osThreadDef(refDataTask, RefDataTask_Func, osPriorityNormal, 0, 640);
  refDataTaskHandle = osThreadCreate(osThread(refDataTask), NULL);

  /* definition and creation of readUdiskTask */
  osThreadDef(readUdiskTask, ReadUdiskTask_Func, osPriorityBelowNormal, 0, 640);
  readUdiskTaskHandle = osThreadCreate(osThread(readUdiskTask), NULL);

  /* definition and creation of printTask */
  osThreadDef(printTask, PrintTask_Func, osPriorityLow, 0, 640);
  printTaskHandle = osThreadCreate(osThread(printTask), NULL);

  /* definition and creation of guiTask */
  osThreadDef(guiTask, GuiTask_func, osPriorityIdle, 0, 640);
  guiTaskHandle = osThreadCreate(osThread(guiTask), NULL);

  /* definition and creation of respondGuiTask */
  osThreadDef(respondGuiTask, RespondGuiTask_func, osPriorityAboveNormal, 0, 640);
  respondGuiTaskHandle = osThreadCreate(osThread(respondGuiTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of gcodeCommand */
  osMessageQDef(gcodeCommand, 16, uint32_t);
  gcodeCommandHandle = osMessageCreate(osMessageQ(gcodeCommand), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* PVD_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PVD_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(PVD_IRQn);
  /* FLASH_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(FLASH_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(FLASH_IRQn);
  /* RCC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(RCC_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(RCC_IRQn);
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* SDIO_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SDIO_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(SDIO_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* OTG_FS_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(OTG_FS_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* OTG_HS_EP1_OUT_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(OTG_HS_EP1_OUT_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(OTG_HS_EP1_OUT_IRQn);
  /* OTG_HS_EP1_IN_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(OTG_HS_EP1_IN_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(OTG_HS_EP1_IN_IRQn);
  /* OTG_HS_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(OTG_HS_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
  /* FPU_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(FPU_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(FPU_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* TIM5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM5_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 3125;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SDIO init function */
static void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 1;

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 41;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 16383;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim5);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USB_OTG_HS init function */
static void MX_USB_OTG_HS_HCD_Init(void)
{

  hhcd_USB_OTG_HS.Instance = USB_OTG_HS;
  hhcd_USB_OTG_HS.Init.dev_endpoints = 6;
  hhcd_USB_OTG_HS.Init.Host_channels = 12;
  hhcd_USB_OTG_HS.Init.speed = HCD_SPEED_FULL;
  hhcd_USB_OTG_HS.Init.dma_enable = ENABLE;
  hhcd_USB_OTG_HS.Init.phy_itface = USB_OTG_EMBEDDED_PHY;
  hhcd_USB_OTG_HS.Init.Sof_enable = DISABLE;
  hhcd_USB_OTG_HS.Init.low_power_enable = DISABLE;
  hhcd_USB_OTG_HS.Init.vbus_sensing_enable = DISABLE;
  hhcd_USB_OTG_HS.Init.use_external_vbus = DISABLE;
  if (HAL_HCD_Init(&hhcd_USB_OTG_HS) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, E2_MOTOR_STEP_Pin|E1_MOTOR_STEP_Pin|E2_MOTOR_EN_Pin|E1_MOTOR_DIR_Pin 
                          |E2_MOTOR_DIR_Pin|E1_MOTOR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, T_CS_Pin|IO_FAN_5V_Pin|IO_FAN_E_Pin|LED1_Pin 
                          |T_MOSI_Pin|X_MOTOR_EN_Pin|X_MOTOR_STEP_Pin|IO_FAN_BOARD_Pin 
                          |Z_MOTOR_STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PWM_BEEP_Pin|X_MOTOR_DIR_Pin|PWM_E_VREF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, T_SCK_Pin|PWM_Z_VREF_Pin|PWM_XY_VREF_Pin|Y_MOTOR_DIR_Pin 
                          |Y_MOTOR_EN_Pin|LCD_LIGHT_UP_Pin|Z_MOTOR_DIR_Pin|PWM_SW_EXT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Y_MOTOR_STEP_Pin|PWM_HEAT_BED_Pin|PWM_HEAT_NOZZLE_Pin|Z_MOTOR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : E2_MOTOR_STEP_Pin E1_MOTOR_STEP_Pin E2_MOTOR_EN_Pin E1_MOTOR_DIR_Pin 
                           E2_MOTOR_DIR_Pin E1_MOTOR_EN_Pin */
  GPIO_InitStruct.Pin = E2_MOTOR_STEP_Pin|E1_MOTOR_STEP_Pin|E2_MOTOR_EN_Pin|E1_MOTOR_DIR_Pin 
                          |E2_MOTOR_DIR_Pin|E1_MOTOR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : T_CS_Pin IO_FAN_5V_Pin IO_FAN_E_Pin T_MOSI_Pin 
                           X_MOTOR_EN_Pin X_MOTOR_STEP_Pin IO_FAN_BOARD_Pin Z_MOTOR_STEP_Pin */
  GPIO_InitStruct.Pin = T_CS_Pin|IO_FAN_5V_Pin|IO_FAN_E_Pin|T_MOSI_Pin 
                          |X_MOTOR_EN_Pin|X_MOTOR_STEP_Pin|IO_FAN_BOARD_Pin|Z_MOTOR_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : X_MIN_Pin */
  GPIO_InitStruct.Pin = X_MIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(X_MIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Z_MIN_Pin Y_MIN_Pin Z_MAX_Pin */
  GPIO_InitStruct.Pin = Z_MIN_Pin|Y_MIN_Pin|Z_MAX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PWM_BEEP_Pin X_MOTOR_DIR_Pin PWM_E_VREF_Pin */
  GPIO_InitStruct.Pin = PWM_BEEP_Pin|X_MOTOR_DIR_Pin|PWM_E_VREF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : T_SCK_Pin PWM_Z_VREF_Pin PWM_XY_VREF_Pin Y_MOTOR_DIR_Pin 
                           Y_MOTOR_EN_Pin Z_MOTOR_DIR_Pin PWM_SW_EXT_Pin */
  GPIO_InitStruct.Pin = T_SCK_Pin|PWM_Z_VREF_Pin|PWM_XY_VREF_Pin|Y_MOTOR_DIR_Pin 
                          |Y_MOTOR_EN_Pin|Z_MOTOR_DIR_Pin|PWM_SW_EXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : T_PEN_Pin T_MISO_Pin */
  GPIO_InitStruct.Pin = T_PEN_Pin|T_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Y_MOTOR_STEP_Pin PWM_HEAT_BED_Pin PWM_HEAT_NOZZLE_Pin Z_MOTOR_EN_Pin */
  GPIO_InitStruct.Pin = Y_MOTOR_STEP_Pin|PWM_HEAT_BED_Pin|PWM_HEAT_NOZZLE_Pin|Z_MOTOR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_POWER_OFF_Pin */
  GPIO_InitStruct.Pin = INT_POWER_OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(INT_POWER_OFF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_LIGHT_UP_Pin */
  GPIO_InitStruct.Pin = LCD_LIGHT_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_LIGHT_UP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_SD_DETECT_Pin */
  GPIO_InitStruct.Pin = INT_SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(INT_SD_DETECT_GPIO_Port, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{
  FSMC_NORSRAM_TimingTypeDef Timing;
  FSMC_NORSRAM_TimingTypeDef ExtTiming;

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_ENABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 60;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */
  ExtTiming.AddressSetupTime = 9;
  ExtTiming.AddressHoldTime = 15;
  ExtTiming.DataSetupTime = 8;
  ExtTiming.BusTurnAroundDuration = 0;
  ExtTiming.CLKDivision = 16;
  ExtTiming.DataLatency = 17;
  ExtTiming.AccessMode = FSMC_ACCESS_MODE_A;

  if (HAL_SRAM_Init(&hsram1, &Timing, &ExtTiming) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init();

  /* init code for USB_HOST */
  MX_USB_HOST_Init();

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* InitTask_Func function */
void InitTask_Func(void const * argument)
{
  /* USER CODE BEGIN InitTask_Func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END InitTask_Func */
}

/* RefDataTask_Func function */
void RefDataTask_Func(void const * argument)
{
  /* USER CODE BEGIN RefDataTask_Func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RefDataTask_Func */
}

/* ReadUdiskTask_Func function */
void ReadUdiskTask_Func(void const * argument)
{
  /* USER CODE BEGIN ReadUdiskTask_Func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ReadUdiskTask_Func */
}

/* PrintTask_Func function */
void PrintTask_Func(void const * argument)
{
  /* USER CODE BEGIN PrintTask_Func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END PrintTask_Func */
}

/* GuiTask_func function */
void GuiTask_func(void const * argument)
{
  /* USER CODE BEGIN GuiTask_func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GuiTask_func */
}

/* RespondGuiTask_func function */
void RespondGuiTask_func(void const * argument)
{
  /* USER CODE BEGIN RespondGuiTask_func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RespondGuiTask_func */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/