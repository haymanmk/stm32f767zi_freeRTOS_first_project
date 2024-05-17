/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32f7xx_timer_extension.h"
#include "double_buffer_mem_manage.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VECT_TAB_OFFSET 0x00100000U // Vector Table base offset field.
#define STACK_SIZE configMINIMAL_STACK_SIZE
#define APB1_CLOCK 108000000    // Hz
#define NVIC_NUM_INTERRUPTS 110 // Number of interrupts in NVIC, 0-109

/**
 * @brief FreeRTOS Task Notify Value for updating pulse frequency
 *
 * The notify value is a 32-bit value which will be split into 2 parts:
 * - The lower 16 bits is the channel ID of the timer
 * - The higher 16 bits is the buffer ID of the double buffer
 */
// Timer Channel ID, 1, 2, 3, 4
#define TASK_NOTIFY_TIM_CHANNEL_1 0x01
#define TASK_NOTIFY_TIM_CHANNEL_2 0x02
#define TASK_NOTIFY_TIM_CHANNEL_3 0x04
#define TASK_NOTIFY_TIM_CHANNEL_4 0x08

// Double Buffer ID, 0 or 1
// The buffer ID is used to identify which buffer should be updated
#define TASK_NOTIFY_BUFFER_ID_OFFSET 16 // bit offset
#define TASK_NOTIFY_BUFFER_ID_0 0x00 << TASK_NOTIFY_BUFFER_ID_OFFSET
#define TASK_NOTIFY_BUFFER_ID_1 0x01 << TASK_NOTIFY_BUFFER_ID_OFFSET

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define COMPUTE_INCREMENT_FOR_PULSE(pulse_freq, clock_freq) ((uint32_t)((uint32_t)clock_freq / ((uint32_t)pulse_freq * 2)))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim5_ch1;
DMA_HandleTypeDef hdma_tim5_ch4_trig;

/* USER CODE BEGIN PV */
TaskHandle_t xHandleTask1 = NULL;
TaskHandle_t xHandleTask2 = NULL;
TaskHandle_t xHandleUpdatePulseData = NULL;
TaskHandle_t xHandleMainTask = NULL;

// Pulse frequency
volatile uint32_t pulseFrequency = 100000; // Hz

// Double Buffer Space
static doubleBufferArray_t *preparingBufferCh1 = NULL;
static doubleBufferArray_t *preparedBufferCh1 = NULL;
static doubleBufferArray_t *preparingBufferCh4 = NULL;
static doubleBufferArray_t *preparedBufferCh4 = NULL;
volatile doubleBufferArray_t *freeBufferCh1 = NULL;
volatile doubleBufferArray_t *freeBufferCh4 = NULL;
static doubleBufferArray_t doubleBuffer1;
static doubleBufferArray_t doubleBuffer2;

// Signals to notify the main task
volatile uint32_t ISRNotificationValue = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void updateBuffer(doubleBufferArray_t *buffer, uint32_t start_index, uint32_t start_value, uint32_t increment, size_t length);
void bootloaderJumpToApp(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void vMainTask(void *pvParameters)
{
  uint32_t ulNotificationValue = 0;
  volatile lastCounter = 0;
  volatile preparingBufferReady = 0;

  // Prepare double buffer
  uint32_t pulseIncrementCh1 = COMPUTE_INCREMENT_FOR_PULSE(pulseFrequency, APB1_CLOCK);
  preparedBufferCh1 = &doubleBuffer1;
  updateBuffer(preparedBufferCh1, 0, 0 + pulseIncrementCh1, pulseIncrementCh1, DOUBLE_BUFFER_SIZE);
  lastCounter = (*preparedBufferCh1)[DOUBLE_BUFFER_SIZE - 1];

  // add a signature in a pulse for debugging purpose
  // (*preparedBufferCh1)[2] += 0.8 * pulseIncrementCh1;

  // assign array to preparingBuffer
  preparingBufferCh1 = &doubleBuffer2;

  // start timer output compare mode with DMA
  HAL_TIM_OC_Start_DMA(&htim5, TIM_CHANNEL_1, (uint32_t)preparedBufferCh1, DOUBLE_BUFFER_SIZE);

  // Reset TIM5 Counter
  __HAL_TIM_SET_COUNTER(&htim5, 0);

  for (;;)
  {
    /* do something  */
    // Set PD7 to low =====> this is the beginning of the preparation
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

    if (preparingBufferReady == 0)
    {
      // set PD7 to high
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

      // vLoggingPrintf("[0]Timer Counter: %x\n", __HAL_TIM_GET_COUNTER(&htim5));
      // prepare for next buffer
      updateBuffer(preparingBufferCh1, 0, lastCounter + pulseIncrementCh1, pulseIncrementCh1, DOUBLE_BUFFER_SIZE);
      // record last counter value
      lastCounter = (*preparingBufferCh1)[DOUBLE_BUFFER_SIZE - 1];

      // add a signature in a pulse for debugging purpose
      // (*preparingBufferCh1)[2] += 0.8 * pulseIncrementCh1;

      preparingBufferReady = 1;

      // set PD7 to low =====> this is the end of the preparation
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
    }

    // Wait for the notification from the interrupt
    // xTaskNotifyWait(0, 0xFFFFFFFF, &ulNotificationValue, portMAX_DELAY);
    if ((ISRNotificationValue & TASK_NOTIFY_TIM_CHANNEL_1) != 0 && preparingBufferReady == 1)
    {
      // set PD6 to high =====> this is the beginning of the notification
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);

      // clear the notification
      ISRNotificationValue &= ~TASK_NOTIFY_TIM_CHANNEL_1;

      // swap buffer
      doubleBufferArray_t *temp = preparedBufferCh1;
      preparedBufferCh1 = preparingBufferCh1;
      preparingBufferCh1 = temp;
      preparingBufferReady = 0;

      // start DMA
      HAL_TIM_OC_Start_DMA(&htim5, TIM_CHANNEL_1, (uint32_t)preparedBufferCh1, DOUBLE_BUFFER_SIZE);

      // set PD6 to low =====> this is the end of the notification
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
    }
  }
}

void vBootloaderTask(void *pvParameters)
{
  for (;;)
  {
    if (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET)
    {
      // Bootloader mode
      // vLoggingPrintf("Bootloader mode is enabled\n");

      // Jump to user application
      bootloaderJumpToApp();
    }
    else
    {
      // vLoggingPrintf("Bootloader mode is disabled\n");
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void vTaskHandleUpdatePulseData(void *pvParameters)
{
  uint32_t ulEventCount = 0;
  uint32_t pulseFrequencyRecord = 0;

  for (;;)
  {
    ulEventCount = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

    if (ulEventCount != 0)
    {
      pulseFrequencyRecord = pulseFrequency;

      vLoggingPrintf("Update pulse frequency: %d\n", pulseFrequencyRecord);
    }
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // Enable DWT Cycle Count Register
  DWT->CTRL |= (0x01 << DWT_CTRL_CYCCNTENA_Pos);

  // Initialize SystemView
  SEGGER_SYSVIEW_Conf();

  // Start recording
  SEGGER_SYSVIEW_Start();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  // Initialize double buffer for TIM5 Channel 1 and Channel 4
  // Calculate the increment value in timer counter for each pulse
  uint32_t pulseIncrementCh1 = COMPUTE_INCREMENT_FOR_PULSE(pulseFrequency, APB1_CLOCK);
  uint32_t pulseIncrementCh4 = COMPUTE_INCREMENT_FOR_PULSE(pulseFrequency, APB1_CLOCK);

  // xTaskCreate(vTaskHandleUpdatePulseData, "UpdatePulseData", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleUpdatePulseData);
  xTaskCreate(vBootloaderTask, "BootloaderTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
  xTaskCreate(vMainTask, "MainTask", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY, &xHandleMainTask);

  // configASSERT(xReturnedTask1);
  // configASSERT(xReturnedTask2);

  tcp_server_init();

  if (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET)
  {
    // Bootloader mode
    // vLoggingPrintf("Bootloader mode is enabled\n");

    // Jump to user application
    bootloaderJumpToApp();
  }

  vTaskStartScheduler();

  /* It should never get here as the scheduler is started. */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    printf("main is running\n");
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
   */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin | RMII_RXD0_Pin | RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin | RMII_MDIO_Pin | RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin | STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin | USB_ID_Pin | USB_DM_Pin | USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin | RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Configure GPIO pins : PD6 and PD7 as output */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void vApplicationMallocFailedHook(void)
{
  static volatile uint32_t ulMallocFailures = 0;

  /* Called if a call to pvPortMalloc() fails because there is insufficient
  free memory available in the FreeRTOS heap.  pvPortMalloc() is called
  internally by FreeRTOS API functions that create tasks, queues, software
  timers, and semaphores.  The size of the FreeRTOS heap is set by the
  configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
  ulMallocFailures++;

  vLoggingPrintf("Malloc failed %d times\n", ulMallocFailures);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)pcTaskName;
  (void)xTask;

  /* Run time stack overflow checking is performed if
  configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
  function is called if a stack overflow is detected. */
  taskDISABLE_INTERRUPTS();
  for (;;)
    ;
}

/**
 * @brief Disable all interrupts
 */
void disable_all_interrupts(void)
{
  for (int i = 0; i < NVIC_NUM_INTERRUPTS; i++)
  {
    NVIC_DisableIRQ(i);
  }
}

/**
 * @brief Clear all pending interrupts
 */
void clear_all_pending_interrupts(void)
{
  for (int i = 0; i < NVIC_NUM_INTERRUPTS; i++)
  {
    NVIC_ClearPendingIRQ(i);
  }
}

/**
 * @brief Stop over drive mode
 */
void stop_over_drive(void)
{
  // Disable over drive mode
  HAL_PWREx_DisableOverDrive();
}

// Jump to user application
void bootloaderJumpToApp(void)
{
  // stop system clock
  // stop_system_clock();

  // Disable all interrupts
  __disable_irq();

  // disable all interrupts
  disable_all_interrupts();

  // clear all pending interrupts
  clear_all_pending_interrupts();

  // enable interrupts
  __enable_irq();

  // reset CONTROL register
  __set_CONTROL(0);

  // Function pointer to the address of the user application
  void (*appResetHandler)(void);

  vLoggingPrintf("Bootloader: Jump to user application\n");

  // Set the vector table offset register to the user application
  // SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET;

  // Set the stack pointer to the user application
  __set_MSP(*(uint32_t *)(FLASH_BASE | VECT_TAB_OFFSET));

  // Get the reset handler address of the user application
  uint32_t resetHandlerAddress = *(uint32_t *)(FLASH_BASE | (VECT_TAB_OFFSET + 4));
  appResetHandler = (void (*)(void))(resetHandlerAddress);

  // Jump to the user application
  appResetHandler();
}

/**
 * @brief Update double buffer with increment value
 */
void updateBuffer(doubleBufferArray_t *buffer, uint32_t start_index, uint32_t start_value, uint32_t increment, size_t length)
{
  for (uint32_t i = start_index; i < start_index + length; i++)
  {
    (*buffer)[i] = start_value + increment * (i - start_index);
  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    // stop counter
    TIM_STOP_COUNTER(htim);

    // notify main task to update pulse data
    ISRNotificationValue |= TASK_NOTIFY_TIM_CHANNEL_1;

    // debug message, print captured value
    // vLoggingPrintf("[ISR]Timer Counter: %x\n", __HAL_TIM_GET_COUNTER(htim));
  }
}

/**
 * @brief interrupt callback as DMA M0 buffer is completely transfered.
 */
void PingPongM0TransferCompleteCallback(TIM_HandleTypeDef *htim)
{
  DMA_HandleTypeDef *hdma;
  uint32_t newAddress;

  // current memory buffer in used is memory 1
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    // free node
    freeBufferCh1 = htim->hdma[TIM_DMA_ID_CC1]->Instance->M0AR;

    if (preparedBufferCh1 == NULL)
    {
      // No buffer is prepared
      // in practice, the pulse generation should be stopped
      return;
    }

    hdma = htim->hdma[TIM_DMA_ID_CC1];
    newAddress = (uint32_t)(*preparedBufferCh1);
    preparedBufferCh1 = NULL;
  }
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
    if (preparedBufferCh4 == NULL)
    {
      // No buffer is prepared
      // in practice, the pulse generation should be stopped
      return;
    }

    hdma = htim->hdma[TIM_DMA_ID_CC4];
    newAddress = (uint32_t)(*preparedBufferCh4);
    preparedBufferCh4 = NULL;
  }
  else
    return;

  // Update the memory 0 address
  if (TIM_OC_Update_DMA_Double_Buffer(hdma, newAddress, SELECT_DOUBLE_BUFFER_M0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief interrupt callback as DMA M1 buffer is completely transfered.
 */
void PingPongM1TransferCompleteCallback(TIM_HandleTypeDef *htim)
{
  DMA_HandleTypeDef *hdma;
  uint32_t newAddress;

  // current memory buffer in used is memory 0
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    // free node
    freeBufferCh1 = htim->hdma[TIM_DMA_ID_CC1]->Instance->M1AR;

    if (preparedBufferCh1 == NULL)
    {
      // No buffer is prepared
      // in practice, the pulse generation should be stopped
      return;
    }

    hdma = htim->hdma[TIM_DMA_ID_CC1];
    newAddress = (uint32_t)(*preparedBufferCh1);
    preparedBufferCh1 = NULL;
  }
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
    if (preparedBufferCh4 == NULL)
    {
      // No buffer is prepared
      // in practice, the pulse generation should be stopped
      return;
    }

    hdma = htim->hdma[TIM_DMA_ID_CC4];
    newAddress = (uint32_t)(*preparedBufferCh4);
    preparedBufferCh4 = NULL;
  }
  else
    return;

  // Update the memory 1 address
  if (TIM_OC_Update_DMA_Double_Buffer(hdma, newAddress, SELECT_DOUBLE_BUFFER_M1) != HAL_OK)
  {
    Error_Handler();
  }
}
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
