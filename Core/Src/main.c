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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for StartGreenLed */
osThreadId_t StartGreenLedHandle;
const osThreadAttr_t StartGreenLed_attributes = {
  .name = "StartGreenLed",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for StartRedLed */
osThreadId_t StartRedLedHandle;
const osThreadAttr_t StartRedLed_attributes = {
  .name = "StartRedLed",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for StartOrangeLed */
osThreadId_t StartOrangeLedHandle;
const osThreadAttr_t StartOrangeLed_attributes = {
  .name = "StartOrangeLed",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for CriticalResourceSemaphore */
osSemaphoreId_t CriticalResourceSemaphoreHandle;
const osSemaphoreAttr_t CriticalResourceSemaphore_attributes = {
  .name = "CriticalResourceSemaphore"
};
/* USER CODE BEGIN PV */
uint8_t StartFlag; // 1 means Up, 0 means Down
volatile uint8_t RedFlag; // 1 means Up, 0 means Down
volatile uint8_t GreenFlag; // 1 means Up, 0 means Down
uint16_t WaitTimeMilliseconds = 550;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);
void GreenLedTask(void *argument);
void RedLedTask(void *argument);
void OrangeLedTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	StartFlag = 1;
	GreenFlag = 1;
	RedFlag = 1;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of CriticalResourceSemaphore */
  CriticalResourceSemaphoreHandle = osSemaphoreNew(1, 1, &CriticalResourceSemaphore_attributes);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of StartGreenLed */
  StartGreenLedHandle = osThreadNew(GreenLedTask, NULL, &StartGreenLed_attributes);

  /* creation of StartRedLed */
  StartRedLedHandle = osThreadNew(RedLedTask, NULL, &StartRedLed_attributes);

  /* creation of StartOrangeLed */
  StartOrangeLedHandle = osThreadNew(OrangeLedTask, NULL, &StartOrangeLed_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void AccessSharedData(void) {
    if (StartFlag == 1) {
        // Set Start flag to Down to indicate resource is in use
        StartFlag = 0;
    } else {
        // Resource contention: Turn on Blue LED
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    }

    // Simulate read/write operations with a delay of 500 milliseconds
    // SimulateReadWriteOperation();
    osDelay(500);

    // Set Start flag back to Up to indicate resource is free
    StartFlag = 1;

    // Turn off Blue LED (if it was turned on during contention)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
}

void SimulateReadWriteOperation(void) {
    volatile uint32_t delay_count = 0;
    const uint32_t delay_target = 5000000; // Adjust this value to approximate 500 ms

    // Dummy loop to simulate processing time
    for (delay_count = 0; delay_count < delay_target; delay_count++) {
        __asm("nop"); // No Operation: Keeps the processor busy without changing code behavior
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_GreenLedTask */
/**
* @brief Function implementing the StartGreenLed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GreenLedTask */
void GreenLedTask(void *argument)
{
  /* USER CODE BEGIN GreenLedTask */
  /* Infinite loop */
  for(;;)
  {
	  GreenFlag = 1;
	  // Turn on Green LED
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

	  // Enter critical section
	  osSemaphoreAcquire(CriticalResourceSemaphoreHandle,WaitTimeMilliseconds);
	  AccessSharedData();
	  osSemaphoreRelease (CriticalResourceSemaphoreHandle);

	  // Turn off Green LED
	  osDelay(200);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

	  GreenFlag = 0;
	  // Delay
	  osDelay(200);
  }
  /* USER CODE END GreenLedTask */
}

/* USER CODE BEGIN Header_RedLedTask */
/**
* @brief Function implementing the StartRedLed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RedLedTask */
void RedLedTask(void *argument)
{
  /* USER CODE BEGIN RedLedTask */
  /* Infinite loop */
  for(;;)
  {
      RedFlag = 1;
      // Turn on Red LED
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

      // Enter critical section
      osSemaphoreAcquire(CriticalResourceSemaphoreHandle,WaitTimeMilliseconds);
      AccessSharedData();
      osSemaphoreRelease (CriticalResourceSemaphoreHandle);

      // Turn off Red LED
      osDelay(550);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

      RedFlag = 0;
      // Delay
      osDelay(550);
  }
  /* USER CODE END RedLedTask */
}

/* USER CODE BEGIN Header_OrangeLedTask */
/**
* @brief Function implementing the StartOrangeLed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OrangeLedTask */
void OrangeLedTask(void *argument)
{
  /* USER CODE BEGIN OrangeLedTask */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
	  osDelay(50);
  }
  /* USER CODE END OrangeLedTask */
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

#ifdef  USE_FULL_ASSERT
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
