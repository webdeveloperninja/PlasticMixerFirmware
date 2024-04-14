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
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TRUE 1
#define FALSE 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

void StartAdditiveMaterialMotorClockwise();
void StopAdditiveMaterialMotor();
void CalibrateVirginMaterialFlow();
void SetCalibrationValveOpen();
void SetCalibrationValveClosed();
void StartFeed();
void StopFeed();
void StartAdditiveMotorTask(void *argument);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// SET FROM CALIBRATION PROCESS
int VirginMaterialGramsPerSecond = 0;

// USER INPUT FROM HMI
int VirginMaterialGramsPerCup = 30;

// USER INPUT FROM HMI
int AdditivePercentRate = 2;

// SET FROM CALIBRATION
int AdditiveMaterialMotorStepsPerSecond = 100;

int CalculateAdditiveMaterialMotorSpeedIntervalSeconds = 300;

// This will be the digital out for a relay to power 24 V pump
int CalibrationValveOpen = FALSE;

// This will be a limit switch state.
int CalibrationTubeFull = FALSE;


TaskHandle_t xStartAdditiveMotorTaskHandle = NULL;

void StartFeed()
{
    if (xStartAdditiveMotorTaskHandle == NULL)
    {
        xTaskCreate(
        	StartAdditiveMotorTask,
			"StartAdditiveMotorTask",
			configMINIMAL_STACK_SIZE,
			NULL,
			2,
			&xStartAdditiveMotorTaskHandle
		);
    }
}

void StopFeed()
{
	StopAdditiveMaterialMotor();

	if (xStartAdditiveMotorTaskHandle != NULL)
	{
		vTaskDelete(xStartAdditiveMotorTaskHandle);
		xStartAdditiveMotorTaskHandle = NULL;
	}
}

void StartAdditiveMotorTask(void *argument)
{
    const TickType_t calculateAdditiveMaterialMotorSpeedInterval = CalculateAdditiveMaterialMotorSpeedIntervalSeconds * 1000 / portTICK_PERIOD_MS;

    while (1)
    {
        CalibrateVirginMaterialFlow();
        StartAdditiveMaterialMotorClockwise();

        printf("Virgin Material Flow Calibration Task\n");
        vTaskDelay(calculateAdditiveMaterialMotorSpeedInterval);
    }
}

void CalibrateVirginMaterialFlow() {
    TickType_t xStartTime, xEndTime;
    int timeToFillMilliseconds;

    SetCalibrationValveOpen();

    xStartTime = xTaskGetTickCount();

    while (CalibrationTubeFull == FALSE) {
    	// DELAY Polling Calibration Tube Full to save CPU cycles
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    xEndTime = xTaskGetTickCount();

    SetCalibrationValveClosed();

    timeToFillMilliseconds = (xEndTime - xStartTime) * portTICK_PERIOD_MS;
    VirginMaterialGramsPerSecond = (float) VirginMaterialGramsPerCup / (timeToFillMilliseconds / 1000.0f);

    printf("Virgin Material Flow Rate: %d grams/second\n", VirginMaterialGramsPerSecond);
}


void StartAdditiveMaterialMotorClockwise()
{
	// TODO: Implement
}


void StopAdditiveMaterialMotor()
{
	// TODO: Implement
}

void SetCalibrationValveOpen()
{
	// TODO: Implement
	CalibrationValveOpen = TRUE;
}

void SetCalibrationValveClosed()
{
	// TODO: Implement
	CalibrationValveOpen = FALSE;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13)
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
        {
            CalibrationTubeFull = TRUE;
            printf("Button Pressed - CalibrationTubeFull set to TRUE\n");
        }
        else // Pin is low (button released)
        {
            CalibrationTubeFull = FALSE;
            printf("Button Released - CalibrationTubeFull set to FALSE\n");
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

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

  /* USER CODE BEGIN RTOS_THREADS */

  // TODO: This will move to an interrupt triggering on HMI Feed button
  StartFeed();

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
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
