/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "print.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for HeartBeatTask */
osThreadId_t HeartBeatTaskHandle;
uint32_t HeartBeatTaskBuffer[ 128 ];
osStaticThreadDef_t HeartBeatTaskControlBlock;
const osThreadAttr_t HeartBeatTask_attributes = {
  .name = "HeartBeatTask",
  .cb_mem = &HeartBeatTaskControlBlock,
  .cb_size = sizeof(HeartBeatTaskControlBlock),
  .stack_mem = &HeartBeatTaskBuffer[0],
  .stack_size = sizeof(HeartBeatTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CANReceiveTask */
osThreadId_t CANReceiveTaskHandle;
uint32_t CANReceiveTaskBuffer[ 256 ];
osStaticThreadDef_t CANReceiveTaskControlBlock;
const osThreadAttr_t CANReceiveTask_attributes = {
  .name = "CANReceiveTask",
  .cb_mem = &CANReceiveTaskControlBlock,
  .cb_size = sizeof(CANReceiveTaskControlBlock),
  .stack_mem = &CANReceiveTaskBuffer[0],
  .stack_size = sizeof(CANReceiveTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTimer01 */
osTimerId_t myTimer01Handle;
osStaticTimerDef_t myTimer01ControlBlock;
const osTimerAttr_t myTimer01_attributes = {
  .name = "myTimer01",
  .cb_mem = &myTimer01ControlBlock,
  .cb_size = sizeof(myTimer01ControlBlock),
};
/* USER CODE BEGIN PV */
uint8_t ubKeyNumber = 0x0;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox=0;

CAN_FilterTypeDef sFilterConfig;

#define PRINT_BUFFER_SIZE 256
char printBuffer[PRINT_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
void StartDefaultTask(void *argument);
void _HeartBeatTask(void *argument);
void _CANReceiveTask(void *argument);
void Callback01(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void __print_service(const char *str) {
    HAL_UART_Transmit(&huart2, (uint8_t*) str, strlen(str), 100);
}

//void _printf(const char *format, va_list args) {
////	va_list args;
////	va_start(args, format);
//
//	if(strlen(format) >= PRINT_BUFFER_SIZE) {
//		const char* errorMessage = "ERROR: format argument has lenght biger than expected len!\n";
//		_print(errorMessage);
//		_print(format);
//		_print("\r\n");
//		return;
//	}
//
//	int len = vsnprintf(printBuffer, PRINT_BUFFER_SIZE, format, args);
////	va_end(args);
//
//	if(len > 0) {
//		_print(printBuffer);
//	}else{
//		const char* errorMessage = "ERROR: To large string to format";
//		_print(errorMessage);
//		_print(format);
//		_print("\r\n");
//	}
//}
//
//void println(const char *format, ...) {
//	va_list args;
//	va_start(args, format);
//	_printf(format, args);
//	va_end(args);
//	_print("\r\n");
//}
//
//void print(const char *format, ...) {
//	va_list args;
//	va_start(args, format);
//	_printf(format, args);
//	va_end(args);
//}
//
////void print(const char *format, ...) {
////	va_list args;
////	va_start(args, format);
////
////	if(strlen(format) >= PRINT_BUFFER_SIZE) {
////		const char* errorMessage = "ERROR: format argument has lenght biger than expected len!\n";
////		_print(errorMessage);
////		_print(format);
////		_print("\r\n");
////		return;
////	}
////
////	int len = vsnprintf(printBuffer, PRINT_BUFFER_SIZE, format, args);
////	va_end(args);
////
////	if(len > 0) {
////		_print(printBuffer);
////	}else{
////		const char* errorMessage = "ERROR: To large string to format";
////		_print(errorMessage);
////		_print(format);
////		_print("\r\n");
////	}
////}
#define CAN_ERROR_MSG_MAPPING_LEN 23

struct {
	uint32_t error_id;
	const char* human_readable_msg;
} can_error_msg_mapping[CAN_ERROR_MSG_MAPPING_LEN] = {
{HAL_CAN_ERROR_NONE            , "No error                                            "},
{HAL_CAN_ERROR_EWG             , "Protocol Error Warning                              "},
{HAL_CAN_ERROR_EPV             , "Error Passive                                       "},
{HAL_CAN_ERROR_BOF             , "Bus-off error                                       "},
{HAL_CAN_ERROR_STF             , "Stuff error                                         "},
{HAL_CAN_ERROR_FOR             , "Form error                                          "},
{HAL_CAN_ERROR_ACK             , "Acknowledgment error                                "},
{HAL_CAN_ERROR_BR              , "Bit recessive error                                 "},
{HAL_CAN_ERROR_BD              , "Bit dominant error                                  "},
{HAL_CAN_ERROR_CRC             , "CRC error                                           "},
{HAL_CAN_ERROR_RX_FOV0         , "Rx FIFO0 overrun error                              "},
{HAL_CAN_ERROR_RX_FOV1         , "Rx FIFO1 overrun error                              "},
{HAL_CAN_ERROR_TX_ALST0        , "TxMailbox 0 transmit failure due to arbitration lost"},
{HAL_CAN_ERROR_TX_TERR0        , "TxMailbox 0 transmit failure due to transmit error  "},
{HAL_CAN_ERROR_TX_ALST1        , "TxMailbox 1 transmit failure due to arbitration lost"},
{HAL_CAN_ERROR_TX_TERR1        , "TxMailbox 1 transmit failure due to transmit error  "},
{HAL_CAN_ERROR_TX_ALST2        , "TxMailbox 2 transmit failure due to arbitration lost"},
{HAL_CAN_ERROR_TX_TERR2        , "TxMailbox 2 transmit failure due to transmit error  "},
{HAL_CAN_ERROR_TIMEOUT         , "Timeout error                                       "},
{HAL_CAN_ERROR_NOT_INITIALIZED , "Peripheral not initialized                          "},
{HAL_CAN_ERROR_NOT_READY       , "Peripheral not ready                                "},
{HAL_CAN_ERROR_NOT_STARTED     , "Peripheral not started                              "},
{HAL_CAN_ERROR_PARAM           ,"Parameter error                                      "}
};


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
  print_init(__print_service);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  /* Set the data to be transmitted */
  int pendingRequests = HAL_CAN_IsTxMessagePending(&hcan, TxMailbox);
  if (pendingRequests == 0) {
      TxData[0] = ubKeyNumber;
      TxData[1] = 0xAD;
      if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
          /* Transmission request Error */
          Error_Handler();
      }
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of myTimer01 */
  myTimer01Handle = osTimerNew(Callback01, osTimerPeriodic, NULL, &myTimer01_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of HeartBeatTask */
  HeartBeatTaskHandle = osThreadNew(_HeartBeatTask, NULL, &HeartBeatTask_attributes);

  /* creation of CANReceiveTask */
  CANReceiveTaskHandle = osThreadNew(_CANReceiveTask, NULL, &CANReceiveTask_attributes);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 32;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
    /*##-2- Configure the CAN Filter ###########################################*/
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14; // how many filters to assign to the CAN1 (master can)

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }

    /*##-3- Start the CAN peripheral ###########################################*/
    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        /* Start Error */
        Error_Handler();
    }

    const uint32_t error_filter = CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF |
			CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR;

    if (HAL_CAN_ActivateNotification(&hcan, error_filter) != HAL_OK) {

        /* Notification Error */
        Error_Handler();
    }

    /*##-5- Configure Transmission process #####################################*/
    TxHeader.StdId = 0x321;
    TxHeader.ExtId = 0x01;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 2;
    TxHeader.TransmitGlobalTime = DISABLE;

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

/* USER CODE BEGIN Header__HeartBeatTask */
/**
* @brief Function implementing the HeartBeatTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__HeartBeatTask */
void _HeartBeatTask(void *argument)
{
  /* USER CODE BEGIN _HeartBeatTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
    int pendingRequests = HAL_CAN_IsTxMessagePending(&hcan, TxMailbox);
    if (pendingRequests == 0) {
    	TxHeader.DLC = 1;
    	TxHeader.StdId = 0x001;
    	TxHeader.IDE = CAN_ID_STD;
    	RxData[0] = 0xAA;
    	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, RxData, &TxMailbox) != HAL_OK) {
            Error_Handler();
        }
    }
  }
  /* USER CODE END _HeartBeatTask */
}

/* USER CODE BEGIN Header__CANReceiveTask */
/**
* @brief Function implementing the CANReceiveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__CANReceiveTask */
void _CANReceiveTask(void *argument)
{
  /* USER CODE BEGIN _CANReceiveTask */
  /* Infinite loop */
    while (1) {
    	uint32_t error = HAL_CAN_GetError(&hcan);
    	if(error != HAL_CAN_ERROR_NONE) {
    		for(int i = 0; i < CAN_ERROR_MSG_MAPPING_LEN; i++){
    			if(error & can_error_msg_mapping[i].error_id) {
    				print("CAN Error is: ");
    				println("%s", can_error_msg_mapping[i].human_readable_msg);
    			}
    		}
    		HAL_CAN_ResetError(&hcan);
    	}

        uint32_t RxFifo = 0;
        uint32_t fillLevel = HAL_CAN_GetRxFifoFillLevel(&hcan, RxFifo);
        if (fillLevel > 0) {
            HAL_CAN_GetRxMessage(&hcan, RxFifo, &RxHeader, RxData);
            if(RxHeader.IDE == CAN_ID_STD){
                println("RX: StdId=%x; DLC=%d", RxHeader.StdId, RxHeader.DLC);
            }else{
                println("RX: ExtId=%x; DLC=%d", RxHeader.ExtId, RxHeader.DLC);
            }
            println("RX: %x%x%x%x%x%x%x%x", RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7]);
        }
        osDelay(100);

    }
  /* USER CODE END _CANReceiveTask */
}

/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */

  /* USER CODE END Callback01 */
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
    while (1) {
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
