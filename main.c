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
#include "can.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim3;

uint8_t TxData[8];					// Data to be sent via CAN
uint32_t TxMailbox;					// CAN temporary mailbox. Required by HAL function
CAN_TxHeaderTypeDef TxHeader;		// Header for can message
CAN_FilterTypeDef canfilterconfig;	// Filter for receiving CAN messages
uint8_t RxData[8];					// data received from can bus
CAN_RxHeaderTypeDef   RxHeader;		// header received by can bus

int CAN_Header_Config(CAN_TxHeaderTypeDef* TxHeader);	// Configures CAN message header.
int CAN_Filter_Config(CAN_FilterTypeDef* canfilterconfig);	// COnfigures filter for CAN message reception
int CAN_Starter(CAN_HandleTypeDef* hcan,CAN_FilterTypeDef* canfilterconfig);	//Starts CAN transmitions and receptions

void CTRL_Reg_Set();
void TORQUE_Reg_Set();
void STATUS_Reg_Set();
uint16_t RegAccess(uint8_t operation, uint8_t address, uint16_t value); // Sends or receives SPI message

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t stepper_step_denominator = 1;
uint32_t pwm_tick_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
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
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  // initialize motor
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

	HAL_Delay(1);
	TORQUE_Reg_Set();
	HAL_Delay(1);
	CTRL_Reg_Set();

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  //CAN START TODO should move it to dma_transport.c in init func
  CAN_Header_Config(&TxHeader);		// Sets header values
  CAN_Filter_Config(&canfilterconfig);// Seds filter values
  CAN_Starter(&hcan1, &canfilterconfig); // starts can

  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int CAN_Filter_Config(CAN_FilterTypeDef* canfilterconfig){
	canfilterconfig->FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig->FilterBank = 0;  // which filter bank to use from the assigned ones
	canfilterconfig->FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig->FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig->FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig->FilterIdHigh = 0x0000;
	canfilterconfig->FilterIdLow = 0x0000;
	canfilterconfig->FilterMaskIdHigh = 0x0000;
	canfilterconfig->FilterMaskIdLow = 0x0000;
	canfilterconfig->SlaveStartFilterBank = 20;
	return 1;
}

int CAN_Header_Config(CAN_TxHeaderTypeDef* TxHeader){
	TxHeader->IDE = CAN_ID_STD;
	TxHeader->StdId = MY_CAN_ID;
	TxHeader->RTR = CAN_RTR_DATA;
	TxHeader->DLC = 8;
	return 1;
}

int CAN_Starter(CAN_HandleTypeDef* hcan,CAN_FilterTypeDef* canfilterconfig){
	if (HAL_CAN_ConfigFilter(hcan, canfilterconfig) != HAL_OK){		// attempting to register can filter
			Error_Handler();
		}
		if (HAL_CAN_Start(hcan) != HAL_OK){		// attempting to start CAN
			Error_Handler();
		}
		if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){	// attempting to enable CAN interrupts.
			Error_Handler();								// CAN1_RX0_IRQHandler() is called when an interrupt is triggered
		}
	return 1;
}


void CTRL_Reg_Set(){
	uint16_t TX = 0x0000;
	TX += (0x03 << 10); // 850 ns dead time
	TX += (0x03 << 8); // Gain of 40
	TX += (((int)sqrt(stepper_step_denominator)) << 3); // 1/4 stepn
	//TX += ((int)(stepper_step_denominator) << 3); // 1/4 stepn
	TX += 0x01 ; // Enable motor
	RegAccess(WRITE, 0x00, TX); // write CTRL Register (Address = 0x00)
	return;
}

void TORQUE_Reg_Set(){
	uint16_t TX = 0x0000;
	TX += (0x01 << 8); // sample time = 100 us
	TX += 0x00; // Torque = 0x3F
//	TX = 0b0001000100000001;
	RegAccess(WRITE, 0x01, TX); // write TORQUE Register (Address = 0x01)
	return;
}

void STATUS_Reg_Set(){
	RegAccess(WRITE, 0x07, 0x0000); // write STATUS Register (Address = 0x00)
}

uint16_t RegAccess(uint8_t operation, uint8_t address, uint16_t value)
{
  uint16_t parcel = value;

  parcel += (address << 12); // register address
  parcel += (operation << 15); // read-write operation choice

  uint16_t received = 0;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
 // HAL_Delay(1);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&parcel, (uint8_t*)&received, 1, 1000);
 // HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
 // HAL_Delay(1);
  received &= ~0xF000; // clear upper 4 bits, leave lower 12 bits

  return received;
}



/* USER CODE END 4 */

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
