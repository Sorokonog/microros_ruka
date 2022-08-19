/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

//#include <std_msgs/msg/u_int16.h>
#include <std_msgs/msg/u_int32.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
extern CAN_HandleTypeDef hcan1;

extern I2C_HandleTypeDef hi2c1;

//extern UART_HandleTypeDef huart2;
//extern DMA_HandleTypeDef hdma_usart2_rx;
//extern DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

extern bool cubemx_transport_open(struct uxrCustomTransport * transport);
extern bool cubemx_transport_close(struct uxrCustomTransport * transport);
extern size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
extern size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

/*
extern bool canfd_transport_open(struct uxrCustomTransport * transport);
extern bool canfd_transport_close(struct uxrCustomTransport * transport);
extern size_t canfd_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
extern size_t canfd_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);
*/

extern void * microros_allocate(size_t size, void * state);
extern void microros_deallocate(void * pointer, void * state);
extern void * microros_reallocate(void * pointer, size_t size, void * state);
extern void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

uint32_t timcounter = 0;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint8_t TxData[8];					// Data to be sent via CAN
extern uint64_t TxMailbox;					// CAN temporary mailbox. Required by HAL function
extern CAN_TxHeaderTypeDef TxHeader;		// Header for can message
extern CAN_FilterTypeDef canfilterconfig;	// Filter for receiving CAN messages

uint8_t buf[12];	// temporaty buffer for i2c message transmition
uint8_t buf_recv[12];	// temporaty buffer for i2c message reception

extern uint8_t RxData[8];					// data received from can bus
extern CAN_RxHeaderTypeDef   RxHeader;		// header received by can bus


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 10000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CANtask */
osThreadId_t CANtaskHandle;
const osThreadAttr_t CANtask_attributes = {
  .name = "CANtask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for I2CTask */
osThreadId_t I2CTaskHandle;
const osThreadAttr_t I2CTask_attributes = {
  .name = "I2CTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LogicTask */
osThreadId_t LogicTaskHandle;
const osThreadAttr_t LogicTask_attributes = {
  .name = "LogicTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void int_cb(const void *);
std_msgs__msg__UInt32 msg_s, msg_p, msg;

void CANtask01(void *argument);
void I2CTask01(void *argument);
void LogicTask01(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */


/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CANtask */
  CANtaskHandle = osThreadNew(CANtask01, NULL, &CANtask_attributes);

  /* creation of I2CTask */
  I2CTaskHandle = osThreadNew(I2CTask01, NULL, &I2CTask_attributes);

  /* creation of LogicTask */
  LogicTaskHandle = osThreadNew(LogicTask01, NULL, &LogicTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	rcl_publisher_t publisher;
	rcl_subscription_t subscriber;
	rclc_executor_t executor;
	rclc_support_t support;
	//rcl_allocator_t allocator;
	rcl_node_t node;

	rmw_uros_set_custom_transport(
			    true,
			    (void *) &hcan1,
			    cubemx_transport_open,
			    cubemx_transport_close,
			    cubemx_transport_write,
			    cubemx_transport_read);

			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
			  /*

			   rmw_uros_set_custom_transport(
			    true,
			    (void *) &hcan1,
				canfd_transport_open,
				canfd_transport_close,
				canfd_transport_write,
				canfd_transport_read);
			   */

			  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
			  freeRTOS_allocator.allocate = microros_allocate;
			  freeRTOS_allocator.deallocate = microros_deallocate;
			  freeRTOS_allocator.reallocate = microros_reallocate;
			  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

			  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
			      printf("Error on default allocators (line %d)\n", __LINE__);
			  }

			  // micro-ROS app

			  rcl_allocator_t allocator = rcl_get_default_allocator();

			  //create init_options
			  rclc_support_init(&support, 0, NULL, &allocator);

			  // create node
			  rclc_node_init_default(&node, "cubemx_node", "", &support);

			  // create publisher
			  rclc_publisher_init_default(
			    &publisher,
			    &node,
			    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
			    "cubemx_publisher");

			  rcl_ret_t rc = rclc_subscription_init_default(
			    &subscriber, &node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
				"int_sub");

			executor = rclc_executor_get_zero_initialized_executor();
			rc = rclc_executor_init(&executor, &support.context, 1, &allocator);

			rc = rclc_executor_add_subscription(
			  &executor, &subscriber, &msg_s,
			  &int_cb, ON_NEW_DATA);


			  msg.data = 0;

	/* Infinite loop */

	  for(;;)
	  {
		msg.data = buf_recv[1] + 256 * buf_recv[0];
		//msg.data = timcounter;

		rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
	    if (ret != RCL_RET_OK)
	    {
	      printf("Error publishing (line %d)\n", __LINE__);
	    }

	    rclc_executor_spin_some(&executor, 1);

	    vTaskDelay(1);
	  }


  /* USER CODE END StartDefaultTask */
}


// Implementation example:
void int_cb(const void * msgin)
{
// Cast received message to used type
const std_msgs__msg__UInt32 * msg_s = (const std_msgs__msg__UInt32 *)msgin;
HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);

}

/* USER CODE BEGIN Header_CANtask01 */
/**
* @brief Function implementing the CANtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CANtask01 */
void CANtask01(void *argument)
{
  /* USER CODE BEGIN CANtask01 */

  /* Infinite loop */
  for(;;)
  {
	  //rclc_executor_spin_some(&executor, 1);
	  vTaskDelay(1000);
  }
  /* USER CODE END CANtask01 */
}

/* USER CODE BEGIN Header_I2CTask01 */
/**
* @brief Function implementing the I2CTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_I2CTask01 */
void I2CTask01(void *argument)
{
  /* USER CODE BEGIN I2CTask01 */
  /* Infinite loop */

  HAL_StatusTypeDef ret;

  for(;;)
  {
		buf[0] = 0x0E;
		ret = HAL_I2C_Master_Transmit(&hi2c1, (0b0110110 << 1) , buf, 1, HAL_MAX_DELAY);
		if(ret == HAL_OK ){
		}
		ret = HAL_I2C_Master_Receive(&hi2c1, 0b0110110 << 1, buf_recv, 2, HAL_MAX_DELAY);
		if(ret == HAL_OK ){
		}
	vTaskDelay(10);
  }
  /* USER CODE END I2CTask01 */
}

/* USER CODE BEGIN Header_LogicTask01 */
/**
* @brief Function implementing the LogicTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LogicTask01 */
void LogicTask01(void *argument)
{
  /* USER CODE BEGIN LogicTask01 */
  /* Infinite loop */
  for(;;)
  {
	  vTaskDelay(1000);
  }
  /* USER CODE END LogicTask01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

