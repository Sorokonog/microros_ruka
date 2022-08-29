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

#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/joint_state.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// ROS entities
std_msgs__msg__UInt32 msg_s, msg_p, msg;
sensor_msgs__msg__JointState js_in, js_out;

extern long pwm_tick_counter;

double angle_to_go = 0;
double velocity_to_go = 0;
double effort_to_go = 0;
double curent_angle; //TODO Kalman angle
double curent_velocity = 0; //TODO Kalman vel
double curent_effort = 0;

uint32_t read_fault = 0; //read fault WD


extern uint8_t TxData[8];					// Data to be sent via CAN
extern uint64_t TxMailbox;					// CAN temporary mailbox. Required by HAL function
extern CAN_TxHeaderTypeDef TxHeader;		// Header for can message
extern CAN_FilterTypeDef canfilterconfig;	// Filter for receiving CAN messages
extern uint8_t RxData[8];					// data received from can bus
extern CAN_RxHeaderTypeDef   RxHeader;		// header received by can bus

extern CAN_HandleTypeDef hcan1;
extern I2C_HandleTypeDef hi2c1;



/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for I2CTask */
osThreadId_t I2CTaskHandle;
const osThreadAttr_t I2CTask_attributes = {
  .name = "I2CTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorController */
osThreadId_t MotorControllerHandle;
const osThreadAttr_t MotorController_attributes = {
  .name = "MotorController",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void int_cb(const void *);
double kalman_angle();
bool create_entities();
void destroy_entities();
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void I2CTask02(void *argument);
void MotorControllerTask03(void *argument);

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

  /* creation of I2CTask */
  I2CTaskHandle = osThreadNew(I2CTask02, NULL, &I2CTask_attributes);

  /* creation of MotorController */
  MotorControllerHandle = osThreadNew(MotorControllerTask03, NULL, &MotorController_attributes);

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
	rcl_allocator_t allocator;
	rcl_node_t node;


	rmw_uros_set_custom_transport(
			    true,
			    (void *) &hcan1,
			    cubemx_transport_open,
			    cubemx_transport_close,
			    cubemx_transport_write,
			    cubemx_transport_read);

			  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
			  freeRTOS_allocator.allocate = microros_allocate;
			  freeRTOS_allocator.deallocate = microros_deallocate;
			  freeRTOS_allocator.reallocate = microros_reallocate;
			  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

			  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
			      printf("Error on default allocators (line %d)\n", __LINE__);
			  }

			  allocator = rcl_get_default_allocator();

			  //create init_options
			  rclc_support_init(&support, 0, NULL, &allocator);

			  // create node
			  rclc_node_init_default(&node, "joint_1_node", "", &support);

			  // create publisher
			  rclc_publisher_init_default(
			    &publisher,
			    &node,
			    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
			    "joint_state_pub");

			    rcl_ret_t rc = rclc_subscription_init_default(
			    &subscriber, &node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
				"joint_state_sub");

			executor = rclc_executor_get_zero_initialized_executor();
			rc = rclc_executor_init(&executor, &support.context, 1, &allocator);

			rc = rclc_executor_add_subscription(
			  &executor, &subscriber, &js_in,
			  &int_cb, ON_NEW_DATA);

			//MSG data filling

				js_out.position.size=1;
				js_out.position.capacity=1;
				js_out.position.data = malloc(js_out.position.capacity*sizeof(double));
				js_out.position.data[0] = 0;
				js_out.velocity.size=1;
				js_out.velocity.capacity=1;
				js_out.velocity.data = malloc(js_out.velocity.capacity*sizeof(double));
				js_out.velocity.data[0] = 0;
				js_out.effort.size=1;
				js_out.effort.capacity=1;
				js_out.effort.data = malloc(js_out.effort.capacity*sizeof(double));
				js_out.effort.data[0] = 0;
				js_out.name.capacity = 1;
				js_out.name.size = 1;
				js_out.name.data = (std_msgs__msg__String*) malloc(js_out.name.capacity*sizeof(std_msgs__msg__String));
				js_out.name.data[0].data = "ruka";


				js_in.position.size=1;
				js_in.position.capacity=1;
				js_in.position.data = malloc(js_in.position.capacity*sizeof(double));
				js_in.position.data[0] = 0;
				js_in.velocity.size=1;
				js_in.velocity.capacity=1;
				js_in.velocity.data = malloc(js_in.velocity.capacity*sizeof(double));
				js_in.velocity.data[0] = 0;
				js_in.effort.size=1;
				js_in.effort.capacity=1;
				js_in.effort.data = malloc(js_in.effort.capacity*sizeof(double));
				js_in.effort.data[0] = 0;
				js_in.name.capacity = 1;
				js_in.name.size = 1;
				js_in.name.data = (std_msgs__msg__String*) malloc(js_in.name.capacity*sizeof(std_msgs__msg__String));


	/* Infinite loop */

	  for(;;)
	  {

		js_out.position.data[0] = curent_angle;
		js_out.velocity.data[0] = pwm_tick_counter;
		js_out.effort.data[0] = effort_to_go;

		rcl_ret_t ret = rcl_publish(&publisher, &js_out, NULL);
	    if (ret != RCL_RET_OK)
	    {
	      printf("Error publishing (line %d)\n", __LINE__);
	    }

	    rclc_executor_spin_some(&executor, 1);
	    vTaskDelay(1);
	  }


  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_I2CTask02 */
/**
* @brief Function implementing the I2CTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_I2CTask02 */
void I2CTask02(void *argument)
{
  /* USER CODE BEGIN I2CTask02 */
	  HAL_StatusTypeDef ret;
	  uint8_t buf[12];	// temporaty buffer for i2c message transmition
	  uint8_t buf_recv[12];	// temporaty buffer for i2c message reception
  /* Infinite loop */
	  for(;;)
	  {
			buf[0] = 0x0E;
			ret = HAL_I2C_Master_Transmit(&hi2c1, (0b0110110 << 1) , buf, 1, HAL_MAX_DELAY);
			if(ret == HAL_OK ){
			}
			ret = HAL_I2C_Master_Receive(&hi2c1, 0b0110110 << 1, buf_recv, 2, HAL_MAX_DELAY);
			if(ret == HAL_OK ){
			}
			curent_angle = (2 * M_PI * (buf_recv[1] + 256 * buf_recv[0]) / 4096);
		vTaskDelay(10);
	  }
  /* USER CODE END I2CTask02 */
}

/* USER CODE BEGIN Header_MotorControllerTask03 */
/**
* @brief Function implementing the MotorController thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorControllerTask03 */
void MotorControllerTask03(void *argument)
{
  /* USER CODE BEGIN MotorControllerTask03 */
  /* Infinite loop */
  for(;;)
  {
	 if (read_fault > 10)
	 {
		 NVIC_SystemReset();
	 }
    vTaskDelay(500);
  }
  /* USER CODE END MotorControllerTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
double kalman_angle()  //TODO KALMAN
{
	return 2.0;
}


void int_cb(const void * msgin)
{
// Cast received message to used type
const sensor_msgs__msg__JointState * js_in = (const sensor_msgs__msg__JointState *)msgin;
angle_to_go = js_in->position.data[0];
velocity_to_go = js_in->velocity.data[0];
effort_to_go = js_in->effort.data[0];
}

/* USER CODE END Application */

