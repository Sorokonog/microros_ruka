/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

//#include <std_msgs/msg/int32.h>
//#include <std_msgs/msg/u_int32.h>
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

std_msgs__msg__String msg, msg_input;
sensor_msgs__msg__JointState js_out, js_in;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_TxHeaderTypeDef   TxHeader;
extern FDCAN_RxHeaderTypeDef   RxHeader;
extern uint8_t TxData[8];
uint8_t RxData[8];

long indx = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 10000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void string_cb(const void * msg_input);

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);



/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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
	rcl_init_options_t init_options;

	rmw_uros_set_custom_transport(
			    true,
			    (void *) &hfdcan1,
			    cubemx_transport_open,
			    cubemx_transport_close,
			    cubemx_transport_write,
			    cubemx_transport_read);

	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	//create init_options
	allocator = rcl_get_default_allocator();
	init_options = rcl_get_zero_initialized_init_options();
	rcl_init_options_init(&init_options, allocator);

	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	rmw_uros_options_set_client_key(CLIENT_KEY, rmw_options);

	//support init
	rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);


	char node_name[13];
	char topic_pub_name[18];
	char topic_sub_name[18];
	char joint_name[8];

	sprintf(node_name,"joint_node_%d", MY_CAN_ID);
	sprintf(topic_pub_name,"joint_state_pub_%d", MY_CAN_ID);
	sprintf(topic_sub_name,"joint_state_sub_%d", MY_CAN_ID);
	sprintf(joint_name,"joint_%d", MY_CAN_ID);


	// create node
	rclc_node_init_default(&node, node_name, "", &support);

	// create publisher
	rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
		topic_pub_name);

	rcl_ret_t rc = rclc_subscription_init_default(
		&subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
		topic_sub_name);

	executor = rclc_executor_get_zero_initialized_executor();
	rc = rclc_executor_init(&executor, &support.context, 1, &allocator);

	rc = rclc_executor_add_subscription(
		&executor, &subscriber, &js_in,
		//&executor, &subscriber, &msg_input,
		&string_cb, ON_NEW_DATA);
		//&motor_controller_cb, ON_NEW_DATA);

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
	js_out.name.data[0].data = joint_name;


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



	//msg.data  = malloc(10*sizeof(char));
	//msg_input.data  = malloc(10*sizeof(char));
	/* Infinite loop */

	  for(;;)
	  {


		js_out.position.data[0] = 1.0; //kalman_angle;
		js_out.velocity.data[0] = 1.0; //pvelocity_to_go;
		js_out.effort.data[0] = 1.0; //prev_effort;


		rcl_ret_t ros_ret = rcl_publish(&publisher, &js_out, NULL);

	    if (ros_ret != RCL_RET_OK)
	    {
	      printf("Error publishing (line %d)\n", __LINE__);
	    }

	    rclc_executor_spin_some(&executor, 1);
	    vTaskDelay(1);
	  }

  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void string_cb(const void * msg_input)
{
}



/* USER CODE END Application */

