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
//i2c vars
HAL_StatusTypeDef i2c_ret;
uint8_t buf[12];	// temporaty buffer for i2c message transmition
uint8_t buf_recv[12];	// temporaty buffer for i2c message reception

// ROS entities
std_msgs__msg__UInt32 msg_s, msg_p, msg;
sensor_msgs__msg__JointState js_in, js_out;

extern long pwm_tick_counter;
long ticks_to_go = 0;
long pwm_timer_period = IDLE_PWM_TIMER_PERIOD;
long pwm_pulse_period = 0;


double angle_to_go = 0.0;
double velocity_to_go = 0.0;
double effort_to_go = 0.0;
double prev_effort = 0.0;

double kalman_angle;

double prev_kalman_angle = 0;
double curent_kalman_angle = 0;

double target_angle_delta = 0;
double init_angle_target = 0;

double encoder_angle;
double angle_by_ticks = 0.0;
double init_angle = 0.0;
double curent_velocity = 0.0; //TODO Kalman vel


//Kalman's coef
float ticks_c, encoder_c;

//Ticks coef
double ticks_per_round = STEPPER_STEP_DEN * TICKS_PER_CYCLE * GEAR_RATIO;

long lower_angle_limits_in_ticks = 0;
long upper_angle_limits_in_ticks = 0;


double pi_2 = 2 * M_PI;

//Vel defines
double real_velocity_to_go = 0;
long lower_velocity_limits_in_radians = 0;
double upper_velocity_limits_in_radians = 0;
double k_of_linear_part_of_traj_pwm_timer_period = 0;

long linear_part_of_traj_pwm_timer_period = 0;


uint32_t read_fault = 0; //read fault WD


extern uint8_t TxData[8];					// Data to be sent via CAN
extern uint64_t TxMailbox;					// CAN temporary mailbox. Required by HAL function
extern CAN_TxHeaderTypeDef TxHeader;		// Header for can message
extern CAN_FilterTypeDef canfilterconfig;	// Filter for receiving CAN messages
extern uint8_t RxData[8];					// data received from can bus
extern CAN_RxHeaderTypeDef   RxHeader;		// header received by can bus

extern CAN_HandleTypeDef hcan1;
extern I2C_HandleTypeDef hi2c1;

enum State_of_motor { Stop, Go, Idle, Effort };

enum State_of_motor state_of_controller;


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
/* Definitions for Soft_WD_Task */
osThreadId_t Soft_WD_TaskHandle;
const osThreadAttr_t Soft_WD_Task_attributes = {
  .name = "Soft_WD_Task",
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

void motor_controller_cb(const void *);
double get_kalman_angle();
double get_encoder_angle();
long ticks_from_angle(double angle);
double angle_from_ticks(long ticks);
double clamp_value(double value, double min_value, double max_value);


extern void TORQUE_Reg_Set(int torque);



/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void I2CTask01(void *argument);
void MotorController01(void *argument);
void Soft_WD_Task04(void *argument);

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
  I2CTaskHandle = osThreadNew(I2CTask01, NULL, &I2CTask_attributes);

  /* creation of MotorController */
  MotorControllerHandle = osThreadNew(MotorController01, NULL, &MotorController_attributes);

  /* creation of Soft_WD_Task */
  Soft_WD_TaskHandle = osThreadNew(Soft_WD_Task04, NULL, &Soft_WD_Task_attributes);

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
		&motor_controller_cb, ON_NEW_DATA);

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


	/* Infinite loop */

	  for(;;)
	  {

		js_out.position.data[0] = kalman_angle;
		js_out.velocity.data[0] = real_velocity_to_go;
		js_out.effort.data[0] = prev_effort;

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
	//run once at start
	  init_angle = get_encoder_angle();
	  prev_kalman_angle = init_angle;
	  lower_angle_limits_in_ticks = - ticks_from_angle(init_angle + LOWER_ANGLE_LIMIT);
	  upper_angle_limits_in_ticks = ticks_from_angle(UPPER_ANGLE_LIMIT - init_angle);


/* Infinite loop */
	  for(;;)
	  {
		kalman_angle = get_kalman_angle();
		vTaskDelay(1);
	  }
  /* USER CODE END I2CTask01 */
}

/* USER CODE BEGIN Header_MotorController01 */
/**
* @brief Function implementing the MotorController thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorController01 */
void MotorController01(void *argument)
{
  /* USER CODE BEGIN MotorController01 */
	upper_velocity_limits_in_radians = pi_2 / (STEPPER_STEP_DEN * TICKS_PER_CYCLE * GEAR_RATIO / APB1_TIMER_CLOCK_FREQUENCY * MIN_PWM_TIMER_PERIOD);
	k_of_linear_part_of_traj_pwm_timer_period = MIN_PWM_TIMER_PERIOD * upper_velocity_limits_in_radians;

  /* Infinite loop */
  for(;;)
  {
	  switch(state_of_controller)
		{
	  	  case(Stop):
	  			  TIM3->CCR1 = 0;
	  	  	  	  TIM3->ARR = IDLE_PWM_TIMER_PERIOD;
	  	  	  	  real_velocity_to_go = 0;
	  			  break;
	  	  case(Effort):
				  TORQUE_Reg_Set((int)(effort_to_go));
	  	  	  	  state_of_controller = Go;
	  			  break;
	  	  case(Idle):
	  			  break;
	  	  case(Go):
					target_angle_delta = angle_to_go - kalman_angle;
			  	  if (target_angle_delta > 0)
			  	  {
			  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			  	  }
			  	  else
			  	  {
			  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			  	  }
						ticks_to_go = ticks_from_angle(target_angle_delta) + pwm_tick_counter;
			  	  	  	if(fabs(target_angle_delta) < PD_ANGLE_THRESHOLD)
			  	  	  	{
						pwm_timer_period = (long)((MAX_PWM_TIMER_PERIOD - linear_part_of_traj_pwm_timer_period) * fabs(target_angle_delta) + linear_part_of_traj_pwm_timer_period); //TODO Kp add and some minor tests
			  	  	  	pwm_pulse_period = (long)(pwm_timer_period / 2);
			  	  	  	}
			  	  	  	else if(fabs(init_angle_target - kalman_angle) < PD_ANGLE_THRESHOLD)
			  	  	  	{
						pwm_timer_period = (long)(MAX_PWM_TIMER_PERIOD - (MAX_PWM_TIMER_PERIOD - linear_part_of_traj_pwm_timer_period) * fabs(init_angle_target - kalman_angle)*1/PD_ANGLE_THRESHOLD); //TODO Kp add and some minor tests
				  	  	pwm_pulse_period = (long)(pwm_timer_period / 2);
			  	  	  	}
			  	  	  	else
			  	  	  	{
			  	  	  		//TODO velocity
			  	  	  	pwm_timer_period = linear_part_of_traj_pwm_timer_period;
			  	  	  	pwm_pulse_period = (long)(pwm_timer_period / 2);
			  	  	  	}
	  	  	  	state_of_controller = Go;
	  	  	  	break;
		}

    osDelay(10);
  }
  /* USER CODE END MotorController01 */
}

/* USER CODE BEGIN Header_Soft_WD_Task04 */
/**
* @brief Function implementing the Soft_WD_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Soft_WD_Task04 */
void Soft_WD_Task04(void *argument)
{
  /* USER CODE BEGIN Soft_WD_Task04 */
  /* Infinite loop */
  for(;;)
  {
	 if (read_fault > 10)
	 {
		 NVIC_SystemReset();
	 }
	 if (fabs(encoder_angle - kalman_angle) > ENCODER_TO_KALMAN_DEVIATION)
	 {
		 NVIC_SystemReset();
	 }
    vTaskDelay(300);

  }
  /* USER CODE END Soft_WD_Task04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void motor_controller_cb(const void * msgin)
{
// Cast received message to used type
const sensor_msgs__msg__JointState * js_in = (const sensor_msgs__msg__JointState *)msgin;
angle_to_go = js_in->position.data[0];
velocity_to_go = js_in->velocity.data[0];
effort_to_go = js_in->effort.data[0];
if (effort_to_go == 0)
{
	state_of_controller = Idle;
}
else if(velocity_to_go == 0)
{
	state_of_controller = Stop;
}
else if(effort_to_go != prev_effort)
{
	TORQUE_Reg_Set((int)(effort_to_go));
	init_angle_target = kalman_angle;
    real_velocity_to_go = clamp_value(velocity_to_go, lower_velocity_limits_in_radians, upper_velocity_limits_in_radians);
	linear_part_of_traj_pwm_timer_period = (long)(k_of_linear_part_of_traj_pwm_timer_period / real_velocity_to_go);
	state_of_controller = Go;
	prev_effort = effort_to_go;
}
else
{
	init_angle_target = kalman_angle;
    real_velocity_to_go = clamp_value(velocity_to_go, lower_velocity_limits_in_radians, upper_velocity_limits_in_radians);
	linear_part_of_traj_pwm_timer_period = (long)(k_of_linear_part_of_traj_pwm_timer_period / real_velocity_to_go);
	state_of_controller = Go;
}
}

double get_encoder_angle()
{
	//get encoder angle
	buf[0] = 0x0E;
	i2c_ret = HAL_I2C_Master_Transmit(&hi2c1, (0b0110110 << 1) , buf, 1, HAL_MAX_DELAY);
	if(i2c_ret == HAL_OK ){
	}
	i2c_ret = HAL_I2C_Master_Receive(&hi2c1, 0b0110110 << 1, buf_recv, 2, HAL_MAX_DELAY);
	if(i2c_ret == HAL_OK ){
	}
	return (2 * M_PI * (buf_recv[1] + 256 * buf_recv[0]) / 4096);
}

double get_kalman_angle()
{
	//Kalman like filtering
	if (TIM3->CCR1 != 0)
	{
		ticks_c = 0.9;
		encoder_c = 0.1;
	}
	else
	{
		ticks_c = 0.1;
		encoder_c = 0.9;
	}
	encoder_angle = get_encoder_angle();
	curent_kalman_angle = prev_kalman_angle + (angle_from_ticks(pwm_tick_counter) - prev_kalman_angle + init_angle) * ticks_c + (encoder_angle - prev_kalman_angle) * encoder_c;
	prev_kalman_angle = curent_kalman_angle;
	return curent_kalman_angle;
}

long ticks_from_angle(double angle)
{
	return ((angle * ticks_per_round)/ (pi_2));
}

double angle_from_ticks(long ticks)
{
	return (pi_2 * ticks / ticks_per_round);
}

double clamp_value(double value, double min_value, double max_value)
{
	return (((min_value < value)? value : min_value) > max_value)? max_value: value;
}

/* USER CODE END Application */
