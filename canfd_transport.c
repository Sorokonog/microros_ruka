#include <microros_transports.h>

#include "main.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include <uxr/client/transport.h>
#include <uxr/client/util/time.h>
#include <rmw_microxrcedds_c/config.h>




// --- micro-ROS Transports ---
#define CAN_DMA_BUFFER_SIZE 256

extern uint8_t TxData[8];					// Data to be sent via CAN
extern uint64_t TxMailbox;					// CAN temporary mailbox. Required by HAL function
extern CAN_TxHeaderTypeDef TxHeader;		// Header for can message
extern CAN_FilterTypeDef canfilterconfig;	// Filter for receiving CAN messages
HAL_StatusTypeDef ret;

static uint8_t can_buffer[CAN_DMA_BUFFER_SIZE];
static size_t can_head = 0, can_tail = 0;
static uint8_t free_level=0;

bool canfd_transport_open(struct uxrCustomTransport * transport){
	//TODO set all init functions for CAN here
    return true;
}

bool canfd_transport_close(struct uxrCustomTransport * transport){
    CAN_HandleTypeDef * can = (CAN_HandleTypeDef*) transport->args;
    HAL_CAN_Stop(can);
    return true;
}

size_t canfd_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err)
{
CAN_HandleTypeDef * can = (CAN_HandleTypeDef*) transport->args;

HAL_StatusTypeDef ret = HAL_ERROR;
uint32_t pTxMailbox;

ret = HAL_CAN_AddTxMessage( can, &TxHeader, buf , &pTxMailbox );
while(HAL_CAN_IsTxMessagePending(can,pTxMailbox));

return (ret == HAL_OK) ? len : 0;
}


size_t canfd_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1);

    /*

	CAN_HandleTypeDef *can = (CAN_HandleTypeDef*) transport->args;

    size_t wrote = 0;

    while ((can_head != can_tail) && (wrote < len)){
        buf[wrote] = can_buffer[can_head];
        can_head = (can_head + 1) % CAN_DMA_BUFFER_SIZE;
        wrote++;
    }
    osDelay(timeout);
    */
    return 0;
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){

	CAN_RxHeaderTypeDef RxHeader;
	uint8_t data[8];

	ret = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, data);

	if (ret == HAL_OK)
	{
	for(int i=0;i<RxHeader.DLC;i++){
	can_buffer[can_tail]=data[i];
	can_tail ++;
	}
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
	if(free_level>0) free_level --;
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
	if(free_level>0) free_level --;
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
	if(free_level>0) free_level --;
}


//#endif //RMW_UXRCE_TRANSPORT_CUSTOM
