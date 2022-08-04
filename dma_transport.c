#include <uxr/client/transport.h>

#include <rmw_microxrcedds_c/config.h>

#include "main.h"
#include "cmsis_os.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

// --- micro-ROS Transports ---
#define CAN_DMA_BUFFER_SIZE 2048
uint32_t can_head = 0;
uint32_t can_tail = 0;
uint8_t can_buffer[CAN_DMA_BUFFER_SIZE];

extern CAN_TxHeaderTypeDef TxHeader;
extern uint32_t TxMailbox;


bool cubemx_transport_open(struct uxrCustomTransport * transport){
    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport){
    return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err){
	CAN_HandleTypeDef * can = (CAN_HandleTypeDef*) transport->args;
    HAL_StatusTypeDef ret = HAL_ERROR;
    uint8_t rv = 0;

	ret = HAL_CAN_AddTxMessage(can, &TxHeader, buf, &TxMailbox);
	while(HAL_CAN_IsTxMessagePending(can, TxMailbox));

	rv =  (len < 8) ? len : 8;
    return (ret == HAL_OK) ? rv : 0;
   }

size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    size_t bytes_recv = 0;

    while ((can_head != can_tail) && (bytes_recv < len)){
        buf[bytes_recv] = can_buffer[can_head];
        can_head = (can_head + 1) % CAN_DMA_BUFFER_SIZE;
        bytes_recv++;
    }
    return bytes_recv;
	}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){

CAN_RxHeaderTypeDef RxHeader;
uint8_t data[8];

HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, data);
	if (RxHeader.StdId == ROUTER_ID)
	{
		for(uint16_t i=0; i<RxHeader.DLC; i++)
			{
			can_buffer[can_tail] = data[i];
			can_tail++;
			}
	}
}


#endif //RMW_UXRCE_TRANSPORT_CUSTOM
