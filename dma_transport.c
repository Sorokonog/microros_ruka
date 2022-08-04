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
#define UART_DMA_BUFFER_SIZE 2048

extern uint8_t TxData[8];					// Data to be sent via CAN
extern uint64_t TxMailbox;					// CAN temporary mailbox. Required by HAL function
extern CAN_TxHeaderTypeDef TxHeader;

static uint8_t dma_buffer[UART_DMA_BUFFER_SIZE];
static size_t dma_head = 0, dma_tail = 0;
static bool init = true;

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
    uint32_t pTxMailbox;
    uint8_t TxData[8];


    TxData[0] = len;
    TxData[1] = 0;
    TxData[2] = 0;
    TxData[3] = 0;
    TxData[4] = 0;

	ret = HAL_CAN_AddTxMessage(can, &TxHeader, buf , &pTxMailbox);
	while(HAL_CAN_IsTxMessagePending(can, pTxMailbox));

	rv =  (len < 8) ? len : 8;
	//if (buf[0] = 126 && init)
	//{
	//	rv = 5;
	//	init = false;
	//}
    return (ret == HAL_OK) ? rv : 0;
   }

size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){

    return 0;
}

#endif //RMW_UXRCE_TRANSPORT_CUSTOM
