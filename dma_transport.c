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
#define CAN_IT_BUFFER_SIZE 2048
uint64_t can_head = 0;
uint32_t can_tail = 0;
uint8_t can_buffer[CAN_IT_BUFFER_SIZE];
CAN_HandleTypeDef * can;

HAL_StatusTypeDef ret = HAL_ERROR;

extern uint32_t read_fault; //soft WD
extern IWDG_HandleTypeDef hiwdg; //hard WD

extern uint8_t TxData[8];					// Data to be sent via CAN
extern uint64_t TxMailbox;					// CAN temporary mailbox. Required by HAL function
extern CAN_TxHeaderTypeDef TxHeader;		// Header for can message
extern CAN_FilterTypeDef canfilterconfig;	// Filter for receiving CAN messages
extern CAN_RxHeaderTypeDef RxHeader;

bool cubemx_transport_open(struct uxrCustomTransport * transport){
	can = (CAN_HandleTypeDef*) transport->args;
    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport){
    return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err){
    uint8_t rv = 0;

    rv = (len < 8) ? len : 8;
    TxHeader.DLC = rv;

	ret = HAL_CAN_AddTxMessage(can, &TxHeader, buf, &TxMailbox);
	while(HAL_CAN_IsTxMessagePending(can, TxMailbox));

    return (ret == HAL_OK) ? rv : 0;
   }

size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err)
{
    size_t rcv = 0;
    int ms_used = 0;

    while(ms_used < timeout)
		{
				while ((can_head != can_tail) && (rcv < len))
					{
						buf[rcv] = can_buffer[can_head];
						can_head = (can_head + 1) % CAN_IT_BUFFER_SIZE;
						rcv++;
					}
				if (rcv == 0)
				{
					ms_used++;
					vTaskDelay(1);
				}
				else
				{
					read_fault = 0;
					HAL_IWDG_Refresh(&hiwdg); //hard WD reset
					return rcv;
				}
		}
    *err = 1;
    read_fault++; //soft WD reset
	HAL_IWDG_Refresh(&hiwdg); //hard WD reset
    return -1;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
uint8_t data[8];
ret = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, data);

if (ret != HAL_OK)
{
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1); //TODO ERROR HANDLER
}

if (RxHeader.StdId == ROUTER_ID)
	{
		for(uint16_t i=0; i<RxHeader.DLC; i++)
			{
			if(can_tail == CAN_IT_BUFFER_SIZE)
			{
			    can_tail = 0;
			}
			can_buffer[can_tail] = data[i];
			can_tail++;
			}
	}
}


#endif //RMW_UXRCE_TRANSPORT_CUSTOM

