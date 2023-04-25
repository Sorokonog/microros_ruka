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

static const uint8_t dlc2len[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};


const uint32_t FDCANLengthToDLC[65] = {
        // 0-8
        FDCAN_DLC_BYTES_0,  FDCAN_DLC_BYTES_1,  FDCAN_DLC_BYTES_2,  FDCAN_DLC_BYTES_3,
        FDCAN_DLC_BYTES_4,  FDCAN_DLC_BYTES_5,  FDCAN_DLC_BYTES_6,  FDCAN_DLC_BYTES_7,
        FDCAN_DLC_BYTES_8,
        // 9-12
        FDCAN_DLC_BYTES_12,  FDCAN_DLC_BYTES_12,  FDCAN_DLC_BYTES_12,  FDCAN_DLC_BYTES_12,
        // 13-16
        FDCAN_DLC_BYTES_16, FDCAN_DLC_BYTES_16, FDCAN_DLC_BYTES_16, FDCAN_DLC_BYTES_16,
        // 17-20
        FDCAN_DLC_BYTES_20, FDCAN_DLC_BYTES_20, FDCAN_DLC_BYTES_20, FDCAN_DLC_BYTES_20,
        // 20-24
        FDCAN_DLC_BYTES_24, FDCAN_DLC_BYTES_24, FDCAN_DLC_BYTES_24, FDCAN_DLC_BYTES_24,
        // 24-32
        FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32,
        FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32,
        // 33-48
        FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48,
        FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48,
        FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48,
        FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48,
        // 49-64
        FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64,
        FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64,
        FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64,
        FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64,
};


uint64_t can_head = 0;
uint32_t can_tail = 0;
uint8_t can_buffer[CAN_IT_BUFFER_SIZE];

FDCAN_HandleTypeDef * fdcan;

HAL_StatusTypeDef ret = HAL_ERROR;

uint32_t read_fault; //soft WD
extern IWDG_HandleTypeDef hiwdg; //hard WD

uint8_t TxData[8];					// Data to be sent via CAN
extern FDCAN_TxHeaderTypeDef TxHeader;		// Header for can message
extern FDCAN_FilterTypeDef canfilterconfig;	// Filter for receiving CAN messages
extern FDCAN_RxHeaderTypeDef RxHeader;

uint8_t dlc = 0; //чтобы не пересоздавать все эти переменные каждый раз
uint8_t headerLen = 0;
uint8_t rv = 0;

uint8_t len_to_dlc(size_t * len)
{
    if (* len<= 8)
    {
        dlc = (uint8_t) * len;
    }
    else if (* len < 12)
    {
        dlc = 8;
    }
    else if (* len < 16)
    {
        dlc = 12;
    }
    else if (* len < 20)
    {
        dlc = 16;
    }
    else if (* len < 24)
    {
        dlc = 20;
    }
    else if (* len < 32)
    {
        dlc = 24;
    }
    else if (* len < 48)
    {
        dlc = 32;
    }
    else if (* len < 64)
    {
        dlc = 48;
    }
    else
    {
        dlc = 64;
    }

    return dlc;
}


bool cubemx_transport_open(struct uxrCustomTransport * transport){
	fdcan = (FDCAN_HandleTypeDef*) transport->args;
    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport){
    return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err){

    rv = len_to_dlc(&len); //To drop out part of buffer which will not be transmitted
    TxHeader.DataLength = FDCANLengthToDLC[rv]; //Size of FDCan data length

    if (ret = HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &TxHeader, buf) != HAL_OK)
    		{
    		Error_Handler();
    		}
	while(HAL_FDCAN_IsTxBufferMessagePending(fdcan, fdcan->LatestTxFifoQRequest));
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

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	uint8_t data[64];


	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, data) != HAL_OK)
		{
		Error_Handler();
		}

	for(uint16_t i=0; i<dlc2len[RxHeader.DataLength >> 16]; i++)
		{
		if(can_tail == CAN_IT_BUFFER_SIZE)
		{
			can_tail = 0;
		}
		can_buffer[can_tail] = data[i]; //TODO replace with memcpy
		can_tail++;
		}

    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      Error_Handler();
    }

}

#endif //RMW_UXRCE_TRANSPORT_CUSTOM