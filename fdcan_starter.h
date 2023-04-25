#include <fdcan.h>

extern FDCAN_TxHeaderTypeDef TxHeader;		// Header for can message
extern FDCAN_FilterTypeDef canfdfilterconfig;	// Filter for receiving CAN messages
extern FDCAN_RxHeaderTypeDef RxHeader;

//FDCAN_HandleTypeDef hfdcan1;



void FDCAN_Filter_Config(FDCAN_FilterTypeDef* sFilterConfig){
	sFilterConfig->IdType = FDCAN_STANDARD_ID;
	sFilterConfig->FilterIndex = 0;
	sFilterConfig->FilterType = FDCAN_FILTER_MASK;
	sFilterConfig->FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig->FilterID1 = 0x07F; //router_id set to 127
	sFilterConfig->FilterID2 = 0x7FF; //;0x1FFFFFFF;
}

void FDCAN_Header_Config(FDCAN_TxHeaderTypeDef* TxHeader){
	TxHeader->Identifier = MY_CAN_ID;

	TxHeader->IdType = FDCAN_STANDARD_ID; // 11-битный ID

	TxHeader->TxFrameType = FDCAN_DATA_FRAME; // Передача обычного кадра данных

	TxHeader->DataLength = FDCAN_DLC_BYTES_8; // Длина данных 8 байт

	TxHeader->ErrorStateIndicator = FDCAN_ESI_ACTIVE; // Индикатор ошибок активен

	TxHeader->BitRateSwitch = FDCAN_BRS_ON; // C переключения частоты передачи

	TxHeader->FDFormat = FDCAN_FD_CAN; // Формат кадра CANFD

	TxHeader->TxEventFifoControl = FDCAN_NO_TX_EVENTS; // Не сохранение параметров события //FDCAN_STORE_TX_EVENTS;// Сохранение параметров события

	TxHeader->MessageMarker = 0x52;// Метка сообщения
}

void FDCAN_Starter(FDCAN_HandleTypeDef* hfdcan, FDCAN_FilterTypeDef* sFilterConfig){
	if (HAL_FDCAN_ConfigFilter(hfdcan, sFilterConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
	  Error_Handler();
	}


	if( HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, 5, 0) != HAL_OK)
	{
		Error_Handler();
	}
	if( HAL_FDCAN_EnableTxDelayCompensation(hfdcan) != HAL_OK)
	{
		Error_Handler();
	}


	if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
	  Error_Handler();
	}
	if( HAL_FDCAN_Start(hfdcan) != HAL_OK )
	{
		Error_Handler();
	}
}
