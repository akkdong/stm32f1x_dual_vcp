/*
 * UART.c
 *
 *
 *
 */

#include "UART.h"
#include "usbd_cdc_if.h"



//
//
//

static uint8_t *UART_PreserveBuffer(UartState *pState, RingBuffer *rb, uint16_t maxLen, uint16_t *pLen)
{
	int32_t len = 0;
	uint8_t *data = RB_GetConsecutiveData(rb, &len);

	if (!data)
		return 0;

	len = MIN(maxLen, len);

	if (pLen)
		*pLen = len;

	return data;
}


//
//
//

void UART_Init(UartState *pState, UART_HandleTypeDef *pHandle, RingBuffer *rx, RingBuffer *tx)
{
	pState->pHandle = pHandle;
	pState->txBuf = tx;
	pState->rxBuf = rx;

	pState->rxPresrvLen = 0;
	pState->txPresrvLen = 0;
	pState->rxCache = -1;
	pState->usbConfState = 0;
}

void UART_DeInit(UartState *pState)
{
	if (HAL_UART_DeInit(pState->pHandle) != HAL_OK)
		Error_Handler();
}


uint16_t UART_RxAvailable(UartState *pState)
{
	return pState->rxPresrvLen == 0 && RB_GetDataCount(pState->rxBuf);
}

uint16_t UART_TxAvailable(UartState *pState)
{
	return pState->txPresrvLen == 0 && RB_GetDataCount(pState->txBuf);
}


void UART_Transmit(UartState *pState, uint8_t *data, uint16_t len)
{
	for (uint16_t i = 0; i < len; ++i)
		RB_Push(pState->txBuf, data[i]);
}

void UART_ProcessReceive(UartState *pState)
{
	RB_Push(pState->rxBuf, pState->rxCache);
	HAL_UART_Receive_IT(pState->pHandle, &pState->rxCache, 1);
}


uint8_t *UART_PreserveRxBuffer(UartState *pState, uint16_t maxLen, uint16_t *pLen)
{
	if (pState->rxPresrvLen > 0)
		return 0;

	uint16_t len = 0;
	uint8_t *data = UART_PreserveBuffer(pState, pState->rxBuf, maxLen, &len);
	if (data)
		pState->rxPresrvLen = len;
	if (pLen)
		*pLen = len;

	return data;
}

uint8_t *UART_PreserveTxBuffer(UartState *pState, uint16_t maxLen, uint16_t *pLen)
{
	if (pState->txPresrvLen > 0)
		return 0;

	uint16_t len = 0;
	uint8_t *data = UART_PreserveBuffer(pState, pState->txBuf, maxLen, &len);
	if (data)
		pState->txPresrvLen = len;
	if (pLen)
		*pLen = len;

	return data;
}

uint16_t UART_CheckoutRxBuffer(UartState *pState)
{
	uint16_t len = 0;

	if (pState->rxPresrvLen > 0)
	{
		len = RB_Flush(pState->rxBuf, pState->rxPresrvLen);
		pState->rxPresrvLen = 0;
	}

	return len;
}

uint16_t UART_CheckoutTxBuffer(UartState *pState)
{
	uint16_t len = 0;

	if (pState->txPresrvLen > 0)
	{
		len = RB_Flush(pState->txBuf, pState->txPresrvLen);
		pState->txPresrvLen = 0;
	}

	return len;
}


void UART_Config(UartState *pState, uint32_t bitrate, uint8_t format, uint8_t parity, uint8_t data)
{
	//
	if(HAL_UART_DeInit(pState->pHandle) != HAL_OK)
		Error_Handler();

	// Set the Stop bit
	switch (format)
	{
	case 0:
		pState->pHandle->Init.StopBits = UART_STOPBITS_1;
		break;
	case 2:
		pState->pHandle->Init.StopBits = UART_STOPBITS_2;
		break;
	default :
		pState->pHandle->Init.StopBits = UART_STOPBITS_1;
		break;
	}

	// Set the parity bit
	switch (parity)
	{
	case 0:
		pState->pHandle->Init.Parity = UART_PARITY_NONE;
		break;
	case 1:
		pState->pHandle->Init.Parity = UART_PARITY_ODD;
		break;
	case 2:
		pState->pHandle->Init.Parity = UART_PARITY_EVEN;
		break;
	default :
		pState->pHandle->Init.Parity = UART_PARITY_NONE;
		break;
	}

	// Set the data type : only 8bits and 9bits is supported
	switch (data)
	{
	case 0x07:
		// With this configuration a parity (Even or Odd) must be set
		pState->pHandle->Init.WordLength = UART_WORDLENGTH_8B;
		break;
	case 0x08:
		if(pState->pHandle->Init.Parity == UART_PARITY_NONE)
			pState->pHandle->Init.WordLength = UART_WORDLENGTH_8B;
		else
			pState->pHandle->Init.WordLength = UART_WORDLENGTH_9B;
		break;
	default :
		pState->pHandle->Init.WordLength = UART_WORDLENGTH_8B;
		break;
	}

	pState->pHandle->Init.BaudRate = bitrate;
	pState->pHandle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	pState->pHandle->Init.Mode = UART_MODE_TX_RX;
	pState->pHandle->Init.OverSampling = UART_OVERSAMPLING_16;

	//
	if(HAL_UART_Init(pState->pHandle) != HAL_OK)
		Error_Handler();

	// Start reception: provide the buffer pointer with offset and the buffer size
	HAL_UART_Receive_IT(pState->pHandle, &pState->rxCache, 1);
}
