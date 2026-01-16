/*
 * UART.h
 *
 *
 *
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"
#include "RingBuffer.h"


//
//
//

typedef struct _UartState
{
	//
	UART_HandleTypeDef *pHandle;
	//
	RingBuffer *txBuf;
	RingBuffer *rxBuf;

	//volatile int32_t trasmitFlag;	// UART.rx --> USB.CDC.tx
	//volatile int32_t txCache;		// USB.CDC.rx --> UART.tx

	volatile uint16_t rxPresrvLen;		// UART.rx --> USB.CDC.tx
	volatile uint16_t txPresrvLen;	// USB.CDC.rx --> UART.tx
	uint8_t rxCache;				// UART.rx cache

	//
	uint8_t usbConfState; // configured or not

} UartState;



//
//
//

void UART_Init(UartState *pState, UART_HandleTypeDef *pHandle, RingBuffer *rx, RingBuffer *tx);
void UART_DeInit(UartState *pState);

void UART_Config(UartState *pState, uint32_t bitrate, uint8_t format, uint8_t parity, uint8_t data);


uint16_t UART_RxAvailable(UartState *pState);
uint16_t UART_TxAvailable(UartState *pState);

void UART_Transmit(UartState *pState, uint8_t *data, uint16_t len);
void UART_ProcessReceive(UartState *pState);

uint8_t *UART_PreserveRxBuffer(UartState *pState, uint16_t maxLen, uint16_t *pLen);
uint8_t *UART_PreserveTxBuffer(UartState *pState, uint16_t maxLen, uint16_t *pLen);

uint16_t UART_CheckoutRxBuffer(UartState *pState);
uint16_t UART_CheckoutTxBuffer(UartState *pState);


#ifdef __cplusplus
}
#endif

#endif /* INC_UART_H_ */
