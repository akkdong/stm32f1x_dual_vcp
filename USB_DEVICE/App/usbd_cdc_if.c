/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v2.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "main.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */

/* USER CODE BEGIN PRIVATE_VARIABLES */

//
//
//

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

USBD_CDC_LineCodingTypeDef Line_Coding[CDC_NO_OF_INSTANCE] =
{
	{
	  115200, /* baud rate*/
	  0x00,   /* stop bits-1*/
	  0x00,   /* parity - none*/
	  0x08    /* nb. of bits 8*/
	},
	{
	  115200, /* baud rate*/
	  0x00,   /* stop bits-1*/
	  0x00,   /* parity - none*/
	  0x08    /* nb. of bits 8*/
	},
};


/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[CDC_NO_OF_INSTANCE][APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
//uint8_t UserTxBufferFS[CDC_NO_OF_CLASS][APP_TX_DATA_SIZE];

uint8_t uart_rxBuf[CDC_NO_OF_INSTANCE][256];
uint8_t uart_txBuf[CDC_NO_OF_INSTANCE][256];

RingBuffer uart_rb_rx[CDC_NO_OF_INSTANCE];
RingBuffer uart_rb_tx[CDC_NO_OF_INSTANCE];

UartState uartState[CDC_NO_OF_INSTANCE];

//volatile uint8_t transmit_flag[CDC_NO_OF_INSTANCE];

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static uint8_t CDC_Init_FS(uint8_t ch);
static uint8_t CDC_DeInit_FS(uint8_t ch);
static uint8_t CDC_Control_FS(uint8_t ch, uint8_t cmd, uint8_t* pbuf, uint16_t length);
static uint8_t CDC_Receive_FS(uint8_t ch, uint8_t* pbuf, uint32_t *Len);
static uint8_t CDC_TransmitCplt(uint8_t ch, uint8_t *Buf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt,
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static uint8_t CDC_Init_FS(uint8_t ch)
{
  /* USER CODE BEGIN 3 */
  RB_Init(&uart_rb_rx[ch], &uart_rxBuf[ch][0], 256);
  RB_Init(&uart_rb_tx[ch], &uart_txBuf[ch][0], 256);

  UART_Init(&uartState[ch], ch == 0 ? &huart2 : &huart3, &uart_rb_rx[ch], &uart_rb_tx[ch]);
  /*
  UART_Config(&uartState[ch],
		  Line_Coding[ch].bitrate,
		  Line_Coding[ch].format,
		  Line_Coding[ch].paritytype,
		  Line_Coding[ch].datatype);
  */

  /* Set Application Buffers */
  uint8_t *data = UART_PreserveRxBuffer(&uartState[ch], 0, NULL);
  USBD_CDC_SetTxBuffer(ch, &hUsbDeviceFS, data, 0);
  USBD_CDC_SetRxBuffer(ch, &hUsbDeviceFS, UserRxBufferFS[ch]);

  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static uint8_t CDC_DeInit_FS(uint8_t ch)
{
  /* USER CODE BEGIN 4 */

  //
  UART_DeInit(&uartState[ch]);

  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static uint8_t CDC_Control_FS(uint8_t ch, uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:
        Line_Coding[ch].bitrate = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |
                                            (pbuf[2] << 16) | (pbuf[3] << 24));
        Line_Coding[ch].format = pbuf[4];
        Line_Coding[ch].paritytype = pbuf[5];
        Line_Coding[ch].datatype = pbuf[6];

        //
        UART_Config(&uartState[ch],
      		  Line_Coding[ch].bitrate,
      		  Line_Coding[ch].format,
      		  Line_Coding[ch].paritytype,
      		  Line_Coding[ch].datatype);
    break;

    case CDC_GET_LINE_CODING:
        pbuf[0] = (uint8_t)(Line_Coding[ch].bitrate);
        pbuf[1] = (uint8_t)(Line_Coding[ch].bitrate >> 8);
        pbuf[2] = (uint8_t)(Line_Coding[ch].bitrate >> 16);
        pbuf[3] = (uint8_t)(Line_Coding[ch].bitrate >> 24);
        pbuf[4] = Line_Coding[ch].format;
        pbuf[5] = Line_Coding[ch].paritytype;
        pbuf[6] = Line_Coding[ch].datatype;
    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static uint8_t CDC_Receive_FS(uint8_t ch, uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  //
  //
  // CDC.rx ---> push received data into UART transmit buffer
  UART_Transmit(&uartState[ch], Buf, *Len);
  //for (uint32_t i = 0; i < *Len; ++i)
  //  RB_Push(&uart_rb_tx[ch], Buf[i]);
  //
  //
  //CDC_Transmit_FS(ch, Buf, *Len);
  //
  //

  USBD_CDC_SetRxBuffer(ch, &hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(ch, &hUsbDeviceFS);

  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t ch, uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData + ch;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(ch, &hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(ch, &hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
  *
  *
  *
  *
  */

uint8_t CDC_TransmitCplt(uint8_t ch, uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
	//
	UART_CheckoutRxBuffer(&uartState[ch]);

	return USBD_OK;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
