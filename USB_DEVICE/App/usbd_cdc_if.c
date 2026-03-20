/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "crc8_crc16.h"
#include "..\\..\\User\\Communication\\core\\uproto.h" 

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

/* TX 闃熷垪澶у皬锛屽彲鎸夐渶瑕佽皟鏁达紙娉ㄦ剰 RAM 鍗犵敤锛? */
#ifndef USB_TX_RING_SIZE
#define USB_TX_RING_SIZE 2048
#endif

/* �??澶т竴娆�?�彂閫佺殑鍒嗙墖�?垮害锛圡TU锛夛紝鎸夐渶璋冩暣銆傛敞�?? USB FS/HS 闄愬埗锛堜緥�?? 512 �?? 64�?? */
#ifndef USB_TX_CHUNK_MAX
#define USB_TX_CHUNK_MAX 512
#endif

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

/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferHS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

//usb鎺ユ敹鏁扮粍,鍙戦?佺殑鍏ㄥ湪杩欎簡
uint8_t usb_buf[USB_SIZE];
uint16_t usb_buf_len = 0; // 褰撳墠宸叉敹鍒伴暱搴?
static uint8_t  printf_buf[2];   // 1 瀛楄妭鏁版嵁 + 1 鍗犱�?
static volatile uint8_t tx_done = 1; // 鍙戦?佸畬鎴愭爣蹇?

static uint8_t  usb_tx_ring[USB_TX_RING_SIZE];
static volatile uint16_t usb_tx_wpos = 0;   // 涓嬩竴涓啓鍏ヤ綅缃?
static volatile uint16_t usb_tx_count = 0;  // 闃熷垪涓凡鏈夊瓧鑺傛暟

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceHS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

extern uproto_context_t proto_ctx;

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_HS(void);
static int8_t CDC_DeInit_HS(void);
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_HS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_HS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */


/* 鍐呴儴锛氳绠楀綋鍓嶈浣嶇疆锛堢�??涓緟鍙�??佸瓧鑺傜殑浣嶇疆�?? */
static inline uint16_t usb_tx_read_pos(void)
{
    // 璇绘寚閽? = 鍐欐寚閽? - count (mod ring size)
    uint16_t r = (usb_tx_wpos + USB_TX_RING_SIZE - usb_tx_count) % USB_TX_RING_SIZE;
    return r;
}

/* 鍐呴儴锛氬綋 USB 绌洪棽鏃朵粠闃熷垪鍙�??佷笅�??娈碉紙闈為樆濉烇�? */
static void start_usb_tx_if_idle(void)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
    if (hcdc == NULL) return;

    /* 濡傛灉澶栬鏄剧ず蹇欙紝鐩存帴杩斿洖锛堝皢鐢卞畬鎴愬洖璋冪户缁彂閫侊級 */
    if (hcdc->TxState != 0) {
        return;
    }

    /* 鍙栧嚭瑕佸彂閫佺殑闀垮害锛堝敖閲忚繛缁殑涓?娈碉�? */
    __disable_irq();
    uint16_t count = usb_tx_count;
    if (count == 0) {
        __enable_irq();
        return; // nothing to send
    }
    uint16_t read_pos = usb_tx_read_pos();
    uint16_t first_chunk = (uint16_t)((USB_TX_RING_SIZE - read_pos) < count ? (USB_TX_RING_SIZE - read_pos) : count);
    /* 闄愬埗鍗曟鍙戦?佹渶澶ч暱搴︼紙閬垮厤瓒呰�? USB MTU�?? */
    uint16_t send_len = (first_chunk > USB_TX_CHUNK_MAX) ? USB_TX_CHUNK_MAX : first_chunk;
    __enable_irq();

    /* 璁剧疆搴撶殑鍙戦?佺紦鍐插苟鎻愪氦锛堥潪闃诲�?? */
    USBD_CDC_SetTxBuffer(&hUsbDeviceHS, &usb_tx_ring[read_pos], send_len);
    USBD_CDC_TransmitPacket(&hUsbDeviceHS);
}

/* proto_port_write锛氬�? data 鍏ラ槦锛屼笉闃诲�?
   杩斿洖�?�為檯鍏ラ槦鐨勫瓧鑺傛暟锛堝彲鑳藉皬�?? len锛屽綋闃熷垪绌洪棿涓嶈冻鏃讹級銆?
*/
static size_t proto_port_write(void *user, const uint8_t *data, size_t len)
{
    (void)user;
    if (data == NULL || len == 0) return 0;

    __disable_irq();
    uint16_t free_space = (uint16_t)(USB_TX_RING_SIZE - usb_tx_count);
    if (free_space == 0) {
        __enable_irq();
        return 0; // 闃熷垪婊★紝涓㈠�?
    }

    /* 灏介噺鍐欏叆鍏ㄩ儴锛屼絾鑻ョ┖闂翠笉瓒冲彧鍐欏叆鑳藉绾崇殑 */
    uint16_t to_write = (len > free_space) ? free_space : (uint16_t)len;

    uint16_t wpos = usb_tx_wpos;
    uint16_t first = (USB_TX_RING_SIZE - wpos);
    if (first > to_write) first = to_write;

    memcpy(&usb_tx_ring[wpos], data, first);
    if (to_write > first) {
        memcpy(&usb_tx_ring[0], data + first, to_write - first);
        wpos = (uint16_t)(to_write - first);
    } else {
        wpos = (uint16_t)(wpos + first);
        if (wpos >= USB_TX_RING_SIZE) wpos -= USB_TX_RING_SIZE;
    }

    usb_tx_wpos = wpos;
    usb_tx_count = (uint16_t)(usb_tx_count + to_write);
    __enable_irq();

    /* 濡傛�? USB 绌洪棽锛岃Е鍙戝彂閫侊紙浼氬湪瀹屾垚鍥炶皟閲岀户缁級 */
    start_usb_tx_if_idle();

    return (size_t)to_write;
}

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_HS =
{
  CDC_Init_HS,
  CDC_DeInit_HS,
  CDC_Control_HS,
  CDC_Receive_HS,
  CDC_TransmitCplt_HS
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CDC media low layer over the USB HS IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_HS(void)
{
  /* USER CODE BEGIN 8 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, UserTxBufferHS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS);
  return (USBD_OK);
  /* USER CODE END 8 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @param  None
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_HS(void)
{
  /* USER CODE BEGIN 9 */
  return (USBD_OK);
  /* USER CODE END 9 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 10 */
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

    break;

  case CDC_GET_LINE_CODING:

    break;

  case CDC_SET_CONTROL_LINE_STATE:

    break;

  case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 10 */
}

/**
  * @brief Data received over USB OUT endpoint are sent over CDC interface
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
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAILL
  */
static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 11 */
  /* Directly feed raw USB bytes to uproto; do not pre-filter/clear Buf */
  if (Len && *Len) {
    uproto_on_rx_bytes(&proto_ctx, Buf, *Len);
  }
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceHS);
  return (USBD_OK);
  /* USER CODE END 11 */
}

/**
  * @brief  Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 12 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceHS);
  /* USER CODE END 12 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_HS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_HS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 14 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 14 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
int __io_putchar(int ch)
{
	
    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) return ch; // 鏈氨缁洿鎺ヤ�?
    printf_buf[0] = (uint8_t)ch;
    tx_done = 0;
    CDC_Transmit_HS(&printf_buf[0], 1); // 鎻愪氦鍗曞瓧�??
    return ch;
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
