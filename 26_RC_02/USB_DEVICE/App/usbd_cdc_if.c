/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
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
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @brief          : USB CDC Interface file
  *
  * ˵����
  * 1. ���ļ����� STM32CubeMX ���ɵ� CDC ģ���д��
  * 2. ��ǰ�汾����Э�������ֻ�ṩ��ԭʼ�ֽ������շ���
  * 3. ���շ���
  *      USB�ص� -> RX���λ���
  * 4. ���ͷ���
  *      Ӧ��д��TX���λ��� -> CDC_App_TxTask() �ƶ�����
  * 5. ���ڴ��� 64 �ֽڵ����ݣ�FS bulk�˵�������=64����
  *    USB�ײ���Զ���ɶ�������ͣ��㲻��Ҫ�ֶ��ֳ� 64 �ֽڡ�
  *
  * ʹ�÷�������ؼ�����
  * - ����ѭ���ﷴ������ CDC_App_TxTask()
  * - �� CDC_App_Read() ���յ�������
  * - �� CDC_App_Write() �� CDC_Transmit_HS() ��������
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "usbd_cdc.h"
#include "usb_device.h"
#include <stdint.h>
#include <string.h>
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

/* ���Լ���Ӧ�ò㻷�λ����С���� CubeMX ��� 2048 ���岻��һ���£� */
#define CDC_APP_RX_RING_SIZE      4096U
#define CDC_APP_TX_RING_SIZE      4096U

/* �����ύ�� USB ջ����󳤶�
 * ˵����
 * - ���ﲻ�� USB ����
 * - �ײ���Զ������FS �°� 64 �ֽ� bulk ����
 * - ���ֵԽ�󣬵����ύ������Խ�ࣻ512 �� FS CDC һ�㹻��
 */
#define CDC_APP_TX_CHUNK_SIZE      512U

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

/* ���Լ���Ӧ�ò㻷�λ��� */
//static uint8_t s_rxRing[CDC_APP_RX_RING_SIZE];
//static uint8_t s_txRing[CDC_APP_TX_RING_SIZE];
uint8_t s_rxRing[CDC_APP_RX_RING_SIZE];
uint8_t s_txRing[CDC_APP_TX_RING_SIZE];

/* ���λ����д����
 * RX��USB�ص�д�룬Ӧ�ò��ȡ
 * TX��Ӧ�ò�д�룬CDC_App_TxTask() ����
 */
static volatile uint32_t s_rxW = 0U;
static volatile uint32_t s_rxR = 0U;
static volatile uint32_t s_txW = 0U;
static volatile uint32_t s_txR = 0U;

/* ��ǰ�Ѿ��ύ��USBջ������û����������ɵ��ֽ��� */
static volatile uint32_t s_txInflight = 0U;

/* RX �������ͳ�ƣ������ã� */
static volatile uint32_t s_rxDropBytes = 0U;

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

/**
  * @brief  ���㻷�λ��������ֽ���
  */
static uint32_t RB_Used(uint32_t w, uint32_t r, uint32_t size)
{
  return (w >= r) ? (w - r) : (size - r + w);
}

/**
  * @brief  ���㻷�λ���ʣ������ֽ���
  * @note   ��һ����λ�������֡������͡��ա�
  */
static uint32_t RB_Free(uint32_t w, uint32_t r, uint32_t size)
{
  return (size - 1U) - RB_Used(w, r, size);
}

/**
  * @brief  ���λ���ѹ��1���ֽ�
  * @retval 1=�ɹ�, 0=ʧ��(������)
  */
static uint8_t RB_PushByte(volatile uint32_t *w,
                           volatile uint32_t *r,
                           uint8_t *buf,
                           uint32_t size,
                           uint8_t data)
{
  uint32_t next = (*w + 1U) % size;

  if (next == *r)
  {
    return 0U; /* �� */
  }

  buf[*w] = data;
  *w = next;
  return 1U;
}

/**
  * @brief  �ӻ��λ��嵯��1���ֽ�
  * @retval 1=�ɹ�, 0=ʧ��(�����)
  */
static uint8_t RB_PopByte(volatile uint32_t *w,
                          volatile uint32_t *r,
                          uint8_t *buf,
                          uint32_t size,
                          uint8_t *data)
{
  if ((w == NULL) || (r == NULL) || (buf == NULL) || (data == NULL) || (size == 0U))
  {
    return 0U;
  }

  /* �� */
  if (*r == *w)
  {
    return 0U;
  }

  *data = buf[*r];
  *r = (*r + 1U) % size;
  return 1U;
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
  /* �� USB ջ�ķ��ͻ��������� */
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, UserTxBufferHS, 0U);

  /* ���� USB ջ���ջ��� */
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS);

  /* ���Ӧ�ò㻷�λ������� */
  s_rxW = 0U;
  s_rxR = 0U;
  s_txW = 0U;
  s_txR = 0U;
  s_txInflight = 0U;
  s_rxDropBytes = 0U;

  /* ������һ�ν��գ�����Ҫ�� */
  USBD_CDC_ReceivePacket(&hUsbDeviceHS);

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
	while(0){//ע��ԭ����
//  switch(cmd)
//  {
//  case CDC_SEND_ENCAPSULATED_COMMAND:

//    break;

//  case CDC_GET_ENCAPSULATED_RESPONSE:

//    break;

//  case CDC_SET_COMM_FEATURE:

//    break;

//  case CDC_GET_COMM_FEATURE:

//    break;

//  case CDC_CLEAR_COMM_FEATURE:

//    break;

//  /*******************************************************************************/
//  /* Line Coding Structure                                                       */
//  /*-----------------------------------------------------------------------------*/
//  /* Offset | Field       | Size | Value  | Description                          */
//  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
//  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
//  /*                                        0 - 1 Stop bit                       */
//  /*                                        1 - 1.5 Stop bits                    */
//  /*                                        2 - 2 Stop bits                      */
//  /* 5      | bParityType |  1   | Number | Parity                               */
//  /*                                        0 - None                             */
//  /*                                        1 - Odd                              */
//  /*                                        2 - Even                             */
//  /*                                        3 - Mark                             */
//  /*                                        4 - Space                            */
//  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
//  /*******************************************************************************/
//  case CDC_SET_LINE_CODING:

//    break;

//  case CDC_GET_LINE_CODING:

//    break;

//  case CDC_SET_CONTROL_LINE_STATE:

//    break;

//  case CDC_SEND_BREAK:

//    break;

//  default:
//    break;
//  }

//  return (USBD_OK);
 }
  (void)length;

  switch (cmd)
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

    case CDC_SET_LINE_CODING:
      /* pbuf[0..3] = bitrate
         pbuf[4]    = stop bits
         pbuf[5]    = parity
         pbuf[6]    = data bits
         �� USB CDC ��˵���ܶೡ��ֻ�ǡ���ʽ�����������Ȳ�����
       */
      break;

    case CDC_GET_LINE_CODING:
      /* �����λ��Ҫ���ȡ���ڲ���������ɷ���Ĭ��ֵ */
      /* ���� 115200 8N1 */
      pbuf[0] = 0x00;
      pbuf[1] = 0xC2;
      pbuf[2] = 0x01;
      pbuf[3] = 0x00; /* 115200 = 0x0001C200 */
      pbuf[4] = 0x00; /* 1 stop bit */
      pbuf[5] = 0x00; /* no parity */
      pbuf[6] = 0x08; /* 8 data bits */
      break;

    case CDC_SET_CONTROL_LINE_STATE:
      /* DTR / RTS �ȿ����߱仯 */
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
//  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, &Buf[0]);
//  USBD_CDC_ReceivePacket(&hUsbDeviceHS);
//  return (USBD_OK);
	uint32_t i;

  /* �� USB �յ����������ֽ�д�������Լ��� RX ���λ��� */
  for (i = 0U; i < *Len; i++)
  {
    if (RB_PushByte(&s_rxW, &s_rxR, s_rxRing, CDC_APP_RX_RING_SIZE, Buf[i]) == 0U)
    {
      /* RX ���λ������ˣ�ͳ�ƶ����ֽ� */
      s_rxDropBytes++;
    }
  }

  /* ���°ѵ�ǰ���彻���� USB ջ�������̹�����һ�ν��� */
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, Buf);
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
//  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
//  if (hcdc->TxState != 0){
//    return USBD_BUSY;
//  }
//  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, Buf, Len);
//  result = USBD_CDC_TransmitPacket(&hUsbDeviceHS)
  uint32_t i;
  uint32_t free_space;

  if ((Buf == NULL) || (Len == 0U))
  {
    return USBD_OK;
  }

  /* TX ring �ռ䲻����ֱ�ӷ���æ */
  free_space = RB_Free(s_txW, s_txR, CDC_APP_TX_RING_SIZE);
  if ((uint32_t)Len > free_space)
  {
    return USBD_BUSY;
  }

  /* ȫ��д�� TX ring */
  for (i = 0U; i < (uint32_t)Len; i++)
  {
    (void)RB_PushByte(&s_txW, &s_txR, s_txRing, CDC_APP_TX_RING_SIZE, Buf[i]);
  }

  /* �������� */
  CDC_App_TxTask();
	
  return USBD_OK;
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

  /* 推进 TX ring 读指针，清除 inflight，提交下一块 */
  s_txR    = (s_txR + s_txInflight) % CDC_APP_TX_RING_SIZE;
  s_txInflight = 0U;
  CDC_App_TxTask();

  /* USER CODE END 14 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @brief  CDC_App_Available
  *         ��ѯ��ǰ RX ���λ������ж����ֽڿɶ�
  */
/**
  * @brief  CDC_App_Available
  *         ��ѯ��ǰ RX ���λ������ж����ֽڿɶ�
  */
uint32_t CDC_App_Available(void)
{
  return RB_Used(s_rxW, s_rxR, CDC_APP_RX_RING_SIZE);
}

/**
  * @brief  CDC_App_TxFree
  *         ��ѯ��ǰ TX ���λ���ʣ���д�ռ�
  */
uint32_t CDC_App_TxFree(void)
{
  return RB_Free(s_txW, s_txR, CDC_APP_TX_RING_SIZE);
}

/**
  * @brief  CDC_App_GetRxDropped
  *         ��ѯ����ʱ�� RX ���������������ֽ���
  */
uint32_t CDC_App_GetRxDropped(void)
{
  return s_rxDropBytes;
}

/**
  * @brief  CDC_App_Read
  *         �� RX ���λ����ж������ max_len ���ֽ�
  * @param  buf     �������
  * @param  max_len ����ȡ�����ֽ�
  * @retval ʵ�ʶ�ȡ�����ֽ���
  *
  * ˵����
  * - ���ǡ�ԭʼ�ֽ�����ȡ��
  * - ����֤һ�ξ���һ֡
  * - ����������Э�飬���ڸ��ϲ�����֡����
  */
uint32_t CDC_App_Read(uint8_t *buf, uint32_t max_len)
{
  uint32_t count = 0U;
  uint8_t  data;

  if ((buf == NULL) || (max_len == 0U))
  {
    return 0U;
  }

  while (count < max_len)
  {
    if (RB_PopByte(&s_rxW, &s_rxR, s_rxRing, CDC_APP_RX_RING_SIZE, &data) == 0U)
    {
      break; /* û������ */
    }

    buf[count++] = data;
  }

  return count;
}

/**
  * @brief  CDC_App_Write
  *         ��һ������д�� TX ���λ��壬�ȴ���������
  * @param  buf ����ָ��
  * @param  len ���ݳ���
  * @retval 1=�ɹ����, 0=ʧ��(�ռ䲻��)
  *
  * ˵����
  * - �� CDC_Transmit_HS() ��������
  * - ���Ǹ�ֱ�۵�Ӧ�ò�ӿ�
  */
uint8_t CDC_App_Write(const uint8_t *buf, uint32_t len)
{
  uint32_t i;
  uint32_t free_space;

  if ((buf == NULL) || (len == 0U))
  {
    return 1U;
  }

  /* Ҫôȫ��д�룬Ҫôһ���ֽڶ���д */
  free_space = RB_Free(s_txW, s_txR, CDC_APP_TX_RING_SIZE);
  if (len > free_space)
  {
    return 0U;
  }

  for (i = 0U; i < len; i++)
  {
    (void)RB_PushByte(&s_txW, &s_txR, s_txRing, CDC_APP_TX_RING_SIZE, buf[i]);
  }

  /* ������������ */
  CDC_App_TxTask();

  return 1U;
}

/**
  * @brief  CDC_App_TxTask
  *         �ƶ� TX ���λ����е�����ͨ�� USB ʵ�ʷ���
  *
  * ˵����
  * - �����������������Ͳ��� USBD_BUSY ��ס���Ĺؼ�����
  * - ��������ѭ���ﷴ������
  * - ���Ƿ������ģ�ÿ��ֻ����Ҫ��״̬�ƽ�
  */
void CDC_App_TxTask(void)
{
  USBD_CDC_HandleTypeDef *hcdc;
  uint32_t used;
  uint32_t linear_len;

  /* CDC �໹û׼���� */
  if (hUsbDeviceHS.pClassData == NULL)
  {
    return;
  }

  hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;

  /* �Ѿ���һ���ύ��ȥ�ˣ���û����ɻص�����Ҫ�ظ��� */
  if (s_txInflight != 0U)
  {
    return;
  }

  /* USB ���æ */
  if (hcdc->TxState != 0U)
  {
    return;
  }

  /* û�д������� */
  used = RB_Used(s_txW, s_txR, CDC_APP_TX_RING_SIZE);
  if (used == 0U)
  {
    return;
  }

  /* ֻȡһ�����������ڴ棬���绷β */
  if (s_txW > s_txR)
  {
    linear_len = s_txW - s_txR;
  }
  else
  {
    linear_len = CDC_APP_TX_RING_SIZE - s_txR;
  }

  if (linear_len > used)
  {
    linear_len = used;
  }

  /* ���Ƶ����ύ���� */
  if (linear_len > CDC_APP_TX_CHUNK_SIZE)
  {
    linear_len = CDC_APP_TX_CHUNK_SIZE;
  }

  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, &s_txRing[s_txR], (uint16_t)linear_len);

  if (USBD_CDC_TransmitPacket(&hUsbDeviceHS) == USBD_OK)
  {
    /* ��¼����Ѿ��ύ��ȥ���ȴ���ɻص��ƽ��ĳ��� */
    s_txInflight = linear_len;
  }
}


/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
