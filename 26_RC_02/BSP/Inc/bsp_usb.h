
#ifndef __BSP_USB_H__
#define __BSP_USB_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"




/* ??¦Ë?? -> ??¦Ë??????????????? */
#define USB_CMD_ARM_IK_RESULT      0x90U

/* ???????? */
#define USB_FRAME_HEAD1            0xA5U
#define USB_FRAME_HEAD2            0x5AU
#define USB_FRAME_TAIL             0xFFU
#define USB_FRAME_OVERHEAD         7U      /* 2? + 1???? + 1???? + 2CRC + 1¦Â */
#define USB_FRAME_MAX_DATA_LEN     255U
#define USB_FRAME_BUF_SIZE         300U

extern uint8_t usb_Buf[USB_FRAME_BUF_SIZE];


/* ?????? */
void     SendByte(uint8_t data);
uint8_t  Send(const uint8_t *data, uint16_t len);
uint16_t CRC16_Check(const uint8_t *data, uint16_t len);
uint8_t  Send_Cmd_Data(uint8_t cmd, const uint8_t *datas, uint8_t len);

/* ??????? */
uint32_t USB_GetSendDropFrames(void);

/* ??????? */
void Data_Analysis(uint8_t cmd, const uint8_t* datas, uint8_t len);
void Receive(uint8_t bytedata);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_USB_H__ */
