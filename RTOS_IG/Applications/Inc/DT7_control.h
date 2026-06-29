#pragma once
#ifndef DT7_CONTROL_H   
#define DT7_CONTROL_H   

#include "include.h"

#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)

extern uint8_t dbus_buf[DBUS_BUFLEN];

typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t roll;
	uint8_t sw1;
	uint8_t sw2;
} rc_info_t;
extern rc_info_t rc;
#define rc_Init   \
{                 \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
}


void dbus_uart_init(void);
void uart_receive_handler(UART_HandleTypeDef* huart);

#endif 
