#include "bsp_uart.h"
#include "imu.h"
#include "CRC.h"
#include "arm_echo_uart10.h"
#include "usart.h"

extern uint8_t imu_rx_byte;
extern uint8_t btReceiveData;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7)
    {
        /* 1. �����յ����ֽڴ���Wit SDK���� */
        WitSerialDataIn(imu_rx_byte);

        /* 2. ���¿��������жϣ����ģ���֤�������գ� */
        if (HAL_UART_Receive_IT(&huart7, &imu_rx_byte, 1) != HAL_OK)
        {
            Error_Handler();
        }
    }
		if (huart->Instance == USART10)
    {
        UART10_Receive(btReceiveData);

        if (HAL_UART_Receive_IT(&huart10, &btReceiveData, 1) != HAL_OK)
        {
            Error_Handler();
        }
    }
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7)
    {
        /* �����ǿ����װ���ж� */
        HAL_UART_Receive_IT(&huart7, &imu_rx_byte, 1);
    }
		if (huart->Instance == USART10)
    {
        /* �����ǿ����װ���ж� */
        HAL_UART_Receive_IT(&huart10, &btReceiveData, 1);
        ArmEchoUart10_ErrorHandler();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART10)
    {
        ArmEchoUart10_TxCpltHandler();
    }
}

