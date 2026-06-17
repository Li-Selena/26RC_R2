#include "PC_RX_Task.h"
#include "cmsis_os.h"

extern uint8_t usb_Buf[USB_FRAME_BUF_SIZE];// USB���ջ�����
extern uint8_t bt_data[BT_FRAME_DATA_LEN];     // ��������


static void USB_RX_task(void);


void PC_RX_Task(void const * argument)
{
  /* USER CODE BEGIN PC_RX_Task */
  /* Infinite loop */
  for(;;)
  {
    
    USB_RX_task();



    osDelay(1);
  }
  /* USER CODE END PC_RX_Task */
}


static void USB_RX_task(void)
{
    uint32_t i;
    uint32_t read_len;

    read_len = CDC_App_Read(usb_Buf, sizeof(usb_Buf));
    for (i = 0; i < read_len; i++)
    {
        Receive(usb_Buf[i]);
    }
}
