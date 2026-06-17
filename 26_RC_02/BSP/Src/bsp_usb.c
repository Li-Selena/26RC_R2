#include "bsp_usb.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include "arm_user.h"
#include "Control_Task.h"

extern volatile uint8_t USB_Task_flag;
extern volatile uint8_t USART_Task_flag ;

void ArmIK_ComponentStep(float x, float y, float z);


/* Э����ջ���? */
uint8_t usb_Buf[USB_FRAME_BUF_SIZE];

/* ����ʧ��ͳ�ƣ�TX ring �ռ䲻������ */
static volatile uint32_t s_usbSendDropFrames = 0U;

/* ����״̬�������� */
static uint8_t  s_usbRxStep    = 0U;
static uint16_t s_usbRxCnt     = 0U;
static uint8_t  s_usbRxLen     = 0U;
static uint8_t  s_usbRxCmd     = 0U;
static uint8_t *s_usbRxDataPtr = 0;
static uint16_t s_usbRxCrc16   = 0U;


/*========================= �ڲ��������� =========================*/

/* ��������ȫ��λ */
static void USB_ParserReset(void)
{
    s_usbRxStep    = 0U;
    s_usbRxCnt     = 0U;
    s_usbRxLen     = 0U;
    s_usbRxCmd     = 0U;
    s_usbRxDataPtr = 0;
    s_usbRxCrc16   = 0U;
}

/* �ѵ�ǰ�ֽڵ����µ�֡ͷ1���¿�ʼ */
static void USB_ParserRestartFromHead1(void)
{
    USB_ParserReset();
    s_usbRxStep = 1U;
    usb_Buf[s_usbRxCnt++] = USB_FRAME_HEAD1;
}

/* ��Э�黺����ѹһ���ֽڣ�ʧ����λ */
static uint8_t USB_ParserPushByte(uint8_t byte)
{
    if (s_usbRxCnt >= USB_FRAME_BUF_SIZE)
    {
        USB_ParserReset();
        return 0U;
    }

    usb_Buf[s_usbRxCnt++] = byte;
    return 1U;
}



/*========================= ���Ͳ��� =========================*/

void SendByte(uint8_t data)
{
    (void)data;
    /* ���ﱣ����ʵ�֡�
       �㵱ǰ USB CDC �����Ѿ��� Send()/Send_Cmd_Data() �ˣ�
       ������Ҫ���ֽڵ������������? */
}

uint8_t Send(const uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len == 0U))
    {
        return 1U;
    }

    /* TX ring �ռ䲻�㣬��¼һ�η��Ͷ��� */
    if (CDC_App_TxFree() < len)
    {
        s_usbSendDropFrames++;
        return 0U;
    }

    if (CDC_App_Write(data, len) == 0U)
    {
        s_usbSendDropFrames++;
        return 0U;
    }

    return 1U;
}

uint32_t USB_GetSendDropFrames(void)
{
    return s_usbSendDropFrames;
}

/* CRC16(Modbus ����ʽ 0xA001) */
uint16_t CRC16_Check(const uint8_t *data, uint16_t len)
{
    uint16_t crc16 = 0xFFFFU;
    uint16_t i;
    uint8_t  j;

    if ((data == 0) || (len == 0U))
    {
        return crc16;
    }

    for (i = 0U; i < len; i++)
    {
        crc16 ^= data[i];

        for (j = 0U; j < 8U; j++)
        {
            if ((crc16 & 0x0001U) != 0U)
            {
                crc16 >>= 1U;
                crc16 ^= 0xA001U;
            }
            else
            {
                crc16 >>= 1U;
            }
        }
    }

    return crc16;
}

/* ����Э��֡
 * ע�⣺
 * 1. ������ԭ����Э�鲻��
 * 2. CRC ��Ȼ�������ֽ���ǰ�����ֽ��ں󡱷���
 */
uint8_t Send_Cmd_Data(uint8_t cmd, const uint8_t *datas, uint8_t len)
{
    uint8_t  buf[USB_FRAME_BUF_SIZE];
    uint16_t cnt = 0U;
    uint16_t i;
    uint16_t crc16;

    if ((datas == 0) && (len != 0U))
    {
        return 0U;
    }

    /* ��֡�� = ���ݳ��� + 7 */
    if (((uint16_t)len + USB_FRAME_OVERHEAD) > USB_FRAME_BUF_SIZE)
    {
        s_usbSendDropFrames++;
        return 0U;
    }

    buf[cnt++] = USB_FRAME_HEAD1;
    buf[cnt++] = USB_FRAME_HEAD2;
    buf[cnt++] = len;
    buf[cnt++] = cmd;

    for (i = 0U; i < (uint16_t)len; i++)
    {
        buf[cnt++] = datas[i];
    }

    /* �� [֡ͷ, ����, ����, ����] �� CRC */
    crc16 = CRC16_Check(buf, cnt);

    /* �����㵱ǰЭ���ֽ����ȸ��ֽڣ�����ֽ�? */
    buf[cnt++] = (uint8_t)(crc16 >> 8);
    buf[cnt++] = (uint8_t)(crc16 & 0xFFU);
    buf[cnt++] = USB_FRAME_TAIL;

    return Send(buf, cnt);
}


/*========================= Э�����ݽ��� =========================*/



/*========================= ����״̬�� =========================*/

/* ���ֽ�ι�������? */
void Receive(uint8_t bytedata)
{
    uint16_t calc_crc;

    switch (s_usbRxStep)
    {
    case 0: /* �ȴ�֡ͷ1 */
        if (bytedata == USB_FRAME_HEAD1)
        {
            USB_ParserRestartFromHead1();
        }
        break;

    case 1: /* �ȴ�֡ͷ2 */
        if (bytedata == USB_FRAME_HEAD2)
        {
            if (USB_ParserPushByte(bytedata) == 0U)
            {
                return;
            }
            s_usbRxStep = 2U;
        }
        else if (bytedata == USB_FRAME_HEAD1)
        {
            /* ���� A5���ѵ�ǰ�ֽڵ����µ�֡ͷ1 */
            USB_ParserRestartFromHead1();
        }
        else
        {
            USB_ParserReset();
        }
        break;

    case 2: /* ���ճ��� */
        s_usbRxLen = bytedata;

        /* ��ֹЭ�黺�����? */
        if (((uint16_t)s_usbRxLen + USB_FRAME_OVERHEAD) > USB_FRAME_BUF_SIZE)
        {
            USB_ParserReset();
            break;
        }

        if (USB_ParserPushByte(bytedata) == 0U)
        {
            return;
        }

        s_usbRxStep = 3U;
        break;

    case 3: /* �������� */
        if (USB_ParserPushByte(bytedata) == 0U)
        {
            return;
        }

        s_usbRxCmd = bytedata;
        s_usbRxDataPtr = &usb_Buf[s_usbRxCnt];

        if (s_usbRxLen == 0U)
        {
            s_usbRxStep = 5U; /* �����ݣ�ֱ��ȥ�� CRC ���ֽ� */
        }
        else
        {
            s_usbRxStep = 4U;
        }
        break;

    case 4: /* ���������� */
        if (USB_ParserPushByte(bytedata) == 0U)
        {
            return;
        }

        /* ��ǰ�Ѿ����˶��ٸ������ֽ� = �ܼ��� - ͷ2 - ����1 - ����1 */
        if ((s_usbRxCnt - 4U) >= (uint16_t)s_usbRxLen)
        {
            s_usbRxStep = 5U;
        }
        break;

    case 5: /* ���� CRC ���ֽ� */
        s_usbRxCrc16 = ((uint16_t)bytedata) << 8;
        s_usbRxStep = 6U;
        break;

    case 6: /* ���� CRC ���ֽ� */
        s_usbRxCrc16 |= bytedata;

        calc_crc = CRC16_Check(usb_Buf, s_usbRxCnt);

        if (s_usbRxCrc16 == calc_crc)
        {
            s_usbRxStep = 7U;
        }
        else if (bytedata == USB_FRAME_HEAD1)
        {
            /* CRC ʧ�ܣ�����ǰ�ֽ������� A5������µ�֡�?1���� */
            USB_ParserRestartFromHead1();
        }
        else
        {
            USB_ParserReset();
        }
        break;

    case 7: /* ����֡β */
        if (bytedata == USB_FRAME_TAIL)
        {
            Data_Analysis(s_usbRxCmd, s_usbRxDataPtr, s_usbRxLen);
            USB_ParserReset();
        }
        else if (bytedata == USB_FRAME_HEAD1)
        {
            /* ֡β����������ֽڿ�������һ֡���? */
            USB_ParserRestartFromHead1();
        }
        else
        {
            USB_ParserReset();
        }
        break;

    default:
        USB_ParserReset();
        break;
    }
}




