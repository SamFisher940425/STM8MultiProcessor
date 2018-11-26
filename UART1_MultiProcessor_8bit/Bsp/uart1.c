/**
  ******************************************************************************
  * @�ļ�         uart1.c
  * @����         LGG
  * @�汾         V1.0.0
  * @����         2017-11-18
  * @ժҪ         uart1Դ�ļ�
  ******************************************************************************
*/ 

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "uart1.h"

extern unsigned char Light_Address;

/**
  * @������       UART1_Config
  * @��  ��       ����UART1�볬���ն�ͨѶ
  * @��  ��       ��
  * @����ֵ       ��
  */
void UART1_Config(void)
{
  /* EVAL COM (UART) configuration -----------------------------------------*/
  /* USART configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - None parity
        - Receive and transmit enabled
        - UART Clock disabled
  */
  UART1_Init((uint32_t)115200, UART1_WORDLENGTH_8D,UART1_STOPBITS_1, UART1_PARITY_NO,
                   UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);
  
//  UART1_WakeUpConfig(UART1_WAKEUP_ADDRESSMARK);
//  UART1_SetAddress(Light_Address);//change by color
//  UART1_ReceiverWakeUpCmd(ENABLE);

  /* ʹ�ܽ����ж� */
  UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);
  
  /* ʹ�ܷ����ж� */
  //UART1_ITConfig(UART1_IT_TXE, ENABLE);

  /* ʹ�� UART1 */
  UART1_Cmd(ENABLE);
  
  
  
}

void UART1_sendchar(unsigned char ch)
{
      UART1_SendData8(ch);
      while(UART1_GetFlagStatus(UART1_FLAG_TC)!=SET);
}

/************************ Copyright (C)2017 LGG. All Rights Reserved *****END OF FILE****/


