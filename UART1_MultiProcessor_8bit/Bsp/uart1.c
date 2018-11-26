/**
  ******************************************************************************
  * @文件         uart1.c
  * @作者         LGG
  * @版本         V1.0.0
  * @日期         2017-11-18
  * @摘要         uart1源文件
  ******************************************************************************
*/ 

/* 包含头文件 ----------------------------------------------------------------*/
#include "uart1.h"

extern unsigned char Light_Address;

/**
  * @函数名       UART1_Config
  * @功  能       配置UART1与超级终端通讯
  * @参  数       无
  * @返回值       无
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

  /* 使能接收中断 */
  UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);
  
  /* 使能发送中断 */
  //UART1_ITConfig(UART1_IT_TXE, ENABLE);

  /* 使能 UART1 */
  UART1_Cmd(ENABLE);
  
  
  
}

void UART1_sendchar(unsigned char ch)
{
      UART1_SendData8(ch);
      while(UART1_GetFlagStatus(UART1_FLAG_TC)!=SET);
}

/************************ Copyright (C)2017 LGG. All Rights Reserved *****END OF FILE****/


