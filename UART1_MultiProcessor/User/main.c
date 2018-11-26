/**
  ******************************************************************************
  * @�ļ�         main.c
  * @����         LGG
  * @�汾         V1.0.0
  * @����         2017-11-18
  * @ժҪ         UART1����
  ******************************************************************************
  * @attention
  * ������         www.jixin.pro
  * Ӳ��ƽ̨       ����STM8S001J3���İ�
  * ����Ч��       UART1����"HyperTerminal Interrupt: UART1-Hyperterminal communication using Interrupt"
  *                ͨ����λ�����͹�������Ϣ�ᷢ�ͻ�ȥ
  ******************************************************************************
  */ 

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm8s.h"
#include "uart1.h"


/* �궨�� --------------------------------------------------------------------*/
/*�궨�尴���� PORT �� IO */
#define KEY_GPIO_PORT  (GPIOB)
#define KEY_GPIO_PINS  (GPIO_PIN_4)

/* �������� ------------------------------------------------------------------*/
void Delay(uint32_t nCount);
void GPIO_Config(void);

static void IWDG_Config(void);
static uint32_t LSIMeasurment(void);

__IO uint32_t LsiFreq = 0;

unsigned char Light_Color='G';//change by color
unsigned char Light_Address=0x2;

unsigned char Mode_Cnt=0;
unsigned char Status_Cnt=0;

extern uint16_t TxBuffer[4];
extern __IO uint8_t TxCounter;

/**
  * @������       main
  * @��  ��       ���������
  * @��  ��       ��
  * @����ֵ       ��
  */
void main(void)
{
 /*******************��ʼ����������������*************************/
  GPIO_Init(KEY_GPIO_PORT, (GPIO_Pin_TypeDef)KEY_GPIO_PINS, GPIO_MODE_IN_PU_NO_IT);
  
  if(!GPIO_ReadInputPin(KEY_GPIO_PORT,KEY_GPIO_PINS)) 
  {
    while(1);
  }
//  GPIO_DeInit(KEY_GPIO_PORT);
 /****************************************************************/
  
  CLK_DeInit();

  /* Configure the Fcpu to DIV1*/
  CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
    
  /* Configure the HSI prescaler to the optimal value */
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);             //����ϵͳʱ��HSI = 16M/1
  
  CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSI, DISABLE, CLK_CURRENTCLOCKSTATE_DISABLE);
  
  Delay(0xFFFF);
  
  GPIO_Config();
  
  UART1_Config();       //UART1��ʼ��
  
  /* Get measured LSI frequency */
  LsiFreq = LSIMeasurment();  
  
  /* IWDG Configuration */
  IWDG_Config();
  
  enableInterrupts();   //ʹ�����ж� 
  
  while (1)
  {
    /* Reload IWDG counter */
    IWDG_ReloadCounter();
    
    if(Status_Cnt>=2)
    {
      switch(Mode_Cnt)
      {
      case '0':
        GPIO_WriteLow(GPIOA,GPIO_PIN_3);
        GPIO_WriteLow(GPIOC,GPIO_PIN_5);
        TxBuffer[0]=Light_Color;
        TxBuffer[1]='0';
        TxBuffer[2]='D';
        TxBuffer[3]='\n';
        UART1_ITConfig(UART1_IT_TXE, ENABLE);
        break;
      case '1':
        GPIO_WriteHigh(GPIOA,GPIO_PIN_3);
        GPIO_WriteLow(GPIOC,GPIO_PIN_5);
        TxBuffer[0]=Light_Color;
        TxBuffer[1]='1';
        TxBuffer[2]='D';
        TxBuffer[3]='\n';
        UART1_ITConfig(UART1_IT_TXE, ENABLE);
        break;
      case '2':
        GPIO_WriteLow(GPIOA,GPIO_PIN_3);
        GPIO_WriteHigh(GPIOC,GPIO_PIN_5);
        TxBuffer[0]=Light_Color;
        TxBuffer[1]='2';
        TxBuffer[2]='D';
        TxBuffer[3]='\n';
        UART1_ITConfig(UART1_IT_TXE, ENABLE);
        break;
      case '3':
        GPIO_WriteHigh(GPIOA,GPIO_PIN_3);
        GPIO_WriteHigh(GPIOC,GPIO_PIN_5);
        TxBuffer[0]=Light_Color;
        TxBuffer[1]='3';
        TxBuffer[2]='D';
        TxBuffer[3]='\n';
        UART1_ITConfig(UART1_IT_TXE, ENABLE);
        break;
      default :
        TxBuffer[0]=Light_Color;
        TxBuffer[1]='F';
        TxBuffer[2]='\n';
        TxBuffer[3]='\n';
        UART1_ITConfig(UART1_IT_TXE, ENABLE);
        break;
      }
      while(UART1_GetITStatus(UART1_IT_TXE)==SET);
      Mode_Cnt=0x00;
      Status_Cnt=0;
    }
  }

}


/**
  * @������       Delay
  * @��  ��       ����ӳ�
  * @��  ��       nCount
  * @����ֵ       ��
  */
void Delay(uint32_t nCount)
{
  /* ���� nCount ֵ */
  while (nCount != 0)
  {
    nCount--;
  }
}

/**
  * @������       GPIO_Config
  * @��  ��       ����ͨ����������ӿ�
  * @��  ��       ��
  * @����ֵ       ��
  */
void GPIO_Config(void)
{
  /* �ֲ�ע�����ἰ �����EMC���� ���͹��� */
  GPIO_Init(GPIOA, GPIO_PIN_2, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOB, GPIO_PIN_0, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOB, GPIO_PIN_1, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOB, GPIO_PIN_2, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOB, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOB, GPIO_PIN_6, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOB, GPIO_PIN_7, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOC, GPIO_PIN_1, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOC, GPIO_PIN_2, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOC, GPIO_PIN_7, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOD, GPIO_PIN_0, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOD, GPIO_PIN_2, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOD, GPIO_PIN_7, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOE, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOF, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_SLOW);
  /* �ֲ�ע�����ἰ �����EMC���� ���͹��� */
  
  GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);
  
  GPIO_Init(GPIOD, GPIO_PIN_1, GPIO_MODE_IN_FL_IT);
}

/**
  * @brief  Configures the IWDG to generate a Reset if it is not refreshed at the
  *         correct time. 
  * @param  None
  * @retval None
  */
static void IWDG_Config(void)
{
  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();
  
  /* IWDG timeout equal to 250 ms (the timeout may varies due to LSI frequency
     dispersion) */
  /* Enable write access to IWDG_PR and IWDG_RLR registers */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  
  /* IWDG counter clock: LSI/128 */
  IWDG_SetPrescaler(IWDG_Prescaler_128);
  
  /* Set counter reload value to obtain 250ms IWDG Timeout.
    Counter Reload Value = 250ms/IWDG counter clock period
                         = 250ms / (LSI/128)
                         = 0.25s / (LsiFreq/128)
                         = LsiFreq/(128 * 4)
                         = LsiFreq/512
   */
  IWDG_SetReload((uint8_t)(LsiFreq/512));
  
  /* Reload IWDG counter */
  IWDG_ReloadCounter();
}

/**
  * @brief  Measure the LSI frequency using timer IC1 and update the calibration registers.
  * @note   It is recommended to use a timer clock frequency of at least 10MHz in order 
  *         to obtain a better in the LSI frequency measurement.
  * @param  None
  * @retval None
  */
static uint32_t LSIMeasurment(void)
{
  uint32_t lsi_freq_hz = 0x0;
  uint32_t fmaster = 0x0;
  uint16_t ICValue1 = 0x0;
  uint16_t ICValue2 = 0x0;

  /* Get master frequency */
  fmaster = CLK_GetClockFreq();

  /* Enable the LSI measurement: LSI clock connected to timer Input Capture 1 */
  AWU->CSR |= AWU_CSR_MSR;

#if defined (STM8S903) || defined (STM8S103) || defined (STM8S003) || defined (STM8S001)
  /* Measure the LSI frequency with TIMER Input Capture 1 */
  
  /* Capture only every 8 events!!! */
  /* Enable capture of TI1 */
  TIM1_ICInit(TIM1_CHANNEL_1, TIM1_ICPOLARITY_RISING, TIM1_ICSELECTION_DIRECTTI,
              TIM1_ICPSC_DIV8, 0);
  
  /* Enable TIM1 */
  TIM1_Cmd(ENABLE);
  
  /* wait a capture on cc1 */
  while((TIM1->SR1 & TIM1_FLAG_CC1) != TIM1_FLAG_CC1);
  /* Get CCR1 value*/
  ICValue1 = TIM1_GetCapture1();
  TIM1_ClearFlag(TIM1_FLAG_CC1);
  
  /* wait a capture on cc1 */
  while((TIM1->SR1 & TIM1_FLAG_CC1) != TIM1_FLAG_CC1);
  /* Get CCR1 value*/
  ICValue2 = TIM1_GetCapture1();
  TIM1_ClearFlag(TIM1_FLAG_CC1);
  
  /* Disable IC1 input capture */
  TIM1->CCER1 &= (uint8_t)(~TIM1_CCER1_CC1E);
  /* Disable timer2 */
  TIM1_Cmd(DISABLE);
  
#else  
  /* Measure the LSI frequency with TIMER Input Capture 1 */
  
  /* Capture only every 8 events!!! */
  /* Enable capture of TI1 */
  TIM3_ICInit(TIM3_CHANNEL_1, TIM3_ICPOLARITY_RISING, TIM3_ICSELECTION_DIRECTTI,
              TIM3_ICPSC_DIV8, 0);

  /* Enable TIM3 */
  TIM3_Cmd(ENABLE);

  /* wait a capture on cc1 */
  while ((TIM3->SR1 & TIM3_FLAG_CC1) != TIM3_FLAG_CC1);
  /* Get CCR1 value*/
  ICValue1 = TIM3_GetCapture1();
  TIM3_ClearFlag(TIM3_FLAG_CC1);

  /* wait a capture on cc1 */
  while ((TIM3->SR1 & TIM3_FLAG_CC1) != TIM3_FLAG_CC1);
    /* Get CCR1 value*/
  ICValue2 = TIM3_GetCapture1();
  TIM3_ClearFlag(TIM3_FLAG_CC1);

  /* Disable IC1 input capture */
  TIM3->CCER1 &= (uint8_t)(~TIM3_CCER1_CC1E);
  /* Disable timer3 */
  TIM3_Cmd(DISABLE);
#endif /* (STM8S903) || (STM8S103) || (STM8S003) || (STM8S001) */

  /* Compute LSI clock frequency */
  lsi_freq_hz = (8 * fmaster) / (ICValue2 - ICValue1);
  
  /* Disable the LSI measurement: LSI clock disconnected from timer Input Capture 1 */
  AWU->CSR &= (uint8_t)(~AWU_CSR_MSR);

 return (lsi_freq_hz);
}


#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/************************ Copyright (C)2017 LGG. All Rights Reserved *****END OF FILE****/
