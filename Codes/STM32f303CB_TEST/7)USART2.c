#include "stm32f30x.h"
#include <stm32f30x_rcc.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_usart.h>

#define LED GPIO_Pin_13
int flag=1;
unsigned char USART_Temp_Data;
 
/**************************************************************************************/
 void GPIO_Config(void)
{
    GPIO_InitTypeDef gpioB;
    /* Enable GPIOB clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    /* Configure PB13 pin as output */
		GPIO_StructInit( &gpioB );
		gpioB.GPIO_Mode = GPIO_Mode_OUT;
		gpioB.GPIO_Pin  = LED;
		GPIO_Init( GPIOB, &gpioB );
}
void RCC_Configuration(void)
{
  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
 
  /* Enable USART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
}
 
/**************************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
 
  /* Connect PD5 to USART2_Tx */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);
 
  /* Connect PD6 to USART2_Rx */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_7);
 
  /* Configure USART Tx/Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	 /* Configure USART Tx/Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}
 
/**************************************************************************************/
 void USART2_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;
 
  /* USART resources configuration (Clock, GPIO pins and USART registers) ----*/
  /* USART configured as follow:
        - BaudRate = 9600 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
  /* USART configuration */
  USART_Init(USART2, &USART_InitStructure);
 
  /* Enable USART */
  USART_Cmd(USART2, ENABLE);
	
}
 
/**************************************************************************************/
 void OutString(char *s)
{
    while(*s)
    {
      while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // Wait for Empty
 
    USART_SendData(USART2, *s++);
    }
}

void OutChar(char s)
{
     while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // Wait for Empty
     USART_SendData(USART2, s);
 }
/**************************************************************************************/
 void digitalWrite(int a)
 {
	 if(a==1)
		GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_SET);
	 if(a==0)
		GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_RESET);
 }
 void delay()
 {
	 long i;
	 for(i=0;i<5000000;i++);
 }
 uint16_t USART_GetChar()
{
    // Wait until data is received
    while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);// Wait for Char
    // Read received char
    return USART_ReceiveData(USART2);
}

void EnableUSART2Interrupt()
{
    // Enable the USART RX Interrupt 
		NVIC_InitTypeDef NVIC_InitStructure;
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
}
void USART2_IRQHandler()
{
			if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//enter interrupt when STM32 receice data.
      {
         USART_ClearITPendingBit(USART2, USART_IT_RXNE);
         USART_Temp_Data = (unsigned char) USART_ReceiveData(USART2); //receive a char
      }
			OutChar(USART_Temp_Data);
}
int main(void)
{
	GPIO_Config();
  RCC_Configuration(); 
  GPIO_Configuration(); 
  USART2_Configuration();
	EnableUSART2Interrupt();
	OutString("This is an echo back test for USART2\r\n");
	while(1) // Don't want to exit
  {
			
  }
}
 
/**************************************************************************************/
 
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
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
