#include <stm32f30x.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_usart.h>
#define BTN GPIO_Pin_5
#define LED GPIO_Pin_13

unsigned char bytes[18];
int count=0;
//Flag to check whether previous pos data has been successfully received
int resp=1;

void initBTN()
{
		GPIO_InitTypeDef gpioA;
		RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE );
		// Configure port A (Button)
		GPIO_StructInit( &gpioA );
		gpioA.GPIO_Mode = GPIO_Mode_IN;
		gpioA.GPIO_Pin  = BTN;
		GPIO_Init( GPIOA, &gpioA );
}

void initLED()
{
		GPIO_InitTypeDef gpioA;
		GPIO_StructInit( &gpioA );
		gpioA.GPIO_Mode = GPIO_Mode_OUT;
		gpioA.GPIO_Pin  = LED;
		GPIO_Init( GPIOA, &gpioA );
}
void initUSART()
{
	/* Enable USART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
  /* Connect PA2 to USART2_Tx */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);
  /* Connect PA3 to USART2_Rx */
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
  USART_InitStructure.USART_BaudRate = 115200;
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

 void OutString(unsigned char *s)
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
          bytes[count] = (unsigned char) USART_ReceiveData(USART2); //receive a char
				  count=count+1;
					if(count==18)
					{
						resp=1;
					}
      }
			//OutChar(USART_Temp_Data);
}
int main()
{
	initBTN();
	initLED();
	initUSART();
	EnableUSART2Interrupt();
	uint8_t button;
	while(1)
  {
		
		button =	GPIO_ReadInputDataBit(GPIOA,BTN);
		if(button==1 && resp==1)
		{
				GPIO_SetBits( GPIOA, LED );
				delay();
				delay();
				count=0;
			  resp=0;
				OutChar(0x02);
				OutChar(0x00);
				
		}
		else
		{
				GPIO_ResetBits( GPIOA, LED );
		}
  }
}
