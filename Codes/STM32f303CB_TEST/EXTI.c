#include "stm32f30x.h"
#include <stm32f30x_rcc.h>
#include <stm32f30x_gpio.h>
#include "stm32f30x_exti.h"
#include "stm32f30x_syscfg.h"

#define LED GPIO_Pin_13
int flag=1;
 

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

 

void EXTI13_Config(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;


  /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
 
  /* Configure PA13 pin as input floating */
	GPIO_StructInit( &GPIO_InitStructure );
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

   /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Connect EXTI13 Line to PA13 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource13);

   /* Configure EXTI13 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line13;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure); //Sets the various registers

  /* Enable and set EXTI13 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

 
void EXTI15_10_IRQHandler(void)
{
  //toggle ked on interrupt
  if (EXTI_GetITStatus(EXTI_Line13) != RESET && flag ==1)
  {
		GPIO_SetBits(GPIOB, LED );
		flag=0;
		/* Clear the EXTI line 13 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line13);
	}
	else if (EXTI_GetITStatus(EXTI_Line13) != RESET && flag ==0)
  {
		GPIO_ResetBits(GPIOB, LED );
		/* Clear the EXTI line 13 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line13);
		flag=1;
	}
}

int main(void)
{
  /* Configure PB3 as an output */
  GPIO_Config();
  /* Configure PB4 in interrupt mode */
  EXTI13_Config();
  /* Infinite loop */
  while (1)
  {
		//EXTI15_10_IRQHandler();
  }
}

