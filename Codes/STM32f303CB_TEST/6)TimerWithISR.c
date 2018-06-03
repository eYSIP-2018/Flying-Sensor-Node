#include "stm32f30x.h"
#include <stm32f30x_rcc.h>
#include <stm32f30x_gpio.h>
#include "stm32f30x_exti.h"
#include "stm32f30x_syscfg.h"
#include <stm32f30x_tim.h>

#define LED GPIO_Pin_13
int flag=0;
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
 
void InitializeTimer()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
 
    TIM_TimeBaseInitTypeDef timerInitStructure; 
    timerInitStructure.TIM_Prescaler = 40000;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 800;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &timerInitStructure);
    TIM_Cmd(TIM2, ENABLE);	
		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void EnableTimerInterrupt()
{
    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure.NVIC_IRQChannelSubPriority = 0;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
}

void TIM2_IRQHandler()
{
				if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET && flag ==1)
				{
						GPIO_SetBits(GPIOB, LED );
						flag=0;
						/* Clear the EXTI line 13 pending bit */
						TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
				}
				else if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET && flag ==0)
				{
						GPIO_ResetBits(GPIOB, LED );
						/* Clear the EXTI line 13 pending bit */
						TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
						flag=1;
				}
}

int main()
{
    GPIO_Config();
    InitializeTimer();
	  EnableTimerInterrupt();
		while(1){}
}


