#include "stm32f30x.h"
#include <stm32f30x_rcc.h>
#include <stm32f30x_gpio.h>
#include "stm32f30x_exti.h"
#include "stm32f30x_syscfg.h"
#include <stm32f30x_tim.h>

#define LED GPIO_Pin_13
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
    timerInitStructure.TIM_Period = 500;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &timerInitStructure);
    TIM_Cmd(TIM2, ENABLE);
}
 
int main()
{
    GPIO_Config();
    InitializeTimer();
 
    for (;;)
    {
        int timerValue = TIM_GetCounter(TIM2);
        if (timerValue == 400)
            GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_SET);
        else if (timerValue == 500)
            GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_RESET);
    }
}

