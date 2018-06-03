#include <stm32f30x.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_gpio.h>
 
#define LED GPIO_Pin_13
 
int main()
{
		long i;
		uint8_t button;
		//Initialise to structures for setting pin modes
		GPIO_InitTypeDef gpioA;
		GPIO_InitTypeDef gpioB;
		//Start Clock for both port a and b
		RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE );
		RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE );
		// Configure port B (LED)
		GPIO_StructInit( &gpioB );
		gpioB.GPIO_Mode = GPIO_Mode_OUT;
		gpioB.GPIO_Pin  = LED;
		GPIO_Init( GPIOB, &gpioB );
		// Configure port A (Button)
		GPIO_StructInit( &gpioA );
		gpioA.GPIO_Mode = GPIO_Mode_IN;
		gpioA.GPIO_Pin  = LED;
		GPIO_Init( GPIOA, &gpioA );
 
      // Blinking LEDS
      while(1)
      {
              
			button =	GPIO_ReadInputDataBit(GPIOA,LED);
			if(button==1)
			{
				// On
				GPIO_SetBits( GPIOB, LED );
				//for( i = 0; i < 50000000; i++ );
       		}
			else
			{
				// All off
				GPIO_ResetBits( GPIOB, LED );
				//for( i = 0; i < 50000000; i++ );
			}
      }
}
