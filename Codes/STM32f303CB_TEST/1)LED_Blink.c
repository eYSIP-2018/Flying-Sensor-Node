#include <stm32f30x.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_gpio.h>
 
#define LED GPIO_Pin_13
 
int main()
{
	long i;
	//Initialise to structures for setting pin modes
    	GPIO_InitTypeDef gpioB;
	//Start Clock for both port b
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE );
	 // Configure port B (LED)
    	GPIO_StructInit( &gpioB );
    	gpioB.GPIO_Mode = GPIO_Mode_OUT;
    	gpioB.GPIO_Pin  = LED;
    	GPIO_Init( GPIOB, &gpioB );
	// Blinking LEDS
      	while(1)
      	{
              		// On
	              GPIO_SetBits( GPIOB, LED );
              	              for( i = 0; i < 50000000; i++ );
              		// All off
              		GPIO_ResetBits( GPIOB, LED );
              		for( i = 0; i < 50000000; i++ );
      }
}
