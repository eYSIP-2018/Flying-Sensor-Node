#include "PlutoPilot.h"
#include "led.h"
#include "control.h"
#include "althold.h"
#include "Sensor.h"
#include "print.h"
#include "utils.h"

//Timer for blinking
Timer *blink1Timer;
Timer *blink2Timer;
Timer *blink3Timer;

//The setup function is called once at Pluto's hardware startup
void plutoInit()
{
// Add your hardware initialization code here
}



//The function is called once before plutoFly when you activate UserCode
void onPilotStart()
{
  // do your one time stuffs here
	blink1Timer=new Timer();
	blink2Timer=new Timer();
	blink3Timer=new Timer();
	Control.disableFlightStatus(true);
	ledOp(L_MID, OFF);
	ledOp(L_LEFT, OFF);
	ledOp(L_RIGHT, OFF);

}



// The loop function is called in an endless loop
void plutoPilot()
{

	//Add your repeated code here
	if(blink1Timer->start(750))
	{
		ledOp(L_MID, TOGGLE);
	}
	if(blink2Timer->start(500))
	{
		ledOp(L_LEFT, TOGGLE);
	}
	if(blink3Timer->start(1000))
	{
		ledOp(L_RIGHT, TOGGLE);
	}

	Print.monitor("Temperature", Barometer.getTemperature());
	Print.redGraph(Althold.getEstimatedAltitude());


}



//The function is called once after plutoFly when you deactivate UserCode
void onPilotFinish()
{

	 // do your cleanup stuffs here

	 Control.disableFlightStatus(false);

}
