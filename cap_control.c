/*
 cap_control.h - capacitance controller
 */

#include <avr/io.h>
#include <avr/pgmspace.h>

#include "config.h"
#include "motion_control.h"
//#include <util/delay.h>
//#include <math.h>
//#include <stdlib.h>
#include "nuts_bolts.h"
#include "stepper.h"
#include "cap_control.h"

#include "wiring_serial.h"

#define NZEROS 5
#define NPOLES 5
#define GAIN   1.894427191e+01
float xv[NZEROS+1];
float yv[NPOLES+1];

float capAverage;
unsigned long capTotal;
unsigned long capTimeout;

uint8_t senseSendPins[3];
uint8_t senseRecvPins[3];

float cc_getAverageVal()
{
	return capAverage;
}

void setCapTimeout(unsigned long timeout)
{
	capTimeout = timeout;
}

void cc_init()
{	
	// initilize capacitive sensing receive ports to be low.
	CAP_DDR   &= ~(1<<X_AXIS_CAP_RECV) & ~(1<<Y_AXIS_CAP_RECV) & ~(1<<Z_AXIS_CAP_RECV);
	CAP_PORT  &= ~(1<<X_AXIS_CAP_RECV) & ~(1<<Y_AXIS_CAP_RECV) & ~(1<<Z_AXIS_CAP_RECV);
	
	CAP_DDR   |=  (1<<X_AXIS_CAP_SEND) |  (1<<Y_AXIS_CAP_SEND) |  (1<<Z_AXIS_CAP_SEND);         // set sendpin to OUTPUT 
	
	END_MILL_CAP_DDR   &= ~(1<<END_MILL_CAP_RECV);
	END_MILL_CAP_PORT  &= ~(1<<END_MILL_CAP_RECV);
	END_MILL_CAP_DDR   |=  (1<<END_MILL_CAP_SEND);         // set sendpin to OUTPUT 
	
	// setup arrays for mapping axis number to pin number
	senseSendPins[0] = 1<<X_AXIS_CAP_SEND;
	senseSendPins[1] = 1<<Y_AXIS_CAP_SEND;
	senseSendPins[2] = 1<<Z_AXIS_CAP_SEND;
	
	senseRecvPins[0] = 1<<X_AXIS_CAP_RECV;
	senseRecvPins[1] = 1<<Y_AXIS_CAP_RECV;
	senseRecvPins[2] = 1<<Z_AXIS_CAP_RECV;
	
	int loopTimingFactor = 310;     // determined empirically -  a hack
	unsigned long CS_Timeout_Millis = (2000 * (float)loopTimingFactor * (float)F_CPU) / 16000000;
	setCapTimeout(CS_Timeout_Millis);
}

void initLowPass()
{
	xv[0] = 0; xv[1] = 0; xv[2] = 0; xv[3] = 0; xv[4] = 0; xv[5] =0; 
	yv[0] = 0; yv[1] = 0; yv[2] = 0; yv[3] = 0; yv[4] = 0; yv[5] =0;
}

void lowPassFilter()
{	
	xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; xv[4] = xv[5]; 
	xv[5] = capTotal / GAIN;
	yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; yv[4] = yv[5]; 
	yv[5] =   (xv[0] + xv[5]) + 5 * (xv[1] + xv[4]) + 10 * (xv[2] + xv[3])
	+ (  0.0000000000 * yv[0]) + ( -0.0557280900 * yv[1])
	+ ( -0.0000000000 * yv[2]) + ( -0.6334368540 * yv[3])
	+ ( -0.0000000000 * yv[4]);
}

//loopTimingFactor = 310;     // determined empirically -  a hack
//CS_Timeout_Millis = (2000 * (float)loopTimingFactor * (float)F_CPU) / 16000000;
//CS_AutocaL_Millis = 20000;
// 
// ex: getCapacitanceValue((1<<X_AXIS_CAP_SEND), (1<<X_AXIS_CAP_RECV), 20000 )
int getCapacitanceValue(int sendBit, int recvBit)
{	
	capTotal = 0;
	//	ddr   |= sendBit;         // set sendpin to OUTPUT 
	CAP_PORT  &= ~sendBit;        // set Send Pin Register low
	
	CAP_DDR   &= ~recvBit;        // set receivePin to input
	CAP_PORT  &= ~recvBit;        // set receivePin Register low to make sure pullups are off
	
	CAP_DDR   |= recvBit;         // set pin to OUTPUT - pin is now LOW AND OUTPUT
	CAP_DDR   &= ~recvBit;        // set pin to INPUT 
	
	CAP_PORT  |= sendBit;         // set send Pin High
	
	while( !(CAP_PIN & recvBit)  && (capTotal < capTimeout )){  // while receive pin is LOW AND total is positive value
		capTotal++;
	}
	
	if (capTotal > capTimeout){
		return -1;             //  total variable over timeout
	}
	
	// set receive pin HIGH briefly to charge up fully - because the while loop above will exit when pin is ~ 2.5V 
	CAP_PORT  |= recvBit;        // set receive pin HIGH - turns on pullup 
	CAP_DDR   |= recvBit;         // set pin to OUTPUT - pin is now HIGH AND OUTPUT
	CAP_DDR   &= ~recvBit;        // set pin to INPUT 
	CAP_PORT  &= ~recvBit;       // turn off pullup
	
	CAP_PORT  &= ~sendBit;        // set send Pin LOW
	
	while( (CAP_PIN & recvBit) && (capTotal < capTimeout)){        // while receive pin is HIGH  AND total is less than timeout
		capTotal++;
	}
	
	if (capTotal >= capTimeout){
		return -1;     // total variable over timeout
    }
	lowPassFilter();
	return 0;
}

int getEndMillCapacitanceValue(int sendBit, int recvBit)
{	
	capTotal = 0;
	//	ddr   |= sendBit;         // set sendpin to OUTPUT 
	END_MILL_CAP_PORT  &= ~sendBit;        // set Send Pin Register low
	
	END_MILL_CAP_DDR   &= ~recvBit;        // set receivePin to input
	END_MILL_CAP_PORT  &= ~recvBit;        // set receivePin Register low to make sure pullups are off
	
	END_MILL_CAP_DDR   |= recvBit;         // set pin to OUTPUT - pin is now LOW AND OUTPUT
	END_MILL_CAP_DDR   &= ~recvBit;        // set pin to INPUT 
	
	END_MILL_CAP_PORT  |= sendBit;         // set send Pin High
	
	while( !(END_MILL_CAP_PIN & recvBit)  && (capTotal < capTimeout )){  // while receive pin is LOW AND total is positive value
		capTotal++;
	}
	
	if (capTotal > capTimeout){
		return -1;             //  total variable over timeout
	}
	
	// set receive pin HIGH briefly to charge up fully - because the while loop above will exit when pin is ~ 2.5V 
	END_MILL_CAP_PORT  |= recvBit;        // set receive pin HIGH - turns on pullup 
	END_MILL_CAP_DDR   |= recvBit;         // set pin to OUTPUT - pin is now HIGH AND OUTPUT
	END_MILL_CAP_DDR   &= ~recvBit;        // set pin to INPUT 
	END_MILL_CAP_PORT  &= ~recvBit;       // turn off pullup
	
	END_MILL_CAP_PORT  &= ~sendBit;        // set send Pin LOW
	
	while( (END_MILL_CAP_PIN & recvBit) && (capTotal < capTimeout)){        // while receive pin is HIGH  AND total is less than timeout
		capTotal++;
	}
	
	if (capTotal >= capTimeout){
		return -1;     // total variable over timeout
    }
	lowPassFilter();
	return 0;
}

// TODO_MM try using a median filter instead of averaging. It is likely to get more consistent results.
int  cc_axisAverageCapValue(int axis, uint8_t numSamples)
{
	initLowPass();
	int i;
	for( i=0; i < NZEROS+1; i++)
	{
		if ( getCapacitanceValue(senseSendPins[axis], senseRecvPins[axis]) < 0) {
			return -1;
		} 
	}
	capAverage = 0;
	for( i=0; i < numSamples; i++)
	{
		if ( getCapacitanceValue(senseSendPins[axis], senseRecvPins[axis]) < 0) {
			return -1;
		}
		capAverage += yv[5];
	}
	capAverage /= numSamples;
	return 0;
}

int  cc_endMillAverageCapValue(uint8_t numSamples)
{
	initLowPass();
	int i;
	for( i=0; i < NZEROS+1; i++)
	{
		if ( getEndMillCapacitanceValue(1<<END_MILL_CAP_SEND, 1<<END_MILL_CAP_RECV) < 0) {
			return -1;
		} 
	}
	capAverage = 0;
	for( i=0; i < numSamples; i++)
	{
		if ( getEndMillCapacitanceValue(1<<END_MILL_CAP_SEND, 1<<END_MILL_CAP_RECV) < 0) {
			return -1;
		}
		capAverage += yv[5];
	}
	capAverage /= numSamples;
	return 0;
}

void cc_measure_cap(int selection)
{
	st_synchronize();
	if(selection == 0 || selection == -2)
	{
		printPgmString(PSTR("X Axis Val: "));
		if( cc_axisAverageCapValue( X_AXIS, 10*5 ) == 0)
		{
			printFloat(capAverage);
		} else {
			print_timed_out();
		}
		print_newline();
	}
	
	if(selection == 1 || selection == -2)
	{
		printPgmString(PSTR("Y Axis Val: "));
		if( cc_axisAverageCapValue(Y_AXIS, 10*5 ) == 0)
		{
			printFloat(capAverage);
		} else {
			print_timed_out();
		}
		print_newline();
	}
	
	if(selection == 2 || selection == -2)
	{
		printPgmString(PSTR("Z Axis Val: "));
		if( cc_axisAverageCapValue(Z_AXIS, 10*5 ) == 0)
		{
			printFloat(capAverage);
		} else {
			print_timed_out();
		}
		print_newline();
	}
	
	if(selection == -1 || selection == -2)
	{
		printPgmString(PSTR("End Mill Val: "));
		if( cc_endMillAverageCapValue( 10*5 ) == 0)
		{
			printFloat(capAverage);
		} else {
			print_timed_out();
		}
		print_newline();
	}
}


