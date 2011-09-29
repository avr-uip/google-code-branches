//#include "uip.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "water_timer.h"


void water_timer_init(void)
{

}

void water_timer_appcall(void)
{
	/*
	is it time to update our clock with the rtc chip?
		if so, do so, and set the timer to do so later.

	check all active timers, water is on ... should it go off?
	
	have we loaded the events for this hour yet?
		if not, scan the eeprom and load events into ram
	
	scan the event list and look for events which are supposed to start now
		if one is found, start it and set a timer for the event
	*/

}

