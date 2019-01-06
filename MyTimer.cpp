/*
 * MyTimer.cpp
 *
 * Created: 11.02.2016 20:19:03
 *  Author: Christof
 */

#include "TLog.h"

extern volatile uint8_t do_sleep;

void init_mytimer(void)
{
	CLK.RTCCTRL = CLK_RTCSRC_EXTCLK_gc | CLK_RTCEN_bm;

	do {
		;/* Wait until RTC is not busy. */
	} while ( RTC.STATUS & RTC_SYNCBUSY_bm );

	RTC.PER = 107;
	RTC.CNT = 0;
	RTC.COMP = 0;
	RTC.CTRL = 7; // Teiler 1 ???
	RTC.INTCTRL	= RTC_OVFINTLVL_MED_gc;
}

void no_function( void )
{
	;
}

ISR ( RTC_OVF_vect )
{
uint8_t i;
	for (i=0;i<MYTIMER_NUM;i++)
	{
		switch(MyTimers[i].state )
		{
			case TM_STOP:
			break;
			case TM_RUN:
				if (MyTimers[i].actual==0)
				{
					if (MyTimers[i].restart == RESTART_YES)
					{
						MyTimers[i].actual = MyTimers[i].value;
						MyTimers[i].state = TM_RUN;
					}
					else
						MyTimers[i].state = TM_STOP;
					if (MyTimers[i].OnReady!=NULL)
						MyTimers[i].OnReady(3);
				}
				MyTimers[i].actual--;
			break;
			case TM_START:
				MyTimers[i].actual = MyTimers[i].value;
				MyTimers[i].state = TM_RUN;
			break;

			case TM_RESET:
				MyTimers[i].state = TM_START;
			break;
		}
	}
}

// NOTHING_CLIMA_TODO ->  START_CONVERSION -> WAIT_CONVERSION -> GET_TEMPERATURE -> NOTHING_CLIMA_TODO
void nextSensorStatus(uint8_t test)
{
	switch (statusSensoren)
	{
		case KLIMASENSOR:
			switch(statusKlima)
			{
/*				case NOTHING_CLIMA_TODO:
					statusKlima = START_TCONVERSION;
				break;*/
				case WAIT_TCONVERSION:
					statusKlima = READ_TCONVERSION;
				break;
				case WAIT_HCONVERSION:
					statusKlima = READ_HCONVERSION;
				break;
			}
		break;
		case TEMPSENSOREN:
            switch(statusTemperature)
            {
                case NOTHING_TODO:
                    statusTemperature = START_CONVERSION;
                break;
                case WAIT_CONVERSION:
                    statusTemperature = GET_TEMPERATURE;
                break;
            }
		break;
		case LASTSENSOR:
			if (statusLastSensor==WAIT_LAST)
				statusLastSensor=READY_LAST;
		break;
	}
}

void nextReportStatus(uint8_t test)
{
	sendStatusReport = true;
	statusReport+=1;
	if( statusReport > LASTREPORT )
        statusReport = TEMPREPORT;
}
void LED_toggle(uint8_t test)
{
//	LED_KLINGEL_TOGGLE;
}

void sekundenTimer(uint8_t test)
{
    if( GET_STUFE1 != 0)
        stufe1Counter++;
    if( GET_STUFE2 != 0)
        stufe2Counter++;
}
void goto_sleep(uint8_t test)
{
}

