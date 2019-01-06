/*
* TLog.cpp
*
* Created: 16.03.2017 13:03:01
* Author : a16007
*/

#include "TLog.h"

using namespace OneWire;
using namespace RomCommands;

void setup()
{
	PORTA_DIRSET = 0xff; // alles Ausgang, sind nur 2 LEDs angeschlossen
	PORTA_OUTSET = 0xff;

	PORTB_DIRSET = 0xff;; // nichts angeschlossen

	PORTC_DIRSET = PIN1_bm | PIN2_bm; // I2C-SCK und 1W-Suspend
    PORTC_OUTSET = PIN2_bm;

	PORTD_DIRSET = PIN0_bm | PIN1_bm | PIN3_bm ;
	PORTD_DIRCLR = PIN2_bm | PIN4_bm | PIN5_bm | PIN6_bm | PIN7_bm ;
	PORTD_OUTCLR = PIN0_bm | PIN1_bm;

	PORTE_DIRSET = PIN1_bm; // SCLK von Sensirion-Chip

	uint8_t i;
	init_clock(SYSCLK,PLL);

	for(i=0;i<20;i++)
	{
		LED_ROT_TOGGLE;
		_delay_ms(50);
	}
	for(i=0;i<20;i++)
	{
		LED_GRUEN_TOGGLE;
		_delay_ms(50);
	}
	initReadMonitor();
	initBusyCounter();
	//init_mytimer();

	PMIC_CTRL = PMIC_LOLVLEX_bm | PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm;
	sei();

	cnet.open(Serial::BAUD_57600,F_CPU);
    s_connectionreset();
}

void setup_twi()
{
    char romBuf[40];
    bool last_dev = false;

	OneWireMaster::CmdResult result = owm.begin(&twiC_Master,0x18);
	cnet.sendInfo("Master Ready",BROADCAST);
	if(result != OneWireMaster::Success)
	{
        cnet.sendAlarm("Failed 1W Master",BROADCAST);
		while(1);
	}
	result = owm.OWReset();
	if(result == OneWireMaster::Success)
	{
		result = OWFirst(owm, searchState);
		if(result == OneWireMaster::Success)
		{
			uint8_t temp_index = 0;
			do
			{
				cnet.sendInfo("Search sensor: ",BROADCAST);
				last_dev = searchState.last_device_flag;
				if( (searchState.romId.familyCode() == 0x28) | (searchState.romId.familyCode() == 0x10))
				{
					if (actNumberSensors<NUMBER_OF_TEMPSENSORS)
					{
						tempSensors[actNumberSensors] = new TempSensor(selector,true,temp_index);
						temp_index++;
						tempSensors[actNumberSensors]->setRomID(searchState.romId);
						buffer_rom_id(romBuf,searchState.romId);
						cnet.sendInfo(romBuf,BROADCAST);
						actNumberSensors++;
					}
					else
					{
						cnet.sendWarning("Too much sensors",BROADCAST);
					}
				}
				result = OWNext(owm, searchState);
			}
			while( (result == OneWireMaster::Success) && (last_dev==false) );
		}
		else
		{
			//cnet.print("OWFirst failed with error code: ");
			//cnet.println(result, Serial::DEC);
		}
	}
	else
	{
		cnet.println("No 1-wire devices");
	}
	sprintf(romBuf,"No. Sensoren:%d", actNumberSensors);
	cnet.sendInfo(romBuf,BROADCAST);
}

int main(void)
{
uint8_t reportStarted = false;
static uint8_t DS18B20ToReport = 0;
	setup();
	init_mytimer();
	setup_twi();
	MyTimers[TIMER_SEKUNDE].state = TM_START;
	uint8_t sensorReady=SENSOR_READY;
	while (1)
	{
		comStateMachine(&cnet);
		doJob(&cnet);
		switch(statusSensoren)
		{
			case KLIMASENSOR:
            LED_ROT_ON;
				sensorReady = doClima();
                    LED_ROT_OFF;
			break;
			case TEMPSENSOREN:
			    sensorReady = doTemperature();
            break;
			case LASTSENSOR:
				sensorReady = doLastSensor();
			break;
		}
		if (sensorReady==SENSOR_READY)
		{
			statusSensoren++;
			if (statusSensoren>LASTSENSOR)
			{
				statusSensoren = KLIMASENSOR;
				if(reportStarted==false)
                {
                    reportStarted = true;
                    MyTimers[TIMER_REPORT].state = TM_START;
                }
			}
		}
		if( sendStatusReport )
        {
            char buffer[16];
            sendStatusReport = false;
            MyTimers[TIMER_REPORT].value = REPORT_BETWEEN_SENSORS;
            MyTimers[TIMER_REPORT].state = TM_START;
            switch(statusReport)
            {
                case TEMPREPORT:
                    sprintf(buffer,"%f",(double)fTemperatur);
                    cnet.sendStandard(buffer,BROADCAST,'C','1','t','F');
                break;
                case HUMIREPORT:
                    sprintf(buffer,"%f",(double)fHumidity);
                    cnet.sendStandard(buffer,BROADCAST,'C','1','h','F');
                break;
                case ABSHUMIREPORT:
                    sprintf(buffer,"%f",(double)fAbsHumitdity);
                    cnet.sendStandard(buffer,BROADCAST,'C','1','a','F');
                break;
                case DEWPOINTREPORT:
                    sprintf(buffer,"%f",(double)fDewPoint);
                    cnet.sendStandard(buffer,BROADCAST,'C','1','d','F');
                break;
                case DS18B20REPORT:
                    if(actNumberSensors>0)
                    {
                        sprintf(buffer,"%f",(double)(tempSensors[DS18B20ToReport]->getMeanTemperature()));
                        cnet.sendStandard(buffer,BROADCAST,'T',int('a')+DS18B20ToReport,'t','F');
                        DS18B20ToReport++;
                    }
                    else
                    {
                        DS18B20ToReport=255;
                    }
                    if(DS18B20ToReport>=actNumberSensors)
                    {
                        DS18B20ToReport = 0;
                    }
                    else
                    {
                        statusReport--; // damit wird erreicht, dass der gleiche Report mit neuer Sensornummer abläuft
                    }
                break;
                case STUFE1REPORT:

                    sprintf(buffer,"%d",stufe1Counter);
                    stufe1Counter = 0;
                    cnet.sendStandard(buffer,BROADCAST,'H','1','s','d');
                break;
                case STUFE2REPORT:

                    sprintf(buffer,"%d",stufe2Counter);
                    stufe2Counter = 0;
                    cnet.sendStandard(buffer,BROADCAST,'H','1','S','d');
                break;
                case ALARMREPORT:
                    if(GET_ALARM!=0)
                        cnet.sendStandard("ON",BROADCAST,'H','1','a','t');
                    else
                        cnet.sendStandard("OFF",BROADCAST,'H','1','a','t');
                break;
                case WATERREPORT:
                    if(GET_WATER!=0)
                        cnet.sendStandard("ON",BROADCAST,'H','1','w','t');
                    else
                        cnet.sendStandard("OFF",BROADCAST,'H','1','w','t');
                break;
                case LASTREPORT:
                    MyTimers[TIMER_REPORT].value = REPORT_BETWEEN_BLOCKS;
                    MyTimers[TIMER_REPORT].state = TM_START;
                break;
            }
        }
	}
}

uint8_t doLastSensor()
{
	switch( statusLastSensor )
	{
		case NOTHING_LAST_TODO:
			MyTimers[TIMER_SENSOREN].value = 100;
			MyTimers[TIMER_SENSOREN].state = TM_START;
			statusLastSensor = WAIT_LAST;
		break;
		case READY_LAST:
			statusLastSensor = NOTHING_LAST_TODO;
		break;
	}
	return statusLastSensor;
}


uint8_t doTemperature()
{
static uint8_t sensorToRead=0;
    switch(statusTemperature)
    {
        case NOTHING_TODO: // ca. 4.75ms
            tempSensors[0]->startConversion(true);
            MyTimers[TIMER_SENSOREN].value = 75;
            MyTimers[TIMER_SENSOREN].state = TM_START;
            statusTemperature = WAIT_CONVERSION;
        break;
        case GET_TEMPERATURE:
            if(actNumberSensors>0)
            {
                int16_t temp;
                tempSensors[sensorToRead]->readTemperature(temp);
    //			ftemp = tempSensors[sensorToRead]->caliTemperature(tempSensors[sensorToRead]->getMeanTemperature() )*1000.0;
                sensorToRead++;
            }
            else
            {
                sensorToRead = 255;
            }
            if (sensorToRead>=actNumberSensors)
            {
                MyTimers[TIMER_SENSOREN].value = 25+measureRate_100ms-10;
                MyTimers[TIMER_SENSOREN].state = TM_START;
                statusTemperature = NOTHING_TODO;
                sensorToRead = 0;
            }
        break;
    }
    return(statusTemperature);
}

uint8_t doClima()
{
static unsigned char crcSH11;
uint8_t error;
static uint16_t iTemperature,iHumidity;
	switch(statusKlima)
	{
		case NOTHING_CLIMA_TODO:
			statusKlima = START_TCONVERSION;
		break;
		case START_TCONVERSION: // Durchlaufzeit ca. 55 µs
			crcSH11 = 0;
			error=startConversion(MEASURE_TEMP,&crcSH11);
			if (error==0)
			{
				statusKlima = WAIT_TCONVERSION;
				MyTimers[TIMER_SENSOREN].value = 32;
				MyTimers[TIMER_SENSOREN].state = TM_START;
			}
			else
				statusKlima = NOTHING_CLIMA_TODO;
		break;
		case READ_TCONVERSION:  // Durchlaufzeit ca. 82 µs
			error = readConversion(&iTemperature,&crcSH11);
			if (error==0)
			{
				statusKlima = START_HCONVERSION;
			}
			else
				statusKlima = NOTHING_CLIMA_TODO;
		break;
		case START_HCONVERSION:
			error=startConversion(MEASURE_HUMI,&crcSH11);
			if (error==0)
			{
				statusKlima = WAIT_HCONVERSION;
				MyTimers[TIMER_SENSOREN].value = 8;
				MyTimers[TIMER_SENSOREN].state = TM_START;
			}
			else
				statusKlima = NOTHING_CLIMA_TODO;
		break;
		case READ_HCONVERSION:
			error = readConversion(&iHumidity,&crcSH11);
			if (error==0)
			{
				statusKlima = CALC_CONVERSION1;
			}
			else
				statusKlima = NOTHING_CLIMA_TODO;
		break;
		case CALC_CONVERSION1:  // Durchlaufzeit ca. 58 µs
			fHumidity = (float)iHumidity;
			fTemperatur = (float)iTemperature;
			calc_sth11(&fHumidity ,&fTemperatur);
			fHumidity = fHumidity+fHumidity*fHumidity/350.0-3.0;
			fTemperatur = fTemperatur-2.0-fTemperatur/100.0;
			statusKlima = CALC_CONVERSION2;
		break;
		case CALC_CONVERSION2:  // Durchlaufzeit ca. 141
			fDewPoint = calc_dewpoint_float(fHumidity,fTemperatur); // calculate dew point temperature
			statusKlima = CALC_CONVERSION3;
		break;
		case CALC_CONVERSION3:  // Durchlaufzeit ca. 230 µs
			fAbsHumitdity = abs_feuchte(fHumidity,fTemperatur); // calculate dew point temperature
			statusKlima = NOTHING_CLIMA_TODO;
/*			MyTimers[TIMER_SENSOREN].value = 100;
			MyTimers[TIMER_SENSOREN].state = TM_START;*/
//			LED_ROT_OFF;
		break;
	}
	return(statusKlima);
}

//*********************************************************************
void buffer_rom_id(char *buffer,OneWire::RomId & romId)
{
	char temp[3];
	strcpy(buffer,"0x");
	for(uint8_t idx = 0; idx < RomId::byteLen; idx++)
	{
		sprintf(temp,"%x",romId[idx]);
		strcat(buffer,temp);
	}
}

void init_clock(int sysclk, int pll)
{
	CLK_t *mein_clock;
	OSC_t *mein_osc;
	mein_clock = &CLK;
	mein_osc = &OSC;
	switch(sysclk)
	{
		case QUARZ:
			mein_osc->XOSCCTRL = OSC_FRQRANGE_12TO16_gc | OSC_XOSCSEL_XTAL_16KCLK_gc;
//			mein_osc->XOSCCTRL = OSC_FRQRANGE_12TO16_gc | OSC_XOSCPWR_bm | OSC_XOSCSEL_XTAL_16KCLK_gc;
			mein_osc->CTRL = OSC_XOSCEN_bm | OSC_RC2MEN_bm | OSC_RC32KEN_bm; // schaltet die 32 MHz-Clock ein

			while((mein_osc->STATUS & OSC_XOSCRDY_bm) == 0)			// wartet bis diese stabil
			;
			while((mein_osc->STATUS & OSC_RC32KRDY_bm) == 0)		// wartet bis diese stabil
			;

			if ( (pll>0) & (pll<16) )
			{
				mein_osc->PLLCTRL = OSC_PLLSRC_XOSC_gc | pll;
				mein_osc->CTRL = OSC_PLLEN_bm | OSC_XOSCEN_bm | OSC_RC2MEN_bm | OSC_RC32KEN_bm; // schaltet zusätzlich die PLL ein

				while((mein_osc->STATUS & OSC_PLLRDY_bm) == 0)		// wartet bis diese stabil
				;
				CCP = CCP_IOREG_gc;										// geschuetztes Register freigeben
				mein_clock->CTRL = CLK_SCLKSEL_PLL_gc;					// umschalten auf PLL-Clock
				mein_osc->CTRL = OSC_PLLEN_bm | OSC_XOSCEN_bm | OSC_RC32KEN_bm;
			}
			else
			{
				CCP = CCP_IOREG_gc;										// geschuetztes Register freigeben
				mein_clock->CTRL = CLK_SCLKSEL_XOSC_gc;					// umschalten auf XOSC-Clock
				mein_osc->CTRL = OSC_XOSCEN_bm | OSC_RC32KEN_bm;
			}
		break; // QUARZ
		case CLK2M:
			mein_osc->CTRL = OSC_RC2MEN_bm | OSC_RC32KEN_bm; // schaltet die 2 MHz-Clock ein
			while((mein_osc->STATUS & OSC_RC2MRDY_bm) == 0)  // wartet bis diese stabil
			;
			while((mein_osc->STATUS & OSC_RC32KRDY_bm) == 0)  // wartet bis diese stabil
			;
			CCP = CCP_IOREG_gc;								// geschuetztes Register freigeben
			mein_clock->CTRL = CLK_SCLKSEL_RC2M_gc;		// umschalten auf 2 MHz-Clock
//			CLKSYS_AutoCalibration_Enable(OSC_RC2MCREF_RC32K_gc,false); // OSC_RC32MCREF_bm
		break;
		case CLK32M:
			mein_osc->CTRL = OSC_RC32MEN_bm | OSC_RC2MEN_bm | OSC_RC32KEN_bm; // schaltet die 32 MHz-Clock ein
			while((mein_osc->STATUS & OSC_RC32MRDY_bm) == 0)  // wartet bis diese stabil
			;
			while((mein_osc->STATUS & OSC_RC32KRDY_bm) == 0)  // wartet bis diese stabil
			;
			CCP = CCP_IOREG_gc;								// geschuetztes Register freigeben
			mein_clock->CTRL = CLK_SCLKSEL_RC32M_gc;		// umschalten auf 32 MHz-Clock
			mein_osc->CTRL = OSC_RC32MEN_bm | OSC_RC32KEN_bm;		// abschalten der 2 MHz-Clock
//			CLKSYS_AutoCalibration_Enable(OSC_RC32MCREF_RC32K_gc,false); // OSC_RC32MCREF_bm
		break;
	}
}

/*! \brief This function enables automatic calibration of the selected internal
 *         oscillator.
 *
 *  Either the internal 32kHz RC oscillator or an external 32kHz
 *  crystal can be used as a calibration reference. The user must make sure
 *  that the selected reference is ready and running.
 *
 *  \param  clkSource    Clock source to calibrate, either OSC_RC2MCREF_bm or
 *                       OSC_RC32MCREF_bm.
 *  \param  extReference True if external crystal should be used as reference.
 */
/*
void CLKSYS_AutoCalibration_Enable( uint8_t clkSource, bool extReference )
{
	OSC.DFLLCTRL = ( OSC.DFLLCTRL & ~clkSource ) |
	               ( extReference ? clkSource : 0 );
	if (clkSource == OSC_RC2MCREF_bm) {
		DFLLRC2M.CTRL |= DFLL_ENABLE_bm;
	} else if (clkSource == OSC_RC32MCREF_RC32K_gc) {   // OSC_RC32MCREF_bm
		DFLLRC32M.CTRL |= DFLL_ENABLE_bm;
	}
}
*/
