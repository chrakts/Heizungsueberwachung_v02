/*
* TempSensor.cpp
*
* Created: 09.04.2017 08:41:29
* Author: Christof
*/


#include "TLog.h"
#include "TempSensor.h"

// default constructor


// default destructor
TempSensor::~TempSensor()
{
} //~TempSensor

void TempSensor::setRomID(const RomId & romId)
{
	this->Info.romID = romId;
	this->setRomId(romId);
	getFromEEProm();
	if (romId.familyCode() == 0x10)
    this->setTemperatureBits(9);
}

void TempSensor::setName(char const * nname)
{
uint8_t i;
	for (i=0;i<NUMBER_OF_SENSORS_NAME_LENGTH;i++)
	{
		this->Info.name[i] = nname[i];
	}
}

char *TempSensor::getName()
{
	return(this->Info.name);
}

void TempSensor::setNumber(uint8_t num)
{
	this->Info.number = num;
}

uint8_t TempSensor::getNumber()
{
	return(this->Info.number);
}

double *TempSensor::getCalibrationCoefficient()
{
	return(this->Info.caliCoefficients);
}


void TempSensor::setTemperatureBits(uint8_t temp)
{
	this->Info.temperature_bits = temp;
}
uint8_t TempSensor::getTemperatureBits()
{
	return(this->Info.temperature_bits);
}

uint8_t TempSensor::compareName(char *cname)
{
	if( strcmp(this->Info.name,cname)==0 )
		return true;
	else
		return false;
}

/*
// uint8_t TempSensor::compareID(OneWire::RomId *cID)
// {
// 	if( this->romId() == *cID )
// 		return true;
// 	else
// 		return false;
// }
*/

uint8_t TempSensor::compareID(OneWire::RomId *cID)
{
	if( this->Info.romID == *cID )
		return true;
	else
		return false;
}

uint8_t TempSensor::compareNumber(uint8_t num)
{
	if( this->Info.number == num )
		return true;
	else
		return false;
}

void TempSensor::setCalibrationCoefficient(double *coeff)
{
uint8_t i;
	if(coeff!=NULL)
	{
		for (i=0;i<4;i++)
		{
			(this->Info.caliCoefficients)[i] = coeff[i];
		}
	}
}

void TempSensor::startConversion(bool startAll)
{
	DS18B20::startConversion(startAll,hasLocalPower);
}

void TempSensor::waitConversion()
{
	DS18B20::waitConversion(hasLocalPower);
}

float TempSensor::caliTemperature(float temp)
{
float temp2;
	temp2 = temp*temp;
	return( (this->Info.caliCoefficients)[0]+(this->Info.caliCoefficients)[1]*temp+(this->Info.caliCoefficients)[2]*temp2+(this->Info.caliCoefficients)[3]*temp2*temp );
}

uint8_t TempSensor::readTemperature(int16_t & temp)
{
OneWireSlave::CmdResult result;
	result = DS18B20::readTemperature(temp);
	if (result == OneWireSlave::Success)
	{
		addTemperature(temp);
		return true;
	}
	else
		return false;
}

void TempSensor::addTemperature(int16_t temp)
{
	history[historyPointer] = temp;

	if(historyCounter<Info.historyMax)
		historyCounter++;
	historyPointer++;
	if (historyPointer==Info.historyMax)
		historyPointer = 0;
}

float TempSensor::getMeanTemperature()
{
uint8_t i;
int16_t temptemp=0;
float ftemp;
float divisor;
//	cli();
	for (i=0;i<historyCounter;i++)
		temptemp += history[i];
//	sei();
//	divisor = (float)(historyCounter<<4);	// ist nur richtig bei 12-bit Auflösung
	divisor = (float)(historyCounter<<(this->Info.temperature_bits-8));	// ist immer richtig
	ftemp = ((float)temptemp) / divisor;
	return(ftemp);
}

uint8_t TempSensor::setHistoryMax(uint8_t history)
{
	if ( (history>0) && (history <33) )
	{
		this->Info.historyMax = history;
		historyCounter = 0;
		historyPointer=0;
		return(true);
	}
	else
		return(false);
}

uint8_t TempSensor::getHistoryMax()
{
	return(this->Info.historyMax);
}

float TempSensor::getCaliMeanTemperature()
{
float ftemp;
	ftemp = this->getMeanTemperature();
	ftemp = this->caliTemperature(ftemp);
	return(ftemp);
}

uint8_t TempSensor::saveToEEProm(SENSINFOS *buffer )
{
	eeprom_write_block(&Info,buffer,sizeof(SENSINFOS));
	return(true);
}

uint8_t TempSensor::getFromEEProm()
{
uint8_t i=0,found = false;
SENSINFOS tempSens;
	do
	{
		eeprom_read_block(&(tempSens),&(storedSensors[i]),sizeof(SENSINFOS));
		found = this->compareID(&(tempSens.romID));
		if(!found)
			i++;
	} while ( (!found) && (i<NUMBER_STORED_SENSORS) );
	if (found)
	{
		memcpy(&(this->Info),&(tempSens),sizeof(SENSINFOS));
//		eeprom_read_block(&(this->Info),&(storedSensors[i]),sizeof(SENSINFOS));
		return true;
	}
	else
	{
		return false;
	}
}
