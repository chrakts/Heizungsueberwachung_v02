/*
* TempSensor.h
*
* Created: 09.04.2017 08:41:29
* Author: Christof
*/


#ifndef __TEMPSENSOR_H__
#define __TEMPSENSOR_H__
#include <stdint.h>

#include "OneWire.h"
#include "Slaves/Sensors/DS18B20/DS18B20.h"
//#include "TLog.h"

using namespace OneWire;
using namespace OneWire::crc;
using namespace RomCommands;

class TempSensor : public DS18B20
{
	//variables
	public:
	protected:
	private:

	//functions
	public:
	TempSensor(RandomAccessRomIterator &selector,bool hLP, uint8_t num) : DS18B20(selector)
	{
		this->hasLocalPower=hLP;
		historyCounter = 0;
		historyPointer = 0;
		Info.name[0] = 0;
		Info.number = num;
		Info.temperature_bits = 12;
		Info.historyMax = 32;
		Info.caliCoefficients[0]= 0.0;
		Info.caliCoefficients[1]= 1.0;
		Info.caliCoefficients[2]= 0.0;
		Info.caliCoefficients[3]= 0.0;
	} //TempSensor
	~TempSensor();
	void setRomID(const RomId & romId);
	void setName(char const *nname);
	char *getName();
	void setNumber(uint8_t num);
	uint8_t getNumber();
	void setTemperatureBits(uint8_t temp);
	uint8_t getTemperatureBits();
	double *getCalibrationCoefficient();
	uint8_t compareName(char *cname);
	uint8_t compareID(OneWire::RomId *cID);
	uint8_t compareNumber(uint8_t num);
	void setCalibrationCoefficient(double *coeff);
	void startConversion(bool startAll);
	void waitConversion();
	uint8_t readTemperature(int16_t & temp);
	float caliTemperature(float temp);
	void addTemperature(int16_t temp);
	float getMeanTemperature();
	uint8_t setHistoryMax(uint8_t history);
	uint8_t getHistoryMax();
	float getCaliMeanTemperature();
	uint8_t saveToEEProm(SENSINFOS  *buffer );
	uint8_t getFromEEProm();

	protected:
	private:
	TempSensor( const TempSensor &c );
//	TempSensor& operator=( const TempSensor &c );
	bool hasLocalPower;
	SensInfos Info;

	int16_t history[32];
	uint8_t historyCounter;
	uint8_t historyPointer;

}; //TempSensor

#endif //__TEMPSENSOR_H__
