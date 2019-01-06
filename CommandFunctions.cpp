/*
 * CommandFunctions.cpp
 *
 * Created: 26.04.2017 14:54:45
 *  Author: a16007
 */

#include "CommandFunctions.h"
#include "External.h"
#include "CRC_Calc.h"

void jobGotCRCError(Communication *output, char function,char address,char job, void * pMem)
{
	output->sendAnswer(fehler_text[CRC_ERROR],quelle_KNET,function,address,job,false);
}

void jobSetIDNumber(Communication *output, char function,char address,char job, void * pMem)
{
	if (strlen((char*)pMem)<=11)
	{
		eeprom_write_block((char*)pMem,IDNumber,strlen((char*)pMem)+1);
		output->sendPureAnswer(quelle_KNET,function,address,job,true);
	}
	else
		output->sendPureAnswer(quelle_KNET,function,address,job,false);
}

void jobSetSerialNumber(Communication *output, char function,char address,char job, void * pMem)
{
	if (strlen((char*)pMem)<=11)
	{
		eeprom_write_block((char*)pMem,SerialNumber,strlen((char*)pMem)+1);
		output->sendPureAnswer(quelle_KNET,function,address,job,true);
	}
	else
	output->sendPureAnswer(quelle_KNET,function,address,job,false);
}

void jobSetIndexNumber(Communication *output, char function,char address,char job, void * pMem)
{
	if (strlen((char*)pMem)<=2)
	{
		eeprom_write_block((char*)pMem,IndexNumber,strlen((char*)pMem)+1);
		output->sendPureAnswer(quelle_KNET,function,address,job,true);
	}
	else
		output->sendPureAnswer(quelle_KNET,function,address,job,false);
}

void jobGetIDNumber(Communication *output, char function,char address,char job, void * pMem)
{
	char temp[12];
	eeprom_read_block(temp,IDNumber,12);
	output->sendAnswer(temp,quelle_KNET,function,address,job,true);
}

void jobGetSerialNumber(Communication *output, char function,char address,char job, void * pMem)
{
	char temp[12];
	eeprom_read_block(temp,SerialNumber,12);
	output->sendAnswer(temp,quelle_KNET,function,address,job,true);
}

void jobGetIndex(Communication *output, char function,char address,char job, void * pMem)
{
	char temp[2];
	eeprom_read_block(temp,IndexNumber,2);
	output->sendAnswer(temp,quelle_KNET,function,address,job,true);
}

void jobGetCTemperatureSensor(Communication *output, char function,char address,char job, void * pMem)
{
	output->sendAnswerDouble(quelle_KNET,function,address,job,(double)fTemperatur,true);
}

void jobGetCHumiditySensor(Communication *output, char function,char address,char job, void * pMem)
{
	output->sendAnswerDouble(quelle_KNET,function,address,job,(double)fHumidity,true);
}

void jobGetCAbsHumiditySensor(Communication *output, char function,char address,char job, void * pMem)
{
	output->sendAnswerDouble(quelle_KNET,function,address,job,(double)fAbsHumitdity,true);
}

void jobGetCDewPointSensor(Communication *output, char function,char address,char job, void * pMem)
{
	output->sendAnswerDouble(quelle_KNET,function,address,job,(double)fDewPoint,true);
}

void jobSetSecurityKey(Communication *output, char function,char address,char job, void * pMem)
{
uint8_t ret = true;
	if (strcmp((char *)pMem,"Phe6%!kdf?+2aQ")==0)
	{
		SecurityLevel = PRODUCTION;
	}
	else if(strcmp((char *)pMem,"D=&27ane%24dez")==0)
	{
		SecurityLevel = DEVELOPMENT;
	}
	else
	{
		SecurityLevel = CUSTOMER;
		ret = false;
	}
	output->sendAnswerInt(quelle_KNET,function,address,job,SecurityLevel,ret);
}

void jobGetSecurityKey(Communication *output, char function,char address,char job, void * pMem)
{
	output->sendAnswerInt(quelle_KNET,function,address,job,SecurityLevel,true);
}

void jobTestTripleIntParameter(Communication *output, char function,char address,char job, void * pMem)
{
	uint16_t *pointer;
	output->sendAnswer("Parameter: ",quelle_KNET,function,address,job,true);
	pointer = (uint16_t*) pMem;
	output->sendAnswerInt(quelle_KNET,function,address,job,pointer[0],true);
	output->sendAnswerInt(quelle_KNET,function,address,job,pointer[1],true);
	output->sendAnswerInt(quelle_KNET,function,address,job,pointer[2],true);
}

void jobTestStringParameter(Communication *output, char function,char address,char job, void * pMem)
{
CRC_Calc mycrc;

	mycrc.String((const char*) pMem);
	output->sendAnswerInt(quelle_KNET,function,address,job,mycrc.Get_CRC(),true);
}

void jobTestFloatParameter(Communication *output, char function,char address,char job, void * pMem)
{
	double *pointer;
	output->sendAnswer("Parameter: ",quelle_KNET,function,address,job,true);
	pointer = (double*) pMem;
	output->sendAnswerDouble(quelle_KNET,function,address,job,pointer[0],true);
}

void jobTestParameter(Communication *output, char function,char address,char job, void * pMem)
{
	uint16_t *pointer;
	output->sendAnswer("Parameter: ",quelle_KNET,function,address,job,true);
	pointer = (uint16_t*) pMem;
	output->sendAnswerInt(quelle_KNET,function,address,job,pointer[0],true);
}

void jobGetCompilationDate(Communication *output, char function,char address,char job, void * pMem)
{
char temp[20];
	strcpy(temp,Compilation_Date);
	output->sendAnswer(temp,quelle_KNET,function,address,job,true);
}

void jobGetCompilationTime(Communication *output, char function,char address,char job, void * pMem)
{
char temp[20];
	strcpy(temp,Compilation_Time);
	output->sendAnswer(temp,quelle_KNET,function,address,job,true);
}

void jobGetFreeMemory(Communication *output, char function,char address,char job, void * pMem)
{
extern int __heap_start, *__brkval;
int v;

	uint16_t mem = (uint16_t) &v - (__brkval == 0 ? (uint16_t) &__heap_start : (uint16_t) __brkval);
	output->sendAnswerInt(quelle_KNET,function,address,job,mem,true);
}


void jobGetNumberOfSensors(Communication *output, char function,char address,char job, void * pMem)
{
	output->sendAnswerInt(quelle_KNET,function,address,job,actNumberSensors,true);
}

void saveSensorsToEEProm(Communication *output, char function,char address,char job, void * pMem)
{
uint8_t i;
	for (i=0;i<actNumberSensors;i++)
	{
		tempSensors[i]->saveToEEProm(&storedSensors[i]);
	}
	output->sendPureAnswer(quelle_KNET,function,address,job,true);
}

void jobSetActiveSensorById(Communication *output, char function,char address,char job, void * pMem)
{
uint8_t found = false, index = 0,i;
char *cId;
OneWire::RomId ID;
	cId = (char *)pMem;
	if (strlen(cId)==16 )
	{
		strlwr(cId);
		for (i=0;i<16;i++)
		{
			if (cId[i]<58)
				cId[i] -= '0';
			else
				cId[i] -= 'a'-10;
		}
		for(i=0;i<8;i++)
			ID[i] = (cId[2*i]) * 16 + cId[2*i+1];
		do
		{
			found = tempSensors[index]->compareID(&ID);
			if( found )
				activeSensor = tempSensors[index];
			index++;
		} while ( (!found) && (index < actNumberSensors) );
	}
	output->sendAnswerInt(quelle_KNET,function,address,job,found,true);
}

void jobSetActiveSensorByName(Communication *output, char function,char address,char job, void * pMem)
{
uint8_t found = false, index = 0;
	do
	{
		found = tempSensors[index]->compareName((char *)pMem);
		if( found )
			activeSensor = tempSensors[index];
		index++;
	} while ( (!found) && (index < actNumberSensors) );
	output->sendAnswerInt(quelle_KNET,function,address,job,found,true);
}

void jobSetActiveSensorByNumber(Communication *output, char function,char address,char job, void * pMem)
{
uint8_t found = false, index = 0;
uint8_t *num;
	num = (uint8_t *)pMem;
	do
	{
		found = tempSensors[index]->compareNumber(num[0]);
		if( found )
			activeSensor = tempSensors[index];
		index++;
	} while ( (!found) && (index < actNumberSensors) );
	output->sendAnswerInt(quelle_KNET,function,address,job,found,true);
}

void jobSetSensorName(Communication *output, char function,char address,char job, void * pMem)
{
char *name;
	name = (char*) pMem;
	if ( (activeSensor!=NULL) && (strlen(name)<NUMBER_OF_SENSORS_NAME_LENGTH+1) )
	{
		activeSensor->setName(name);
		output->sendPureAnswer(quelle_KNET,function,address,job,true);
	}
	else
        output->sendAnswer(fehler_text[NO_ACTIVE_SENSOR],quelle_KNET,function,address,job,false);
}

void jobSetSensorNumber(Communication *output, char function,char address,char job, void * pMem)
{
uint8_t *num;
	num = (uint8_t*) pMem;
	if ( activeSensor!=NULL )
	{
		activeSensor->setNumber(num[0]);
		output->sendPureAnswer(quelle_KNET,function,address,job,true);
	}
	else
        output->sendAnswer(fehler_text[NO_ACTIVE_SENSOR],quelle_KNET,function,address,job,false);
}

void jobGetSensorParameter(Communication *output, char function,char address,char job, void * pMem)
{
uint8_t i;
char answer[45]="";
char temp[4];
	if ( activeSensor!=NULL )
	{
		sprintf(answer,"%s,%d,0x",activeSensor->getName(),activeSensor->getNumber());
		for (i=0;i<8;i++)
		{
			sprintf(temp,"%02x",activeSensor->romId()[i]);
			strcat(answer,temp);
		}
		sprintf(temp,",%hu",activeSensor->getTemperatureBits());
		strcat(answer,temp);
		output->sendAnswer(answer,quelle_KNET,function,address,job,true);
	}
	else
        output->sendAnswer(fehler_text[NO_ACTIVE_SENSOR],quelle_KNET,function,address,job,false);
}

void jobSetCalibrationSensor(Communication *output, char function,char address,char job, void * pMem)
{
double *num;
	num = (double*) pMem;
	if ( activeSensor!=NULL )
	{
		activeSensor->setCalibrationCoefficient(num);
		output->sendPureAnswer(quelle_KNET,function,address,job,true);
	}
	else
        output->sendAnswer(fehler_text[NO_ACTIVE_SENSOR],quelle_KNET,function,address,job,false);
}

void jobGetCalibrationSensor(Communication *output, char function,char address,char job, void * pMem)
{
char answer[50]="";
double *coeff;
	if ( activeSensor!=NULL )
	{
		coeff = activeSensor->getCalibrationCoefficient();
		sprintf(answer,"T=%lf+%lf*t+%lf*t^2,%lf*t^3",coeff[0],coeff[1],coeff[2],coeff[3]);
		output->sendAnswer(answer,quelle_KNET,function,address,job,true);
	}
	else
        output->sendAnswer(fehler_text[NO_ACTIVE_SENSOR],quelle_KNET,function,address,job,false);
}

void jobGetSensorSequence(Communication *output, char function,char address,char job, void * pMem)
{
uint8_t i;
char answer[45]="";
char temp[4];
	for (i=0;i<actNumberSensors;i++)
	{
		sprintf( answer,"%d,",tempSensors[i]->getNumber() );
		strcat(answer,temp);
	}
    output->sendAnswer(answer,quelle_KNET,function,address,job,true);
}

void jobGetTemperatureActualSensor(Communication *output, char function,char address,char job, void * pMem)
{
	if ( activeSensor!=NULL )
	{
    output->sendAnswerDouble(quelle_KNET,function,address,job,(double)activeSensor->getMeanTemperature(),true);
	}
	else
    output->sendAnswer(fehler_text[NO_ACTIVE_SENSOR],quelle_KNET,function,address,job,false);
}

void jobGetCTemperatureActualSensor(Communication *output, char function,char address,char job, void * pMem)
{
char answer[20]="";

	if ( activeSensor!=NULL )
	{
    output->sendAnswerDouble(quelle_KNET,function,address,job,(double)activeSensor->getCaliMeanTemperature(),true);
	}
	else
    output->sendAnswer(fehler_text[NO_ACTIVE_SENSOR],quelle_KNET,function,address,job,false);
}

void jobSetMeasureRate(Communication *output, char function,char address,char job, void * pMem)
{
	uint16_t rate;
	rate = ((uint16_t *)pMem)[0];
	if (rate<10)
	{
        output->sendAnswer("Rate [100ms] must not smaller than 10",quelle_KNET,function,address,job,false);
	}
	else
	{
		measureRate_100ms = rate;
		output->sendPureAnswer(quelle_KNET,function,address,job,true);
	}
}

void jobGetMeasureRate(Communication *output, char function,char address,char job, void * pMem)
{
    output->sendAnswerInt(quelle_KNET,function,address,job,measureRate_100ms,true);
}

void jobSetAverageRate(Communication *output, char function,char address,char job, void * pMem)
{
	uint8_t rate;
	rate = ((uint8_t *)pMem)[0];
	if ( activeSensor!=NULL )
	{
		if (activeSensor->setHistoryMax(rate))
		{
			output->sendPureAnswer(quelle_KNET,function,address,job,true);
		}
		else
		{
		    output->sendAnswer(fehler_text[PARAMETER_ERROR],quelle_KNET,function,address,job,false);
		}
	}
	else
		output->sendAnswer(fehler_text[NO_ACTIVE_SENSOR],quelle_KNET,function,address,job,false);
}

