/*
 * constants.h
 *
 * Created: 24.05.2018 19:24:50
 *  Author: chrak_2
 */


#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include "RomId/RomId.h"
using namespace OneWire;


enum{QUARZ,CLK2M,CLK32M};

#define SYSCLK QUARZ

#define PLL 2



#define LED_ROT_ON		PORTA_OUTCLR = PIN6_bm
#define LED_ROT_OFF		PORTA_OUTSET = PIN6_bm
#define LED_ROT_TOGGLE	PORTA_OUTTGL = PIN6_bm
#define LED_GRUEN_ON		PORTA_OUTCLR = PIN5_bm
#define LED_GRUEN_OFF		PORTA_OUTSET = PIN5_bm
#define LED_GRUEN_TOGGLE	PORTA_OUTTGL = PIN5_bm

#define GET_STUFE1      (PORTD_IN & PIN7_bm)
#define GET_STUFE2      (PORTD_IN & PIN6_bm)
#define GET_ALARM       (PORTD_IN & PIN5_bm)
#define GET_WATER       (PORTD_IN & PIN4_bm)

#define TRAUNSTEIN			591.0  // sealevel of Traunstein


//DS18B20-Definitionen
#define NUMBER_OF_TEMPSENSORS 10
#define NUMBER_OF_SENSORS_NAME_LENGTH 12

struct SensInfos
{
	RomId romID;
	uint8_t number;
	uint16_t temperature_bits;
    uint8_t historyMax;
	char name[NUMBER_OF_SENSORS_NAME_LENGTH];
	double caliCoefficients[4];
};

#define NUMBER_STORED_SENSORS NUMBER_OF_TEMPSENSORS

#define BROADCAST "BR"

typedef struct SensInfos SENSINFOS;

#define SENSOR_READY 0

// Reihenfolge, in der die Sensoren abgefragt werden
enum{KLIMASENSOR=0,TEMPSENSOREN ,LASTSENSOR};

// Ablauf der Abfrage des Sensirion-Sensors
enum{NOTHING_CLIMA_TODO=SENSOR_READY,
	START_TCONVERSION,WAIT_TCONVERSION,READ_TCONVERSION,
	START_HCONVERSION,WAIT_HCONVERSION,READ_HCONVERSION,
CALC_CONVERSION1,CALC_CONVERSION2,CALC_CONVERSION3};

// Ablauf der Abfrage der DS18B20-Sensoren
enum{NOTHING_TODO,START_CONVERSION,WAIT_CONVERSION,GET_TEMPERATURE};

enum{NOTHING_LAST_TODO=SENSOR_READY,WAIT_LAST,READY_LAST};


#define REPORT_BETWEEN_SENSORS 20   // Luecken zwischen Sensoren: 200ms
#define REPORT_BETWEEN_BLOCKS  6000 // alle 60 Sekunden

enum{TEMPREPORT=0,HUMIREPORT,ABSHUMIREPORT,DEWPOINTREPORT,DS18B20REPORT,STUFE1REPORT,STUFE2REPORT,ALARMREPORT,WATERREPORT,LASTREPORT};

enum{MEMORY_ERROR,PARAMETER_ERROR,UNKNOWN_ERROR,TRANSMISSION_ERROR,SECURITY_ERROR,CRC_ERROR,NO_ACTIVE_SENSOR};

enum{CRC_NIO,CRC_IO,CRC_NO,CRC_YES};



#endif /* CONSTANTS_H_ */
