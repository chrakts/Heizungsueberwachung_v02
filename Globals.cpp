/*
 * Globals.cpp
 *
 * Created: 19.03.2017 09:24:37
 *  Author: Christof
 */
#define EXTERNALS_H_

#include "TLog.h"

using namespace OneWire;
using namespace RomCommands;


const char *Node = "H1";
char IDNumber[12] EEMEM = "1784324-01";
char SerialNumber[12] EEMEM = "1958632254";
char IndexNumber[2] EEMEM = "A";


const char *fehler_text[]={"memory errors","parameter error","unknown job","no transmission","command not allowed","CRC error","no active sensor"};

char Compilation_Date[] = __DATE__;
char Compilation_Time[] = __TIME__;

char quelle_KNET[3]="";


volatile TIMER MyTimers[MYTIMER_NUM]= {	{TM_START,RESTART_YES,50,0,nextSensorStatus},
                                        {TM_STOP,RESTART_YES,REPORT_BETWEEN_SENSORS,0,nextReportStatus},
                                        {TM_STOP,RESTART_YES,100,0,sekundenTimer},
										{TM_STOP,RESTART_NO,100,0,NULL}		// Timeout-Timer
};

TempSensor *activeSensor=NULL;

float fTemperatur=-999,fHumidity=-999,fDewPoint=-999,fAbsHumitdity=-999;

volatile uint8_t statusSensoren = KLIMASENSOR;
volatile uint8_t statusReport = TEMPREPORT;
volatile bool    sendStatusReport = false; // dient dazu, keinen Report zu senden, bevor Sensordaten vorhanden sind
volatile uint8_t statusKlima = NOTHING_CLIMA_TODO;
volatile uint8_t statusLastSensor = NOTHING_LAST_TODO;
volatile uint8_t statusTemperature=NOTHING_TODO;

volatile uint8_t stufe1Counter=0;
volatile uint8_t stufe2Counter=0;
int errno;      // Globale Fehlerbehandlung

char SecurityLevel = 0;

uint16_t measureRate_100ms=10;
//extern uint8_t averageRate=32;

DS2484 owm;
TempSensor *tempSensors[NUMBER_OF_TEMPSENSORS];
uint8_t actNumberSensors = 0;
/* Global variables for TWI */
TWI_Master_t twiC_Master;    /*!< TWI master module. */
TWI_Master_t twiE_Master;    /*!< TWI master module. */

SearchState searchState;
MultidropRomIterator selector(owm);


/*
// SENSINFOS storedSensors[NUMBER_STORED_SENSORS] EEMEM ={
// 	{{0x28,0x9C,0xDF,0x36,0x09,0x00,0x00,0x2C},0,12,"FirstSensor",{0.0,1.0,0.0,0.0}} ,
// 	{{0x28,0x62,0xD2,0x4B,0x01,0x00,0x00,0xB1},1,12,"Secondo",{0.0,1.0,0.0,0.0}},
// 	{{0x28,0x19,0xAC,0xBF,0x03,0x00,0x00,0x2E},2,12,"LastSensor",{0.0,1.0,0.0,0.0}}
// };
*/

SENSINFOS storedSensors[NUMBER_STORED_SENSORS] EEMEM;

volatile bool nextSendReady=false;

Communication cnet(1,Node,5);


