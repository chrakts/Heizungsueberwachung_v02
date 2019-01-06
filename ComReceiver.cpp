/*
 * ComReceiver.cpp
 *
 * Created: 15.04.2017 06:14:06
 *  Author: Christof
 */
#include "ComReceiver.h"
#include "CommandFunctions.h"
#include <ctype.h>

uint8_t rec_state_KNET = RCST_WAIT;
uint8_t function_KNET=0;
uint8_t job_KNET=0;
uint8_t address_KNET=0;
uint8_t crc_KNET=CRC_NO;
char *parameter_text_KNET=NULL;
uint8_t parameter_text_length_KNET;
uint8_t parameter_text_pointer_KNET;

char *temp_parameter_text_KNET=NULL;
uint8_t temp_parameter_text_length_KNET;
uint8_t temp_parameter_text_pointer_KNET;

uint8_t bootloader_attention;		// nur wenn true, dann darf Bootloader gestartet werden.
uint8_t reset_attention;			// nur wenn true, dann darf Reset ausgeloest werden.

void (*bootloader)( void ) = (void (*)(void)) (BOOT_SECTION_START/2);       // Set up function pointer
void (*reset)( void ) = (void (*)(void)) 0x0000;       // Set up function pointer

#define NUM_COMMANDS 35

COMMAND commands[NUM_COMMANDS] =
	{
		{'-','-',CUSTOMER,NOPARAMETER,0,jobGotCRCError}, // Achtung, muss immer der erste sein
		{'S','K',CUSTOMER,STRING,16,jobSetSecurityKey},
		{'S','k',CUSTOMER,NOPARAMETER,0,jobGetSecurityKey},
		{'P','i',CUSTOMER,NOPARAMETER,0,jobGetIDNumber},
		{'P','s',CUSTOMER,NOPARAMETER,0,jobGetSerialNumber},
		{'P','x',CUSTOMER,NOPARAMETER,0,jobGetIndex},
		{'P','I',PRODUCTION,STRING,13,jobSetIDNumber},
		{'P','S',PRODUCTION,STRING,13,jobSetSerialNumber},
		{'P','X',PRODUCTION,STRING,3,jobSetIndexNumber},
		{'C','t',CUSTOMER,NOPARAMETER,0,jobGetCTemperatureSensor},
		{'C','h',CUSTOMER,NOPARAMETER,0,jobGetCHumiditySensor},
		{'C','d',CUSTOMER,NOPARAMETER,0,jobGetCDewPointSensor},
		{'C','a',CUSTOMER,NOPARAMETER,0,jobGetCAbsHumiditySensor},
		{'S','C',DEVELOPMENT,NOPARAMETER,0,jobGetCompilationDate},
		{'S','T',DEVELOPMENT,NOPARAMETER,0,jobGetCompilationTime},
		{'S','x',CUSTOMER,UINT_16,1,jobTestParameter},
		{'S','d',CUSTOMER,FLOAT,1,jobTestFloatParameter},
		{'S','s',CUSTOMER,STRING,20,jobTestStringParameter},
		{'S','D',CUSTOMER,UINT_16,3,jobTestTripleIntParameter},
		{'S','m',PRODUCTION,NOPARAMETER,0,jobGetFreeMemory},
		{'A','I',CUSTOMER,STRING,18,jobSetActiveSensorById},
		{'A','N',CUSTOMER,STRING,14,jobSetActiveSensorByName},
		{'A','#',CUSTOMER,UINT_8,1,jobSetActiveSensorByNumber},
		{'P','N',CUSTOMER,STRING,NUMBER_OF_SENSORS_NAME_LENGTH+2,jobSetSensorName},
		{'P','#',CUSTOMER,UINT_8,1,jobSetSensorNumber},
		{'P','p',CUSTOMER,NOPARAMETER,0,jobGetSensorParameter},
		{'P','q',CUSTOMER,NOPARAMETER,0,jobGetSensorSequence},
		{'P','C',PRODUCTION,FLOAT,4,jobSetCalibrationSensor},
		{'P','c',PRODUCTION,NOPARAMETER,0,jobGetCalibrationSensor},
		{'P','n',CUSTOMER,NOPARAMETER,0,jobGetNumberOfSensors},
		{'T','a',PRODUCTION,NOPARAMETER,0,jobGetTemperatureActualSensor},
		{'T','c',CUSTOMER,NOPARAMETER,0,jobGetCTemperatureActualSensor},
		{'S','E',PRODUCTION,NOPARAMETER,0,saveSensorsToEEProm},
		{'M','r',PRODUCTION,NOPARAMETER,16,jobGetMeasureRate},
		{'M','R',PRODUCTION,UINT_16,16,jobSetMeasureRate},
	};




void doJob(Communication *output)
{
	if (  (rec_state_KNET == RCST_DO_JOB) && (job_KNET > 0) )
	{
		if (SecurityLevel < commands[job_KNET-1].security)
		{
			 output->sendAnswer(fehler_text[SECURITY_ERROR],quelle_KNET,commands[job_KNET-1].function,address_KNET,commands[job_KNET-1].job,false);
		}
		else
		{
			if ( job_KNET<=NUM_COMMANDS )
			{
				commands[job_KNET-1].commandFunction(output,commands[job_KNET-1].function,address_KNET,commands[job_KNET-1].job, parameter_text_KNET);
			}
		}
		free_parameter_KNET();
		LED_GRUEN_OFF;
		rec_state_KNET = RCST_WAIT;
		function_KNET = 0;
		job_KNET = 0;

	}
}

void comStateMachine(Communication *input)
{
	uint8_t ready,i;
	uint8_t error = NO_ERROR;
	char act_char,temp;
	static char crcString[5];
	static uint8_t crcIndex;
	uint8_t length;

	char infoType;
	if( input->getChar(act_char) == true )
	{
		if( 1==0 )
		{
			rec_state_KNET = RCST_L1;
		}
		else
		{
			switch( rec_state_KNET )
			{
				case RCST_WAIT:
					LED_GRUEN_OFF;
					if( act_char=='#' )
					{
//						LED_GRUEN_ON;
						crcIndex = 0;
						crcGlobal.Reset();
						crcGlobal.Data(act_char);
						rec_state_KNET = RCST_L1;
					}
				break;
				case RCST_L1:
					if( isxdigit(act_char)!=false )
					{
						if( act_char<58)
							length = 16*(act_char-48);
						else
						{
							act_char = tolower(act_char);
							length = 16*(act_char-87);
						}
						crcGlobal.Data(act_char);
						rec_state_KNET = RCST_L2;
					}
					else
						rec_state_KNET = RCST_WAIT;
				break;
				case RCST_L2:
					if( isxdigit(act_char)!=false )
					{
						if( act_char<58)
							length += (act_char-48);
						else
						{
							act_char = tolower(act_char);
							length += (act_char-87);
						}
						crcGlobal.Data(act_char);
						rec_state_KNET = RCST_HEADER;
					}
					else
						rec_state_KNET = RCST_WAIT;
				break;
				case RCST_HEADER:
					if ( (act_char&4)==4 )
					{
						crc_KNET=CRC_YES;
						crcGlobal.Data(act_char);
					}
					else
					{
						crc_KNET=CRC_NO;
					}
					rec_state_KNET = RCST_Z1;
				break;
				case RCST_Z1:
					if(crc_KNET==CRC_YES)
						crcGlobal.Data(act_char);
					if( act_char==Node[0] )
                        rec_state_KNET = RCST_Z2;
                    else
                    {
                        if( act_char=='B' )
                            rec_state_KNET = RCST_BR2;
                        else
                            rec_state_KNET= RCST_WAIT;
                    }

/*					switch ( act_char )
					{
						case Node[0]:
							rec_state_KNET = RCST_Z2;
						break;
						case 'B':
							rec_state_KNET = RCST_BR2;
						break;
						default:
							rec_state_KNET= RCST_WAIT;
					}*/
				break;
				case RCST_Z2:
					if( act_char==Node[1] )
                    {
						if(crc_KNET==CRC_YES)
							crcGlobal.Data(act_char);
						rec_state_KNET = RCST_Q1;
//						LED_ROT_ON;
                    }
                    else
                    {
                        rec_state_KNET= RCST_WAIT;
                    }
/*

					if ( act_char==Node[1] )
					{
						if(crc_KNET==CRC_YES)
							crcGlobal.Data(act_char);
						rec_state_KNET = RCST_Q1;
						LED_ROT_ON;
					}
					else
					{
						rec_state_KNET= RCST_WAIT;
					}*/
				break;
				case RCST_BR2:
				if ( act_char=='R' )
				{
					if(crc_KNET==CRC_YES)
						crcGlobal.Data(act_char);
					rec_state_KNET = RCST_Q1;
				}
				else
				{
					rec_state_KNET= RCST_WAIT;
				}
				break;
				case RCST_Q1:
					if(crc_KNET==CRC_YES)
						crcGlobal.Data(act_char);
					quelle_KNET[0]=act_char;
					rec_state_KNET = RCST_Q2;
				break;
				case RCST_Q2:
					if(crc_KNET==CRC_YES)
						crcGlobal.Data(act_char);
					quelle_KNET[1]=act_char;
					quelle_KNET[2]=0;
					rec_state_KNET = RCST_KEADER;
				break;
				case RCST_KEADER:
					infoType=act_char;
					if (infoType=='S')
					{
						if(crc_KNET==CRC_YES)
							crcGlobal.Data(act_char);
						rec_state_KNET=RCST_WAIT_FUNCTION;
					}
					else
					{
						rec_state_KNET=RCST_WAIT;
					}
				break;
				case RCST_WAIT_FUNCTION:
					rec_state_KNET = RCST_WAIT_ADDRESS;
					ready = false;
					temp = 0;
					i = 0;
					do
					{
						if(commands[i].function == act_char)
						{
							temp = act_char;
							ready = true;
							if(crc_KNET==CRC_YES)
								crcGlobal.Data(act_char);
						}
						i++;
						if(i==NUM_COMMANDS)
							ready = true;
					}while (!ready);
					function_KNET = temp;
				break;
				case RCST_WAIT_ADDRESS:
					rec_state_KNET = RCST_WAIT_JOB;
					address_KNET = act_char;
                    if(crc_KNET==CRC_YES)
                        crcGlobal.Data(act_char);
				break;
				case RCST_WAIT_JOB:
					rec_state_KNET = RCST_NO_PARAMETER;
					ready = false;
					temp = 0;
					i = 0;
					do
					{
						if(commands[i].function == function_KNET)
						{
							if(commands[i].job == act_char)
							{
								temp = i+1;  // Achtung: job_KNET ist immer eins größer als der Index
								ready = true;
								if(crc_KNET==CRC_YES)
									crcGlobal.Data(act_char);
							}
						}
						i++;
						if(i==NUM_COMMANDS)
							ready = true;
					}while (!ready);
					job_KNET = temp;
					if (job_KNET==0)
					{
						bootloader_attention = false;
						function_KNET = 0;
						job_KNET = 0;
						rec_state_KNET = RCST_WAIT;
					}
					else
					{
						if( commands[job_KNET-1].ptype != NOPARAMETER )
						{
							parameter_text_KNET = (char*) getMemory(commands[job_KNET-1].ptype,commands[job_KNET-1].pLength);
//							input->pformat("habe Speicher belegt: %d Bytes\n\r",commands[job_KNET-1].pLength);
							if (parameter_text_KNET==NULL)
							{
								input->sendInfo("!!!!!Error!!!!!!","BR");
							}
							parameter_text_length_KNET = commands[job_KNET-1].pLength;
							if( commands[job_KNET-1].ptype != STRING )
								temp_parameter_text_KNET = (char *) getMemory(STRING,MAX_TEMP_STRING);
							rec_state_KNET = RCST_GET_DATATYPE;
						}
						else
							rec_state_KNET = RCST_NO_PARAMETER;
						parameter_text_pointer_KNET = 0;
						temp_parameter_text_pointer_KNET = 0;
					}
				break;
				case RCST_NO_PARAMETER: // dann muss der Datentyp = '?' sein
					if( act_char=='?' )
					{
						if(crc_KNET==CRC_YES)
						{
							crcGlobal.Data(act_char);
							rec_state_KNET = RCST_CRC;
						}
						else
							rec_state_KNET = RCST_WAIT_END1;
					}
					else
						rec_state_KNET = RCST_WAIT;
				break;
				case RCST_GET_DATATYPE: // einziger bekannter Datentyp : 'T'
					if( act_char=='T' )
					{
						if(crc_KNET==CRC_YES)
							crcGlobal.Data(act_char);
						rec_state_KNET = RCST_GET_PARAMETER;
					}
					else
						rec_state_KNET = RCST_WAIT;
				break;

				case RCST_CRC:
					if ( isxdigit(act_char) )
					{
						crcString[crcIndex] =  act_char;
						crcIndex++;
						if (crcIndex>=4)
						{
							crc_KNET=CRC_IO;
							if(crcGlobal.compare(crcString) != true )
							{
								job_KNET = 1;	// das ist der CRC-Error-Job
							}
							rec_state_KNET = RCST_WAIT_END1;
						}
					}
					else
					{
						rec_state_KNET = RCST_WAIT;
						job_KNET = 0;
						free_parameter_KNET();
					}
				break;

				case RCST_WAIT_END1:
					if( act_char=='\r' )
						rec_state_KNET = RCST_WAIT_END2;
					else
					{
						rec_state_KNET = RCST_WAIT;
						job_KNET = 0;
						free_parameter_KNET();
					}
				break;
				case RCST_WAIT_END2:
					if( act_char=='\n' )
						rec_state_KNET = RCST_DO_JOB;
					else
					{
						rec_state_KNET = RCST_WAIT;
						job_KNET = 0;
						free_parameter_KNET();
					}
				break;
				case RCST_GET_PARAMETER:
					if(crc_KNET==CRC_YES)
						crcGlobal.Data(act_char);
					if ( commands[job_KNET-1].ptype==STRING )
					{
						if( (act_char=='<') )					// Parameterende
						{
							if(crc_KNET==CRC_YES)
								rec_state_KNET = RCST_CRC;
							else
								rec_state_KNET = RCST_WAIT_END1;

							parameter_text_KNET[parameter_text_pointer_KNET] = 0;
//							input->println("-----------------");
//							input->println(parameter_text_KNET);
						}
						else
						{
							if( parameter_text_pointer_KNET < parameter_text_length_KNET-2 )
							{
								parameter_text_KNET[parameter_text_pointer_KNET] = act_char;
								parameter_text_pointer_KNET++;
							}
							else // zu langer Parameter
							{
								rec_state_KNET = RCST_WAIT;
								error = ERROR_PARAMETER;
								function_KNET = 0;
								free_parameter_KNET();
							}

						}
					} // if STRING
					else // if some Number-Parameter
					{
						if ((act_char=='<') || (act_char==','))
						{
							errno = 0;
							temp_parameter_text_KNET[temp_parameter_text_pointer_KNET] = 0;		// Zahlenstring abschießen
							if ( parameter_text_pointer_KNET < commands[job_KNET-1].pLength )   // wird noch ein Parameter erwartet?
							{
								uint32_t wert;
								switch(commands[job_KNET-1].ptype)
								{
									case UINT_8:
										uint8_t *pointer_u8;
										pointer_u8 =  (uint8_t*) parameter_text_KNET;
										wert = strtoul(temp_parameter_text_KNET,NULL,0);
										if(wert<256)
											pointer_u8[parameter_text_pointer_KNET] = (uint8_t) wert;
										else
											error = ERROR_PARAMETER;
									break;
									case UINT_16:
										uint16_t *pointer_u16;
										pointer_u16 =  (uint16_t*) parameter_text_KNET;
										wert = strtoul(temp_parameter_text_KNET,NULL,0);
										if(wert<65536)
											pointer_u16[parameter_text_pointer_KNET] = (uint16_t) wert;
										else
											error = ERROR_PARAMETER;
//										input->println(temp_parameter_text_KNET);	!!!!!!!!!!!!!!!!!auskommentiert!!!!!!!!!!!!!!!!!!!!
//										input->pformat("Wert: %\>d\n",wert);		!!!!!!!!!!!!!!!!!auskommentiert!!!!!!!!!!!!!!!!!!!!
									break;
									case UINT_32:
										uint32_t *pointer_u32;
										pointer_u32 =  (uint32_t*) parameter_text_KNET;
										pointer_u32[parameter_text_pointer_KNET] = strtoul(temp_parameter_text_KNET,NULL,0);
									break;
									case FLOAT:
										double *pointer_d;
										pointer_d =  (double*) parameter_text_KNET;
										pointer_d[parameter_text_pointer_KNET] = strtod(temp_parameter_text_KNET,NULL);
									break;
								}
							}
							else
								error = ERROR_PARAMETER;
							if( parameter_text_pointer_KNET < parameter_text_length_KNET-1 ) // Zeiger auf nächsten Parameter
							{
								parameter_text_pointer_KNET++;
								temp_parameter_text_pointer_KNET = 0;						// zurücksetzen für nächsten Parameter
							}
							else
                            {
                                    ;
                            } // hier noch abfangen falls zu viele Parameter eingeben wurden
							if ( errno != 0)
								error = ERROR_PARAMETER;
							if ((act_char=='<'))
							{
								if(crc_KNET==CRC_YES)
									rec_state_KNET = RCST_CRC;
								else
									rec_state_KNET = RCST_WAIT_END1;
							}
							/* hier noch abfangen, falls zu wenige Parameter eingegeben wurden ************************ */
						}
						else // weiterer Character eines Parameters
						{
							if( temp_parameter_text_pointer_KNET < MAX_TEMP_STRING-2 )
							{
								temp_parameter_text_KNET[temp_parameter_text_pointer_KNET] = act_char;
								temp_parameter_text_pointer_KNET++;
							}
							else // zu langer Parameter
								error = ERROR_JOB;
						}
						if ( error != NO_ERROR )
						{
								function_KNET = 0;
								rec_state_KNET = RCST_WAIT;
								free_parameter_KNET();
						}
					}
				break; // case RCST_GET_PARAMETER

			} // end of switch
//			input->pformat("State: %x, char:%x, job:%d\r\n",rec_state_KNET,act_char,job_KNET);
		}
	}
}

void *getMemory(uint8_t type,uint8_t num)
{
uint8_t size=1;
void *mem=NULL;
	switch(type)
	{
		case STRING:
			size = 1;
		break;
		case UINT_8:
			size = 1;
		break;
		case UINT_16:
			size = 2;
		break;
		case UINT_32:
			size = 4;
		break;
		case FLOAT:
			size = sizeof(double);
		break;
		default:
			size = -1;

	}
	if (size>0)
	{
		mem =  malloc(size*num);

	}
	return( mem );
}

void free_parameter_KNET(void)
{
	if (parameter_text_KNET)
	{
		free( parameter_text_KNET );
		parameter_text_KNET = NULL;
		parameter_text_length_KNET = 0;
	}
	if (temp_parameter_text_KNET != NULL)
	{
		free( temp_parameter_text_KNET );
		temp_parameter_text_KNET = NULL;
		temp_parameter_text_pointer_KNET = 0;
	}
}

