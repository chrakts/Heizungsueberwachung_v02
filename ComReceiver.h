/*
 * ComReceiver.h
 *
 * Created: 15.04.2017 06:34:41
 *  Author: Christof
 */


#ifndef COMRECEIVER_H_
#define COMRECEIVER_H_

#include "Communication.h"

//#include "TLog.h"

enum{NOPARAMETER=0,STRING,UINT_8,UINT_16,UINT_32,FLOAT};
enum {NO_ERROR = 0,ERROR_SPEICHER,ERROR_PARAMETER,ERROR_JOB,ERROR_TRANSMISSION};
enum{ RCST_WAIT=0,RCST_L1,RCST_L2,RCST_HEADER,RCST_Z1,RCST_Z2,RCST_BR2,RCST_Q1,RCST_Q2,RCST_KEADER,RCST_WAIT_NODE,RCST_WAIT_FUNCTION,RCST_WAIT_ADDRESS,RCST_WAIT_JOB,RCST_DO_JOB,RCST_WAIT_END1,RCST_WAIT_END2,RCST_WAITCRC,RCST_CRC,RCST_GET_DATATYPE,RCST_GET_PARAMETER,RCST_NO_PARAMETER,RCST_ATTENTION};


#define MAX_TEMP_STRING 20

enum{CUSTOMER,PRODUCTION,DEVELOPMENT};

struct Command
{
	char function;
	char job;
	char security;
	uint8_t ptype;
	uint8_t pLength;
	void  (*commandFunction)  (Communication *output, char function,char address,char job, void *parameterMem);
};

typedef struct Command COMMAND;

void doJob(Communication *output);
void comStateMachine(Communication *input);
void *getMemory(uint8_t type,uint8_t num);
void free_parameter_KNET(void);

#endif /* COMRECEIVER_H_ */
