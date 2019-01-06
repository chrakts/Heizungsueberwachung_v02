/*
 * MyTimer.h
 *
 * Created: 11.02.2016 20:20:03
 *  Author: Christof
 */


#ifndef MYTIMER_H_
#define MYTIMER_H_

#define MYTIMER_NUM	4

enum{TM_START,TM_RESET,TM_STOP,TM_RUN};
enum{RESTART_YES,RESTART_NO};

enum{TIMER_SENSOREN,TIMER_REPORT,TIMER_SEKUNDE,TIMER_TIMEOUT};

struct Timer
{
	uint8_t		state;
	uint8_t		restart;
	uint16_t	value;
	uint16_t	actual;
	void  (*OnReady)  (uint8_t test);
};

typedef struct Timer TIMER;

void nextSensorStatus(uint8_t test);
void nextReportStatus(uint8_t test);
void sekundenTimer(uint8_t test);
void LED_toggle(uint8_t test);
void init_mytimer(void);
void goto_sleep(uint8_t test);


#endif /* MYTIMER_H_ */
