/*
 * baud.h
 *
 * Created: 19.03.2017 06:27:47
 *  Author: Christof
 */ 



#ifndef BAUD_H_
#define BAUD_H_ 

	#define VALUE_TO_STRING(x) #x
	#define VALUE(x) VALUE_TO_STRING(x)
	#define VAR_NAME_VALUE(var) #var "="  VALUE(var)

	#if F_CPU == 32000000
		#if   BAUD == 9600
			#define BSEL    3317
			#define BSCALE  -4
		#elif BAUD == 19200
			#define BSEL    3301
			#define BSCALE  -5
		#elif BAUD == 28800
			#define BSEL    1095
			#define BSCALE  -4
		#elif BAUD == 38400
			#define BSEL    3269
			#define BSCALE  6
		#elif BAUD == 57600
			#define BSEL    1079
			#define BSCALE  -5
		#elif BAUD == 76800
			#define BSEL     3205
			#define BSCALE  -7
		#elif BAUD == 115200
			#define BSEL    1047
			#define BSCALE  -6
		#endif

	#elif F_CPU == 16000000

	#else
		#pragma GCC error "nicht unterstuetzte CPU-Frequenz fuer Baudrate-Berechnung"

	#endif /*  F_CPU */

	#ifndef BAUD
		#pragma GCC error "nicht unterstuetzte Baudrate fuer gewaehlte CPU-Freq."
	#endif
	#if BAUD
		#pragma message   (VAR_NAME_VALUE(BAUD))
		#pragma message   (VAR_NAME_VALUE(BSEL))
		#pragma message   (VAR_NAME_VALUE(BSCALE))
	#endif
#endif /* BAUD_H_ */