/* 
* Wire.h
*
* Created: 16.03.2017 13:37:02
* Author: a16007
*/


#ifndef __WIRE_H__
#define __WIRE_H__

#include <stdint.h>

class Wire
{
//variables
public:
protected:
private:

//functions
public:
	Wire();
	~Wire();
	
	void begin();
	void beginTransmission(uint8_t i2c_adrs);
	void write( uint8_t * data, uint8_t num_bytes );
	uint8_t requestFrom( uint8_t i2c_adrs, uint8_t num_bytes );
	int available();
	uint8_t read();
	uint8_t endTransmission();
protected:
private:
	Wire( const Wire &c );
	Wire& operator=( const Wire &c );
	
	

}; //Wire

#endif //__WIRE_H__
