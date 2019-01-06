/*********************************************
 * vim:sw=8:ts=8:si:et
 * This is the header file the sensirion temperature and
 * humidity sensor.
 *
 * Based on ideas from the sensirion application note and modified
 * for atmega88/168.
 * A major part of the code was optimized and as a result the compiled
 * code size was reduced to 50%.
 *
 * Modifications by: Guido Socher
 *
 * Note: the sensirion SHTxx sensor does _not_ use the industry standard
 * I2C. The sensirion protocol looks similar but the start/stop
 * sequence is different. You can not use the avr TWI module.
 *
 *********************************************/
//@{
#ifndef SENSIRION_PROTOCOL_H
#define SENSIRION_PROTOCOL_H

// note: hardware connection definitions are in sensirion_protocol.c at
// the beginning of the inensirion_protocol.c file. You need to change
// the coded there if you connect the sensor to other pins.
//

//adr command r/w
#define STATUS_REG_W 0x06 //000 0011 0
#define STATUS_REG_R 0x07 //000 0011 1
#define MEASURE_TEMP 0x03 //000 0001 1
#define MEASURE_HUMI 0x05 //000 0010 1
#define RESET 0x1e        //000 1111 0

// physical connection:

#define SETSCK1 PORTE_OUTSET = PIN1_bm								// PORTC|=(1<<PC5)
#define SETSCK0	PORTE_OUTCLR = PIN1_bm  							// PORTC&=~(1<<PC5)
#define SCKOUTP PORTE_DIRSET = PIN1_bm								// DDRC|=(1<<DDC5)
//
#define SETDAT1	PORTE_OUTSET = PIN0_bm								// PORTC|=(1<<PC4)
#define SETDAT0	PORTE_OUTCLR = PIN0_bm								// PORTC&=~(1<<PC4)
#define GETDATA (PORTE_IN & PIN0_bm)								// (PINC&(1<<PINC4))
//
#define DMODEIN	PORTE_DIRCLR = PIN0_bm							// DDRC&=~(1<<DDC4)
#define PULLUP1	PORTE_PIN0CTRL = PORT_OPC_PULLUP_gc				// PORTC|=(1<<PINC4)
#define DMODEOU	PORTE_DIRSET = PIN0_bm							// DDRC|=(1<<DDC4)

//pulswith long
#define S_PULSLONG _delay_us(3.0)
#define S_PULSSHORT _delay_us(1.0)


uint8_t make_measure_float(float * fTemp,float * fHumi,float * fDew,float * fabsHumi);
extern void s_connectionreset(void);
extern char s_softreset(void);
extern char s_measure(unsigned int *p_value, unsigned char mode);
uint8_t startConversion(unsigned char mode,unsigned char *crc_state);
uint8_t readConversion( uint16_t *p_value, unsigned char *crc_state );
uint8_t writeStatusRegister(uint8_t reg,unsigned char *crc_state);
uint8_t readStatusRegister(uint8_t *reg);
extern int calc_sth11_temp(unsigned int t);
extern unsigned char rhcalc_int(unsigned int s);
extern unsigned char calc_sth11_humi(unsigned int h, int t);
void calc_sth11(float *p_humidity ,float *p_temperature);
extern int calc_dewpoint(unsigned char rh,int t);
float calc_dewpoint_float(float h,float t);
extern int log10_approx(unsigned char x);
float abs_feuchte( float h, float t);

#endif /* SENSIRION_PROTOCOL_H */
//@}
