
#ifndef OneWire_H
#define OneWire_H

#include <stdint.h>
#include "Masters/Masters.h"
#include "Slaves/Slaves.h"
#include "RomId/RomCommands.h"
#include "Wire.h" // #####
#include "Serial.h" // #####
#include <util/delay.h> // #####
#include "CRC_Calc.h"



void setup();

void print_rom_id(OneWire::RomId & romId);
void buffer_rom_id(char *buffer,OneWire::RomId & romId);
void init_clock(int sysclk, int pll);
void CLKSYS_AutoCalibration_Enable( uint8_t clkSource, bool extReference );
uint8_t doClima();

uint8_t doLastSensor();
uint8_t doTemperature();

#endif /* MBED_OneWire */
