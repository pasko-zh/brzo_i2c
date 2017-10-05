/*
ADT7420.ino -- An example sketch showing how to use an ADT7420 with brzo i2c

Copyright (c) 2016 Pascal Kurtansky (pascal at kurtansky dot ch).
All rights reserved.

This file is part of the library brzo_i2c.

Brzo_i2c is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Brzo_i2c is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

// Analog Devices ADT7420 Datasheet
// http://www.analog.com/en/products/analog-to-digital-converters/integrated-special-purpose-converters/integrated-temperature-sensors/adt7420.html

#include "brzo_i2c\brzo_i2c.h"

uint8_t SDA_PIN = 5;
uint8_t SCL_PIN = 4;
uint8_t ADT7420_ADR = 0x49;
uint8_t buffer[10];
uint8_t error = 0;
float temp = 0.0;

uint8_t ICACHE_RAM_ATTR get_temp_celsius(float *t) {
	uint32_t ADC_code = 0;
	uint8_t	bcode = 0;

	brzo_i2c_start_transaction(ADT7420_ADR, 400);
		buffer[0] = 0x03;
		buffer[1] = 0xA0;	

		// Sends 2 bytes from the beginning of buffer, i.e. buffer[0] and buffer[1], no repeated start
		brzo_i2c_write(buffer, 2, false);

		// Or if you would like to send 2 bytes starting from buffer[5]
		// buffer[5] = 0x03;
		// buffer[6] = 0xA0;
		// Sends buffer[5] and buffer[6], no repeated start
		// brzo_i2c_write(&buffer[5], 2, false);

		buffer[0] = 0x00;
		// Sends buffer[0], with repeated start
		brzo_i2c_write(buffer, 1, true);
		
		// Receives 2 bytes and saves them to buffer[0] and buffer[1], no repeated start
		brzo_i2c_read(buffer, 2, false);

		// Or if you would like to receive 2 bytes starting from buffer[7]
		// Receives 2 bytes and saves them to buffer[7] and buffer[8], no repeated start
		// brzo_i2c_read(&buffer[7], 2, false);

		// NOTE: The last i2c command within a transaction must not have a repeated start!
		// i.e. before calling brzo_i2c_end_transaction() you should have brzo_i2c_...(..., ..., false);
	bcode = brzo_i2c_end_transaction();
	

	// For the example with buffer[7] and buffer[8] you do
	// ADC_code = ((buffer[7] << 8) | buffer[8]);

	ADC_code = ((buffer[0] << 8) | buffer[1]);
	*t = ADC_code / 128.0;
	
	return bcode;
}

void setup() {
	delay(1000);
	Serial.begin(115200);
	brzo_i2c_setup(SDA_PIN, SCL_PIN, 2000);
}

void loop() {
	Serial.println("Waiting 5 seconds...");
	delay(5000);
	error = get_temp_celsius(&temp);
	if (error == 0) {
		Serial.print("Temp = ");
		Serial.println(temp, 8);
	}
	else {
		Serial.print("Brzo error : ");
		Serial.println(error);
	}
}
