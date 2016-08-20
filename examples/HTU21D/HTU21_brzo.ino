/*
HTU21_brzo.ino -- An example sketch showing how to use an HTU21 with brzo i2c

Copyright (c) 2016 Pascal Kurtansky (pascal at kurtansky dot ch).
All rights reserved.

This file is part of the library brzo_i2c.

HTU21_brzo is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

HTU21_brzo is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/


#include "brzo_i2c\brzo_i2c.h"

uint8_t SDA_PIN = 5;
uint8_t SCL_PIN = 4;
uint8_t SCL_speed = 200;

uint8_t HTU21_ADR = 0x40;
// Temperature measuring time is max 50 msec @ 14 bits
uint32_t SCL_STRETCH_TIMEOUT = 50000;
uint8_t MAX_RETRIES = 3;
uint8_t buffer[5];
uint8_t error = 0;
float temp = 0.0;

uint8_t ICACHE_RAM_ATTR soft_reset() {
	uint8_t	bcode = 0;
	// Soft Reset command
	buffer[0] = 0xFE;
	brzo_i2c_start_transaction(HTU21_ADR, SCL_speed);
		brzo_i2c_write(buffer, 1, false);
	bcode = brzo_i2c_end_transaction();
	return bcode;
}

uint8_t ICACHE_RAM_ATTR get_temp_hold_master(float *t) {
	uint8_t	bcode = 0;
	uint16_t Stemp = 0;
	unsigned long a, b;
	// Trigger Temperature Measurement command
	buffer[0] = 0xE3;
	brzo_i2c_start_transaction(HTU21_ADR, SCL_speed);
		brzo_i2c_write(buffer, 1, true);
		a = micros();
		brzo_i2c_read(buffer, 3, false);
		b = micros();
	bcode = brzo_i2c_end_transaction();
	
	Stemp = (buffer[0] << 8) | buffer[1];
	*t = -46.85 + (175.72*Stemp)/65536.0;
	Serial.print("SCL Stretch duration in usec: ");
	Serial.println(b - a);
	return bcode;
}

uint8_t ICACHE_RAM_ATTR get_temp_no_hold_master(float *t) {
	uint8_t	bcode = 0;
	uint16_t Stemp = 0;
	// Trigger Temperature Measurement command
	buffer[0] = 0xF3;
	brzo_i2c_start_transaction(HTU21_ADR, SCL_speed);
		brzo_i2c_write(buffer, 1, false);
	bcode = brzo_i2c_end_transaction();
	// Temperature measuring time is max 50 msec @ 14 bits 
	delay(50);
	if (bcode == 0) {
		brzo_i2c_start_transaction(HTU21_ADR, SCL_speed);
			brzo_i2c_read(buffer, 3, false);
		bcode = brzo_i2c_end_transaction();
	}
	Stemp = (buffer[0] << 8) | buffer[1];
	*t = -46.85 + (175.72*Stemp) / 65536.0;
	return bcode;
}

void setup() {
	delay(1000);
	Serial.begin(115200);

	brzo_i2c_setup(SDA_PIN, SCL_PIN, SCL_STRETCH_TIMEOUT);
	// HTU21 needs at most 15ms while SCK is high for reaching idle state
	delay(15);

	uint8_t i = 1;
	do {
		error = soft_reset();
		if (error == 0) {
			Serial.println("Soft reset OK ");
		}
		else {
			Serial.print("Soft reset fail. Brzo error : ");
			Serial.println(error);
		}
		i++;
	} while ((error != 0) && (i <= MAX_RETRIES));
}

void loop() {
	Serial.println("Waiting 5 seconds...");
	delay(5000);

	// Without SCL stretching (called "no hold master" in the HTU21 datasheet)
	error = get_temp_no_hold_master(&temp);
	
	// With SCL stretching (called "hold master" in the HTU21 datasheet)
	// error = get_temp_hold_master(&temp);
	// ! Be aware that you will be without any interrupts during 45--50 msec, i.e. during the brzo transaction!
	// ! Remember: brzo_i2c_start_transaction(.) disables all interrupts!

	if (error == 0) {
		Serial.print("Temp = ");
		Serial.println(temp, 8);
	}
	else {
		Serial.print("Brzo error : ");
		Serial.println(error);
	}
}