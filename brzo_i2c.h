/*
brzo_i2c.h -- A fast i2c master for the esp8266 written in assembly language

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

#ifndef _BRZO_I2C_h
#define _BRZO_I2C_h

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif
	void brzo_i2c_setup(uint8_t sda, uint8_t scl, uint32_t clock_stretch_time_out_usec);
	void ICACHE_RAM_ATTR brzo_i2c_start_transaction(uint8_t slave_address, uint16_t SCL_frequency_KHz);
	void ICACHE_RAM_ATTR brzo_i2c_write(uint8_t *data, uint8_t no_of_bytes, boolean repeated_start);
	void ICACHE_RAM_ATTR brzo_i2c_read(uint8_t *data, uint8_t nr_of_bytes, boolean repeated_start);
	uint8_t ICACHE_RAM_ATTR brzo_i2c_end_transaction();
#ifdef __cplusplus
}
#endif
#endif