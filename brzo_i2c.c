/*
brzo_i2c.c -- A fast i2c master for the esp8266 written in assembly language

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

#ifndef ARDUINO
#include <eagle_soc.h>
#include <ets_sys.h>
#include <os_type.h>
#include <osapi.h>
#include <gpio.h>
#include <user_interface.h>

extern void ets_isr_mask(unsigned intr); // missing definition
extern void ets_isr_unmask(unsigned intr); // missing definition

#ifndef ICACHE_RAM_ATTR
#define ICACHE_RAM_ATTR
#endif
#endif

#include "brzo_i2c.h"

// Global variables
uint16_t sda_bitmask, scl_bitmask, iteration_scl_halfcycle;
uint16_t iteration_remove_spike;
uint32_t iteration_scl_clock_stretch;
uint16_t ACK_polling_loop_usec;
uint8_t i2c_slave_address;
uint16_t i2c_SCL_frequency = 0;
uint8_t i2c_error = 0;


void ICACHE_RAM_ATTR brzo_i2c_write(uint8_t *data, uint32_t no_of_bytes, bool repeated_start)
{
	// Pointer to Data Buffer, Number of Bytes to Send from Data Buffer
	// Returns 0 or Error encoded as follows
	// Bit 0 (1) : Bus not free, i.e. either SDA or SCL is low
	// Bit 1 (2) : Not ACK ("NACK") by slave during write: Either the slave did not respond to the given slave address; 
	//            or it did not ACK a byte transferred by the master.
	// Bit 2 (4) : -- 
	// Bit 3 (8) : Clock Stretching by slave exceeded maximum clock stretching time
	// Bit 4 (16): --
	// Bit 5 (32): --


	// Do not perform an i2c write if a previous i2c command has already failed
	if (i2c_error > 0) return;
	uint8_t byte_to_send = i2c_slave_address << 1;
	// Assembler Variables
	uint32_t a_set = 0, a_repeated = 0, a_in_value = 0, a_temp1 = 0, a_bit_index = 0;
	if (repeated_start == true) a_repeated = 1;
	else a_repeated = 0;
	asm volatile (
		// If BRZO_I2C_DISABLE_INTERRUPTS is set to 1, then all interrupts are disabled, 
		//   i.e. interrupts up to the highest interrupt level of 15
		//   the current level is saved in %[r_temp1] but we will not use that value again, 
		//   instead we will just enable all interrupt levels at the end of this routine
		#if BRZO_I2C_DISABLE_INTERRUPTS != 0
		"RSIL   %[r_temp1], 15;"
		#endif
		"MOVI   %[r_set], 0x60000304;"

		// Check if bus is free and send START
		"OR     %[r_temp1], %[r_sda_bitmask], %[r_scl_bitmask];"
		"L16UI  %[r_in_value], %[r_set], 20;" // offset is 20d = 14h = > in: 0x60000318
		"MEMW;"
		"MOVI.N %[r_error], 1;"
		// If either SDA or SCL is low, then bus is not free and thus jump to l_exit
		"BNALL  %[r_in_value], %[r_temp1], l_exit;"
		// Bus is free, so we can send START
		"MOVI.N %[r_error], 0;"
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		// Set SCL = 1
		"S16I   %[r_scl_bitmask], %[r_set], 0;"
		"MEMW;"
		// Set SDA = 0
		// Delay for tHD;STA  >= 4.0 usec for standard mode, 0.6 usec for fast or 0.26 usec fast mode plus
		//  => a delay of one half cycle is enough to meet those timings
		"S16I   %[r_sda_bitmask], %[r_set], 4;"  // clear: 0x60000308
		"l_w01:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_w01;"
		// Post Condition: SDA = 0, SCL = 1

		// The outer loop, sending 1...n data bytes
		"l_send_byte:"
		// select the MSB of byte_to_send
		"MOVI   %[r_bit_index], 128;"
		// The inner loop, sending 1...8 bits
		"l_send_bit:"
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		// check if the bit of byte_to_send at bit_index is 0 or 1
		"BALL   %[r_byte_to_send], %[r_bit_index], l_sda1_scl0;"
		// SDA = 0, SCL = 0
		"S16I   %[r_scl_bitmask], %[r_set], 4;" // clear: 0x60000308
		"MEMW;"
		"S16I   %[r_sda_bitmask], %[r_set], 4;" // clear: 0x60000308
		"l_w02:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_w02;"
		"j l_sdax_scl1;"

		"l_sda1_scl0:"
		// SDA = 1, SCL = 0
		"S16I   %[r_scl_bitmask], %[r_set], 4;" // clear: 0x60000308
		"MEMW;"
		"S16I   %[r_sda_bitmask], %[r_set], 0;"
		"l_w03:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_w03;"

		"l_sdax_scl1:"
		// SDA = leave unchanged and set SCL = 1
		// Check for clock stretching
		// Delay is little bit shorter, i.e. half_cycle - delta
		"ADDI   %[r_temp1], %[r_iteration_scl_halfcycle], -5;"
		// Let SCL raise
		"S16I   %[r_scl_bitmask], %[r_set], 0;"
		"MEMW;"
		"l_w04:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		// Explicitly BGEZ instead of BNEZ
		"BGEZ   %[r_temp1], l_w04;"

		// Sample SCL value
		"L16UI  %[r_in_value], %[r_set], 20;" // offset is 20d = 14h = > in: 0x60000318
		"MEMW;"
		// r_temp1 holds the number of iterations for clock stretch timeout 
		"MOV.N  %[r_temp1], %[r_iteration_scl_clock_stretch];"
		// Branch if SCL = 1, i.e. no stretching
		"BALL   %[r_in_value], %[r_scl_bitmask], l_no_stretch;"
		// SCL = 0, i.e. stretching by the slave, i.e. it pulls SCL low
		"l_stretch:"
		// Sample SCL value
		"L16UI  %[r_in_value], %[r_set], 20;" // offset is 20d = 14h = > in: 0x60000318
		"MEMW;"
		// Branch if SCL = 1, i.e. no more stretching
		"BALL   %[r_in_value], %[r_scl_bitmask], l_scl_high_by_slave;"
		// SCL is still low
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		// Did we reach the clock stretch timeout?
		// Branch if we have not yet reached the timeout
		"BNEZ   %[r_temp1], l_stretch;"
		// We have reached the clock stretch timeout, i.e. SCL is still pulled low by the slave
		// Before the clock stretching, the master has set SCL to high but left SDA unchanged. 
		// Thus, if the LSB was zero, SDA is still pulled low by the master. 
		// We therefore have to release SDA
		// (if SDA was already high, setting it to high again won't do any harm)
		"S16I   %[r_sda_bitmask], %[r_set], 0;"
		"MEMW;"
		// Error: Bus is not free, since SCL is still low AND clock stretch timeout reached
		"MOVI.N %[r_error], 8;"
		// We explicitly do not send a STOP instead we exit, i.e. jump to l_exit and not to l_send_stop
		"j l_exit;"

		"l_scl_high_by_slave:"
		// SCL was set high by the slave
		// We have to make sure that SCL = 1 for a complete half cycle
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		"l_w041:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_w041;"

		"l_no_stretch:"
		// Postcondition: SCL = 1 for a half cycle
		// Are there bits left that we need to send?
		"SRLI   %[r_bit_index], %[r_bit_index], 1;"
		// When the LSB of the byte_to_send was sent, i.e. bit index was 1 before SRLI, it will now be zero
		// As long as the LSB was not sent keep on sending bits, i.e. jump
		"BNEZ   %[r_bit_index], l_send_bit;"
		// we have sent 8 Bits

		// check for ACK by slave
		// Precondition
		// SDA = LSB (i.e. SDA = 0, since we have an i2c write), SCL = 1
		// SCL = 0
		// Spike reducing waits here
		"S16I   %[r_scl_bitmask], %[r_set], 4;"  // clear : 0x60000308
		"MOV.N  %[r_temp1], %[r_iteration_minimize_spike];"
		"l_w05:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_w05;"
		// Reduce number of iterations by the ones we've already used
		"SUB    %[r_temp1], %[r_iteration_scl_halfcycle], %[r_iteration_minimize_spike];"
		// Now we let SDA raise. 
		// In case of an ACK the i2c slave is pulling SDA down
		// In case of an NACK, SDA raises
		"S16I   %[r_sda_bitmask], %[r_set], 0;"
		"MEMW;"
		"l_w06:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BGEZ   %[r_temp1], l_w06;"

		// Delay is little bit shorter, i.e. half_cycle - delta
		// Because we will have a L16UI after in this half cycle
		"ADDI   %[r_temp1], %[r_iteration_scl_halfcycle], -5;"
		// Set SCL = 1, i.e. start of the second half cycle of the 9th SCL cycle
		"S16I   %[r_scl_bitmask], %[r_set], 0;"
		"MEMW;"
		// Delay for the second half cycle of the 9th SCL cycle
		"l_w07:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BGEZ   %[r_temp1], l_w07;"
		// Sample SDA at the end of the 9th clock cycle
		// In the case of an NACK we want to leave enough time that SDA can raise 
		// If sda_value AND sda_bitmask == 0 => ACK else we have an NACK
		"L16UI  %[r_in_value], %[r_set], 20;" // offset is 20d = 14h = > in: 0x60000318
		"BNALL  %[r_in_value], %[r_sda_bitmask], l_slave_ack;"
		"MOVI.N %[r_error], 2;"
		// NACK by slave
		// Postcondition:
		//   SDA = 1 (NACK) and SCL = 1 
		//   9th Clock Cycle is finished
		"j l_send_stop;"

		"l_slave_ack:"
		// ACK
		// Precondition: SDA = 0 (still pulled low by the slave) and SCL = 1
		// The slave will pull SDA low as long as SCL = 1
		// We have to set SDA = 0 by the master
		// clear : 0x60000308
		"S16I   %[r_sda_bitmask], %[r_set], 4;" 
		// Postcondition:
		//   SDA = 0 and SCL = 1
		//   9th Clock Cycle is finished
		
		"BEQZ   %[r_no_of_bytes], l_send_stop;"
		// Branch if there are no more Data Bytes to send
		// Load the corresponding element of array data[.] into byte_to_send
		"L8UI   %[r_byte_to_send], %[r_adr_array_element], 0;"
		// Move the pointer to the next array element (since we have an array of bytes, the increment is 1)
		"ADDI.N %[r_adr_array_element], %[r_adr_array_element], 1;"
		// Decrement the number of bytes to send
		"ADDI.N %[r_no_of_bytes], %[r_no_of_bytes], -1;"
		"j l_send_byte;"

		"l_send_stop:"
		// Send Stop 
		// We have to make sure that SDA = 0 and SCL = 1, before we send the STOP sequence,
		//   i.e. "A LOW to HIGH transition on the SDA line while SCL is HIGH"
		// In order to achieve this econdition, we have to distinguish between 
		//   1) NACK: SDA = 1, SCL = 1
		//   2) ACK: SDA = 0, SCL = 1
		//      SDA is still pulled low by the slave, so we have to signal the slave to release it.
		//      We will do this by letting SCL go low. 
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		// if we had a NACK then r_error = 2
		// if we had an ACK then r_error = 0
		"BNEZ.N %[r_error], l_stop_after_NACK;"
		// Send stop after ACK
		// Precondition: SDA = 0, SCL = 1

		// We are at the beginning of the 10th cycle (if there was no clock stretching)
		// Set SCL = 0
		// During the first half cycle the slave should release SDA...
		"S16I   %[r_scl_bitmask], %[r_set], 4;" // clear : 0x60000308
		"MEMW;"
		"l_w08:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_w08;"

		// Check for a repeated start
		// Branch if r_repeated is 0, i.e. is no repeated start, just send stop
		"BEQZ.N %[r_repeated], l_no_repeated_start;"
		// Make sure that the precondition for the next command (i.e. the start) will be met
		// Currently, SCL = 0 and SDA is starting to raise, since the slave has released it
		// To be on the safe side, we set both SCL = 1 _and SDA = 1 
		// SDA  = 1
		"S16I   %[r_sda_bitmask], %[r_set], 0;"
		"MEMW;"
		// SCL = 1;
		"S16I   %[r_scl_bitmask], %[r_set], 0;"
		// Postcondition: SCL = 1 and SDA = 1, now the next i2c command send start
		"j l_exit;"

		"l_no_repeated_start:"
		// For the second half cycle, we set SDA = 0, SCL = 1
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		"S16I   %[r_sda_bitmask], %[r_set], 4;" // clear : 0x60000308
		"MEMW;"
		"S16I   %[r_scl_bitmask], %[r_set], 0;"
		"l_w09:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_w09;"

		// For the first half cycle of the 11th cycle, we set SDA = 1 and leave SCL = 1
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		// SDA = 1 (SCL is already high, we don't need to change it)
		"S16I   %[r_sda_bitmask], %[r_set], 0;"
		"MEMW;"
		"l_w10:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_w10;"
		"j l_exit;"

		"l_stop_after_NACK:"
		// Send stop after NACK
		// Precondition: SDA = 1, SCL = 1

		// SDA = 0
		// SCL = 1 : In "normal" cycles we woud set SCL to 0
		"S16I   %[r_sda_bitmask], %[r_set], 4;"  // clear: 0x60000308																		 
		"S16I   %[r_scl_bitmask], %[r_set], 0;"
		// Delay for the first half cycle of 10th cycle
		"MEMW;"
		"l_w11:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_w11;"
		// Postcondition: SDA = 0 and SCL = 1

		// Now we set SDA = 1 and leave SCL = 1 : This ist the STOP condition, 
		//   i.e. the "A LOW to HIGH transition on the SDA line while SCL is HIGH"
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		"S16I   %[r_sda_bitmask], %[r_set], 0;"
		// SDA = 1 (SCL is already high, we don't need to change it)
		"MEMW;"
		"l_w12:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_w12;"

		"l_exit:"
		// If BRZO_I2C_DISABLE_INTERRUPTS is set to 1,  
		//   enable all interrupts again, i.e. interrupts with interrupt level >= 1
		#if BRZO_I2C_DISABLE_INTERRUPTS != 0
		"RSIL   %[r_temp1], 0;"
		#endif

		: [r_set] "+r" (a_set), [r_repeated] "+r" (a_repeated), [r_temp1] "+r" (a_temp1), [r_in_value] "+r" (a_in_value), [r_error] "+r" (i2c_error), [r_bit_index] "+r" (a_bit_index), [r_adr_array_element] "+r" (&data[0]), [r_byte_to_send] "+r" (byte_to_send), [r_no_of_bytes] "+r" (no_of_bytes)
		: [r_sda_bitmask] "r" (sda_bitmask), [r_scl_bitmask] "r" (scl_bitmask), [r_iteration_scl_halfcycle] "r" (iteration_scl_halfcycle), [r_iteration_minimize_spike] "r" (iteration_remove_spike), [r_iteration_scl_clock_stretch] "r" (iteration_scl_clock_stretch)
		: "memory"
	);
	return;
}

void ICACHE_RAM_ATTR brzo_i2c_read(uint8_t *data, uint32_t nr_of_bytes, bool repeated_start)
{
	// Pointer to Data Buffer, Number of Bytes to Read from Data Buffer
	// Set i2c_error as follows
	// Bit 0 (1) : Bus not free, i.e. either SDA or SCL is low
	// Bit 1 (2) : --
	// Bit 2 (4) : Not ACK ("NACK") by slave during read, i.e. slave did not respond to the given slave address
	// Bit 3 (8) : Clock Stretching by slave exceeded maximum clock stretching time
	// Bit 4 (16): Function called with 0 bytes to be read by the master. 
	//             Command not sent to the slave, since this could yield to a bus stall (SDA remains 0)
	// Bit 5 (32): --

	// Set i2c_error and return if "empty" i2c read
	if (nr_of_bytes == 0) {
		i2c_error = 16;
		return;
	}
	// Do not perform an i2c read if a previous i2c command has already failed
	if (i2c_error > 0) return;
	// Assembler Variables
	uint32_t a_set = 0, a_repeated = 0, a_in_value = 0, a_temp1 = 0, a_temp2 = 0, a_bit_index = 0;
	a_temp2 = 0;
	if (repeated_start == true) a_repeated = 1;
	else a_repeated = 0;
	// a_temp2 holds 7 Bit slave address, with the LSB = 1 for i2c read
	a_temp2 = (i2c_slave_address << 1) | 1;

	asm volatile (
		// If BRZO_I2C_DISABLE_INTERRUPTS is set to 1, then all interrupts are disabled, 
		//   i.e. interrupts up to the highest interrupt level of 15
		#if BRZO_I2C_DISABLE_INTERRUPTS != 0
		"RSIL   %[r_temp1], 15;"
		#endif
		"MOVI   %[r_set], 0x60000304;"

		// Check if bus is free and send START
		"OR     %[r_temp1], %[r_sda_bitmask], %[r_scl_bitmask];"
		"L16UI  %[r_in_value], %[r_set], 20;"
		"MEMW;"
		"MOVI.N %[r_error], 1;"
		"BNALL  %[r_in_value], %[r_temp1], l_exit_r;"
		"MOVI.N %[r_error], 0;"
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		"S16I   %[r_scl_bitmask], %[r_set], 0;"
		"MEMW;"
		"S16I   %[r_sda_bitmask], %[r_set], 4;"
		"l_r01:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_r01;"
		// Post Condition: SDA = 0, SCL = 1

		// a_temp2 holds the slave address OR 1, i.e. i2c read
		// Send slave address
		"MOVI   %[r_bit_index], 128;"
		"l_send_bit_r:"
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		"BALL   %[r_temp2], %[r_bit_index], l_sda1_scl0_r;"
		"S16I   %[r_scl_bitmask], %[r_set], 4;"
		"MEMW;"
		"S16I   %[r_sda_bitmask], %[r_set], 4;"
		"l_r02:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_r02;"
		"j l_sdax_scl1_r;"
		"l_sda1_scl0_r:"
		"S16I   %[r_scl_bitmask], %[r_set], 4;"
		"MEMW;"
		"S16I   %[r_sda_bitmask], %[r_set], 0;"
		"l_r03:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_r03;"
		"l_sdax_scl1_r:"
		"ADDI   %[r_temp1], %[r_iteration_scl_halfcycle], -5;"
		"S16I   %[r_scl_bitmask], %[r_set], 0;"
		"MEMW;"
		"l_r04:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BGEZ   %[r_temp1], l_r04;"
		"L16UI  %[r_in_value], %[r_set], 20;"
		"MEMW;"
		"MOV.N  %[r_temp1], %[r_iteration_scl_clock_stretch];"
		"BALL   %[r_in_value], %[r_scl_bitmask], l_no_stretch_r;"
		"l_stretch_r:"
		"L16UI  %[r_in_value], %[r_set], 20;"
		"MEMW;"
		"BALL   %[r_in_value], %[r_scl_bitmask], l_scl_high_by_slave_r;"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"BNEZ   %[r_temp1], l_stretch_r;"
		"MOVI.N %[r_error], 8;"
		"j l_exit_r;"
		"l_scl_high_by_slave_r:"
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		"l_r05:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_r05;"
		"l_no_stretch_r:"
		"SRLI   %[r_bit_index], %[r_bit_index], 1;"
		"BNEZ   %[r_bit_index], l_send_bit_r;"
		"S16I   %[r_scl_bitmask], %[r_set], 4;"
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		// Stretch the first half cycle of the 9th cycle a little bit
		// Because the the second half cycle of the 8th cycle was a little bit short, due to the not jump above
		"ADDI.N %[r_temp1], %[r_temp1], 3;"
		"l_r06:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_r06;"
		"ADDI   %[r_temp1], %[r_iteration_scl_halfcycle], -5;"
		"S16I   %[r_scl_bitmask], %[r_set], 0;"
		"MEMW;"
		"l_r07:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BGEZ   %[r_temp1], l_r07;"
		"L16UI  %[r_in_value], %[r_set], 20;"
		"BNALL  %[r_in_value], %[r_sda_bitmask], l_slave_ack_r;"
		"MOVI.N %[r_error], 4;"
		"j l_stop_after_NACK_r;"

		"l_slave_ack_r:"
		// Postcondition:
		//   ACK by slave for slave address and read command
		//	 SDA = 0 by the slave (!)
		//   SCL = 1
		//   9th Clock Cycle is finished
		//   At least one byte should be read by the master, i.e. nr_of_bytes >= 1

		"l_read_byte:"
		// Precondition
		//   SCL = 1
		//   SDA = 0  -- either: by the slave when it ACKnowledged the slave address and read command, 
		//     or by the master, i.e. when the master ACKnowledged the reception of a byte from the slave
		// r_temp2 is free to be re-used
		// r_temp2 will hold the bits read from the slave and eventually a whole byte
		// initialize r_temp2 = 0
		"MOVI.N %[r_temp2], 0;"
		// r_bit_index points to the MSB of data byte to be received (the slave transmitts the MSB first)
		"MOVI   %[r_bit_index], 128;"

		"l_read_bit:"
		// Spike reducing delay: 
		"MOV.N  %[r_temp1], %[r_iteration_minimize_spike];"
		// SCL = 0
		// the master does not yet release SDA, i.e. leave SDA = 0 
		"S16I   %[r_scl_bitmask], %[r_set], 4;" // clear: 0x60000308
		"MEMW;"
		"l_r08:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_r08;"
		// Reduce number of iterations by the ones we've already used
		"SUB    %[r_temp1], %[r_iteration_scl_halfcycle], %[r_iteration_minimize_spike];"
		// Let SDA raise, i.e. the slave gains control over SDA 
		"S16I   %[r_sda_bitmask], %[r_set], 0;"
		"MEMW;"
		"l_r09:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BGEZ   %[r_temp1], l_r09;"

		// Postcondition
		// SCL = 0
		// SDA controlled by slave
		// First half cycle is finished

		// Check for clock stretching and sample the value of SDA slightly before the next clock cycle will be over
		// Therefore, we reduce the delay counter: half_cycle - delta
		"ADDI   %[r_temp1], %[r_iteration_scl_halfcycle], -5;"
		// Let SCL raise
		"S16I   %[r_scl_bitmask], %[r_set], 0;"
		"MEMW;"
		"l_r10:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BGEZ   %[r_temp1], l_r10;"
		// Sample SCL and SDA value
		"L16UI  %[r_in_value], %[r_set], 20;" // offset is 20d = 14h => in: 0x60000318
		"MEMW;"
		// r_temp1 holds the number of iterations for clock stretch timeout 
		"MOV.N  %[r_temp1], %[r_iteration_scl_clock_stretch];"
		// Branch if SCL = 1, i.e. no stretching
		"BALL   %[r_in_value], %[r_scl_bitmask], l_no_stretch_rB;"
		// SCL = 0, i.e. stretching by the slave, i.e. it pulls SCL low
		"l_stretch_rB:"
		// Sample SCL value
		"L16UI  %[r_in_value], %[r_set], 20;" // offset is 20d = 14h => in: 0x60000318
		"MEMW;"
		// Branch if SCL = 1, i.e. no more stretching
		"BALL   %[r_in_value], %[r_scl_bitmask], l_scl_high_by_slave_rB;"
		// SCL is still low
		// Reduce the number of clock stretching iterations
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		// Did we reach the clock stretch timeout?
		// Branch if we have not yet reached the timeout
		"BNEZ   %[r_temp1], l_stretch_rB;"
		// We have reached the clock stretch timeout, i.e. SCL is still pulled low by the slave
		// Different than with the brzo_i2c_write, we don't have to set SDA to high here, since both SDA and SCL 
		//	 are controlled by the slave. 
		// Error: Bus is not free, since SCL is still low AND clock stretch timeout reached
		"MOVI.N %[r_error], 8;"
		// We explicitly do not send a STOP instead we exit, i.e. jump to l_exit and not to l_send_stop
		"j l_exit_r;"

		"l_scl_high_by_slave_rB:"
		// SCL was set high by the slave
		// Also, the slave has started to change SDA, either to an 0 or 1, depending on the value of the bit to send
		// We will thus have to sample SDA at the end of the half cycle minus delta
		"ADDI   %[r_temp1], %[r_iteration_scl_halfcycle], -5;"
		"l_r11:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BGEZ   %[r_temp1], l_r11;"
		// Sample SDA value
		"L16UI  %[r_in_value], %[r_set], 20;" // offset is 20d = 14h => in: 0x60000318
		"MEMW;"

		"l_no_stretch_rB:"
		// Postcondition: SCL = 1 for a half cycle
		// r_in_value contains also the SDA value, i.e. the bit sent from the slave

		// If the bit sent from the slave is 1 then perfom an OR else not.
		"BNALL %[r_in_value], %[r_sda_bitmask], l_slave_sent_low;"
		"OR    %[r_temp2], %[r_temp2], %[r_bit_index];"
		"l_slave_sent_low:"
		// Move the bit index to the next bit
		"SRLI   %[r_bit_index], %[r_bit_index], 1;"
		// Are there bits left that we need to read?
		// When the LSB was read, i.e. bit index was 1 before SRLI, it will now be zero
		// As long as the LSB was not yet received jump
		"BNEZ   %[r_bit_index], l_read_bit;"
		// we have read 8 Bits

		// Postcondition:
		//   SDA = LSB from the slave 
		//   SCL = 1 
		//   8th Clock Cycle is finished

		// Store the received byte held in register r_temp2 into memory
		"S8I    %[r_temp2], %[r_adr_array_element], 0;"
		// Decrement the number of bytes to read
		"ADDI.N %[r_nr_of_bytes], %[r_nr_of_bytes], -1;"
		// Are there any more bytes the master should read?
		"BEQZ   %[r_nr_of_bytes], l_no_more_bytes_to_read;"
		// yes...
		// Move the pointer to the next array element (since we have an array of bytes, the increment is 1)
		"ADDI.N %[r_adr_array_element], %[r_adr_array_element], 1;"

		// Send ACK to the Slave
		// The master sets SCL = 0 and SDA = 0 (= ACK)
		// If the value of the LSB from the slave was 1, SDA will start to fall from high to low
		//  i.e. the master is pulling SDA low. Else SDA will remain low.
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		// SCL = 0
		"S16I   %[r_scl_bitmask], %[r_set], 4;" // clear : 0x60000308
		"MEMW;"
		// SDA = 0
		"S16I   %[r_sda_bitmask], %[r_set], 4;" // clear : 0x60000308
		"l_r12:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_r12;"

		// The master will stop pulling SDA low the end of the 2nd half cycle of the 9th cycle
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		// Set SCL = 1 and still pull SDA low
		"S16I   %[r_scl_bitmask], %[r_set], 0;"
		"l_r13:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_r13;"

		// Postcondition:
		//   SDA = 0
		//   SCL = 1 
		//   9th Clock Cycle is finished

		// Read next byte
		"j l_read_byte;"

		"l_no_more_bytes_to_read:"
		// Send an NACK to the Slave
		// The master sets SCL = 0 and SDA = 1
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		// SCL = 0
		"S16I   %[r_scl_bitmask], %[r_set], 4;" // clear : 0x60000308
		"MEMW;"
		// SDA = 1
		"S16I   %[r_sda_bitmask], %[r_set], 0;"
		"l_r14:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_r14;"
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		// Set SCL = 1 and don't change SDA, i.e. leave SDA = 1
		"S16I   %[r_scl_bitmask], %[r_set], 0;"
		"l_r15:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_r15;"

		// Postcondition
		//   SDA = 1
		//   SCL = 1 
		//   9th Clock Cycle is finished
		"l_stop_after_NACK_r:"
		// Send stop after either the following events
		//  i) Read(slave_address) was not acknowledged by the slave, i.e. No ACK from slave ("NACK")
		// ii) The master has sent an NACK to the slave after having received the last byte, 
		//     i.e. all nr_of_bytes bytes have been received

		// Only in the case ii) a repeated start makes sense, so only then we check for a repeated start
		// Case i) means r_error = 4 
		// Send STOP in case i)
		"BNEZ   %[r_error], l_send_stop_r;"
		// case ii) check for a repeated start, i.e. if r_repeated is 0 then send stop
		"BEQZ   %[r_repeated], l_send_stop_r;"
		// Prepere for repeated start, i.e. finish the 9th cycle by setting SCL = 0 leave SDA = 1
		// And start the first half cycle of the 10th cycle
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		// SCL = 0
		"S16I   %[r_scl_bitmask], %[r_set], 4;" // clear: 0x60000308
		"MEMW;"
		"l_r16:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_r16;"
		// Set SCL = 1
		"S16I   %[r_scl_bitmask], %[r_set], 0;"
		"MEMW;"
		"j l_exit_r;"

		"l_send_stop_r:"
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		// SDA = 0
		"S16I   %[r_sda_bitmask], %[r_set], 4;"  // clear: 0x60000308
		"MEMW;"
		// SCL = 0
		// clear: 0x60000308
		"S16I   %[r_scl_bitmask], %[r_set], 4;" 
		// Delay for the first half cycle of 10th cycle
		"l_r17:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_r17;"
		// Postcondition: SDA = 0 and SCL = 0

		// SCL = 1, leave SDA = 0
		"S16I   %[r_scl_bitmask], %[r_set], 0;"
		// Delay for tsu;STOP >= 4.0 usec for standard mode, 0.6 usec for fast or 0.26 usec fast mode plus
		//  => a delay of one half cycle is enough to meet those timings
		"MEMW;"
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		"l_r18:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_r18;"
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		// leave SCL = 1 and set SDA = 1 (this is the STOP condition)
		"S16I   %[r_sda_bitmask], %[r_set], 0;"
		// Delay for tBUF >= 4.7 usec for standard mode, 1.3 usec for fast or 0.5 usec fast mode plus
		//  => a delay of one half cycle is enough to meet those timings
		"l_r19:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_r19;"

		"l_exit_r:"
		// If BRZO_I2C_DISABLE_INTERRUPTS is set to 1,  
		//   enable all interrupts again, i.e. interrupts with interrupt level >= 1
		#if BRZO_I2C_DISABLE_INTERRUPTS != 0
		"RSIL   %[r_temp1], 0;"
		#endif

		: [r_set] "+r" (a_set), [r_repeated] "+r" (a_repeated), [r_temp1] "+r" (a_temp1), [r_in_value] "+r" (a_in_value), [r_error] "+r" (i2c_error), [r_bit_index] "+r" (a_bit_index), [r_adr_array_element] "+r" (&data[0]), [r_temp2] "+r" (a_temp2), [r_nr_of_bytes] "+r" (nr_of_bytes)
		: [r_sda_bitmask] "r" (sda_bitmask), [r_scl_bitmask] "r" (scl_bitmask), [r_iteration_scl_halfcycle] "r" (iteration_scl_halfcycle), [r_iteration_minimize_spike] "r" (iteration_remove_spike), [r_iteration_scl_clock_stretch] "r" (iteration_scl_clock_stretch)
		: "memory"
	);
	return;
}

void ICACHE_RAM_ATTR brzo_i2c_ACK_polling(uint16_t ACK_polling_time_out_usec) {
	// Timeout for ACK polling in usec
	// Returns 0 or Error encoded as follows
	// Bit 0 (1) : Bus not free, i.e. either SDA or SCL is low
	// Bit 1 (2) : If the ACK polling time out was exceeded, we will have a NACK, too. 
	// Bit 2 (4) : --
	// Bit 3 (8) : --
	// Bit 4 (16): --
	// Bit 5 (32): ACK Polling timeout exceeded

	// Assembler Variables
	uint32_t a_set = 0, a_in_value = 0, a_temp1 = 0, a_bit_index = 0;
	uint16_t iteration_ACK_polling_timeout;
	uint8_t byte_to_send = i2c_slave_address << 1;

	if (ACK_polling_time_out_usec == 0) {
		iteration_ACK_polling_timeout = 1;
	}
	else {
		iteration_ACK_polling_timeout = ACK_polling_time_out_usec / ACK_polling_loop_usec;
	}

	asm volatile (
		// If BRZO_I2C_DISABLE_INTERRUPTS is set to 1, then all interrupts are disabled, 
		//   i.e. interrupts up to the highest interrupt level of 15
		//   the current level is saved in %[r_temp1] but we will not use that value again, 
		//   instead we will just enable all interrupt levels at the end of this routine
		#if BRZO_I2C_DISABLE_INTERRUPTS != 0
		"RSIL   %[r_temp1], 15;"
		#endif
		"MOVI   %[r_set], 0x60000304;"

		// Check if bus is free and send START
		"OR     %[r_temp1], %[r_sda_bitmask], %[r_scl_bitmask];"
		"L16UI  %[r_in_value], %[r_set], 20;" // offset is 20d = 14h = > in: 0x60000318
		"MEMW;"
		"MOVI.N %[r_error], 1;"
		// If either SDA or SCL is low, then bus is not free and thus jump to l_exit_a
		"BNALL  %[r_in_value], %[r_temp1], l_exit_a;"
		// Bus is free, so we can send START
		"MOVI.N %[r_error], 0;"

		// The ACK polling loop starts here
		"l_ACK_loop:"
		// Decrease the number of iterations for the ACK polling loop by 1
		"ADDI   %[r_iteration_ACK_polling_timeout], %[r_iteration_ACK_polling_timeout], -1;"
		
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		// Set SCL = 1
		"S16I   %[r_scl_bitmask], %[r_set], 0;"
		"MEMW;"
		// Set SDA = 0
		// Delay for tHD;STA  >= 4.0 usec for standard mode, 0.6 usec for fast or 0.26 usec fast mode plus
		//  => a delay of one half cycle is enough to meet those timings
		"S16I   %[r_sda_bitmask], %[r_set], 4;"  // clear: 0x60000308
		"l_w01_a:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_w01_a;"
		// Post Condition: SDA = 0, SCL = 1

		// select the MSB of byte_to_send
		"MOVI   %[r_bit_index], 128;"
		// The loop for sending 1...8 bits
		"l_send_bit_a:"
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		// check if the bit of byte_to_send at bit_index is 0 or 1
		"BALL   %[r_byte_to_send], %[r_bit_index], l_sda1_scl0_a;"
		// SDA = 0, SCL = 0
		"S16I   %[r_scl_bitmask], %[r_set], 4;" // clear: 0x60000308
		"MEMW;"
		"S16I   %[r_sda_bitmask], %[r_set], 4;" // clear: 0x60000308
		"l_w02_a:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_w02_a;"
		"j l_sdax_scl1_a;"

		"l_sda1_scl0_a:"
		// SDA = 1, SCL = 0
		"S16I   %[r_scl_bitmask], %[r_set], 4;" // clear: 0x60000308
		"MEMW;"
		"S16I   %[r_sda_bitmask], %[r_set], 0;"
		"l_w03_a:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_w03_a;"

		"l_sdax_scl1_a:"
		// SDA = leave unchanged and set SCL = 1
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		// Let SCL raise
		"S16I   %[r_scl_bitmask], %[r_set], 0;"
		"MEMW;"
		"l_w04_a:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_w04_a;"
		// Postcondition: SCL = 1 for a half cycle
		
		// Are there bits left that we need to send?
		"SRLI   %[r_bit_index], %[r_bit_index], 1;"
		// When the LSB of the byte_to_send was sent, i.e. bit index was 1 before SRLI, it will now be zero
		// As long as the LSB was not sent keep on sending bits, i.e. jump
		"BNEZ   %[r_bit_index], l_send_bit_a;"
		// we have sent 8 Bits

		// check for ACK by slave
		// Precondition
		// SDA = LSB (i.e. SDA = 0, since we have an i2c write), SCL = 1
		// SCL = 0
		// Spike reducing waits here
		"S16I   %[r_scl_bitmask], %[r_set], 4;"  // clear : 0x60000308
		"MOV.N  %[r_temp1], %[r_iteration_minimize_spike];"
		"l_w05_a:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_w05_a;"
		// Reduce number of iterations by the ones we've already used
		"SUB    %[r_temp1], %[r_iteration_scl_halfcycle], %[r_iteration_minimize_spike];"
		// Now we let SDA raise. 
		// In case of an ACK the i2c slave is pulling SDA down
		// In case of an NACK, SDA raises
		"S16I   %[r_sda_bitmask], %[r_set], 0;"
		"MEMW;"
		"l_w06_a:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BGEZ   %[r_temp1], l_w06_a;"

		// Delay is little bit shorter, i.e. half_cycle - delta
		// Because we will have a L16UI after in this half cycle
		"ADDI   %[r_temp1], %[r_iteration_scl_halfcycle], -5;"
		// Set SCL = 1, i.e. start of the second half cycle of the 9th SCL cycle
		"S16I   %[r_scl_bitmask], %[r_set], 0;"
		"MEMW;"
		// Delay for the second half cycle of the 9th SCL cycle
		"l_w07_a:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BGEZ   %[r_temp1], l_w07_a;"
		// Sample SDA at the end of the 9th clock cycle, because
		//      in the case of an NACK we want to leave enough time that SDA can raise 
		// If sda_value AND sda_bitmask == 0 => ACK else we have an NACK
		"L16UI  %[r_in_value], %[r_set], 20;" // offset is 20d = 14h = > in: 0x60000318
		// If there was an ACK then jump
		//  Since we had an ACK, Postcondition: SDA is still pulled low by the slave (SDA = 0), SCL = 1
		"BNALL  %[r_in_value], %[r_sda_bitmask], l_slave_ack_a;"
		// NACK by slave
		"MOVI.N %[r_error], 2;"
		// If we have not yet gone through all iterations, jump to the beginning of the ACK loop
		//   Since we had a NACK, Postcondition: SDA = 1 and SCL = 1
		"BNEZ   %[r_iteration_ACK_polling_timeout], l_ACK_loop;"
		// else we have a timeout and a NACK
		// ACK polling timeout error 
		"MOVI.N %[r_error], 32;"
		// Since we had a NACK, Postcondition: SDA = 1 and SCL = 1
		// We will send a STOP after the timeout and the NACK, just to be on the safe side
		"j l_stop_after_NACK_a;" 
		
		"l_slave_ack_a:"
		// Precondition: SDA = 0 (still pulled low by the slave) and SCL = 1
		// 9th Clock Cycle is finished

		// When we had an ACK after some ACK polling iteration (but without a timeout), 
		//   r_error is = 2, since we obviously had an ACK now, we have to set r_error = 0
		"MOVI.N %[r_error], 0;"

		// The slave will pull SDA low as long as SCL = 1
		// First, we have to set SDA = 0 by the master. This is to prevent a spike if the slave shall release
		//   SDA a little bit too early
		// clear : 0x60000308
		"S16I   %[r_sda_bitmask], %[r_set], 4;"
		//	SDA is still pulled low by the slave (and the master), so we have to signal the slave to release it.
		//  We will do this by letting SCL go low. 
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		// We are at the beginning of the 10th cycle
		// Set SCL = 0
		// During the first half cycle the slave should release SDA...
		"S16I   %[r_scl_bitmask], %[r_set], 4;" // clear : 0x60000308
		"MEMW;"
		"l_w08_a:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_w08_a;"
		// Make sure that the precondition for the next command (i.e. the start) will be met
		// Currently, SCL = 0 and SDA is still set low by the master
		// We thus set both SCL = 1 _and SDA = 1 
		// SDA  = 1
		"S16I   %[r_sda_bitmask], %[r_set], 0;"
		"MEMW;"
		// SCL = 1;
		"S16I   %[r_scl_bitmask], %[r_set], 0;"
		// Postcondition: SCL = 1 and SDA = 1, now the next i2c command send start
		"j l_exit_a;"

		"l_stop_after_NACK_a:"
		// Send stop after NACK and timeout
		// Precondition: SDA = 1, SCL = 1
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		// SDA = 0
		// SCL = 1 : In "normal" cycles we woud set SCL to 0
		"S16I   %[r_sda_bitmask], %[r_set], 4;"  // clear: 0x60000308
		"S16I   %[r_scl_bitmask], %[r_set], 0;"
		// Delay for the first half cycle of 10th cycle
		"MEMW;"
		"l_w11_a:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_w11_a;"
		// Postcondition: SDA = 0 and SCL = 1

		// Now we set SDA = 1 and leave SCL = 1 : This ist the STOP condition, 
		//   i.e. the "A LOW to HIGH transition on the SDA line while SCL is HIGH"
		"MOV.N  %[r_temp1], %[r_iteration_scl_halfcycle];"
		"S16I   %[r_sda_bitmask], %[r_set], 0;"
		// SDA = 1 (SCL is already high, we don't need to change it)
		"MEMW;"
		"l_w12_a:"
		"ADDI.N %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_w12_a;"

		"l_exit_a:"
		// If BRZO_I2C_DISABLE_INTERRUPTS is set to 1,  
		//   enable all interrupts again, i.e. interrupts with interrupt level >= 1
		#if BRZO_I2C_DISABLE_INTERRUPTS != 0
		"RSIL   %[r_temp1], 0;"
		#endif

		: [r_set] "+r" (a_set), [r_temp1] "+r" (a_temp1), [r_in_value] "+r" (a_in_value), [r_error] "+r" (i2c_error), [r_bit_index] "+r" (a_bit_index), [r_byte_to_send] "+r" (byte_to_send), [r_iteration_ACK_polling_timeout] "+r" (iteration_ACK_polling_timeout)
		: [r_sda_bitmask] "r" (sda_bitmask), [r_scl_bitmask] "r" (scl_bitmask), [r_iteration_scl_halfcycle] "r" (iteration_scl_halfcycle), [r_iteration_minimize_spike] "r" (iteration_remove_spike)
		: "memory"
		);
	return;
}

void ICACHE_RAM_ATTR brzo_i2c_start_transaction(uint8_t slave_address, uint16_t SCL_frequency_KHz)
{
	// 7 Bit Slave Address; SCL Frequency in Steps of 100 KHz, range: 100 -- 1000 KHz

	i2c_slave_address = slave_address;
	if (i2c_SCL_frequency != SCL_frequency_KHz) {
		uint16_t fr_sel = (SCL_frequency_KHz + 50) / 100;
		ACK_polling_loop_usec = 95 / fr_sel;
		if (system_get_cpu_freq() == 160) {
			if (fr_sel <= 1) iteration_scl_halfcycle = 156;
			else if (fr_sel == 2) iteration_scl_halfcycle = 79;
			else if (fr_sel == 3) iteration_scl_halfcycle = 51;
			else if (fr_sel == 4) iteration_scl_halfcycle = 38;
			else if (fr_sel == 5) iteration_scl_halfcycle = 30;
			else if (fr_sel == 6) iteration_scl_halfcycle = 24;
			else if (fr_sel == 7) iteration_scl_halfcycle = 20;
			else if (fr_sel == 8) iteration_scl_halfcycle = 18;
			else if (fr_sel == 9) iteration_scl_halfcycle = 15;
			else iteration_scl_halfcycle = 14;
		}
		else {
			// 80 MHz
			if (fr_sel <= 1) iteration_scl_halfcycle = 80;
			else if (fr_sel == 2) iteration_scl_halfcycle = 37;
			else if (fr_sel == 3) iteration_scl_halfcycle = 26;
			else if (fr_sel == 4) iteration_scl_halfcycle = 19;
			else if (fr_sel == 5) iteration_scl_halfcycle = 14;
			else if (fr_sel == 6) iteration_scl_halfcycle = 11;
			else if (fr_sel == 7) iteration_scl_halfcycle = 9;
			else iteration_scl_halfcycle = 8;
		}
	}
}

uint8_t ICACHE_RAM_ATTR brzo_i2c_end_transaction()
{
	// returns 0 if transaction completed successfully or error code encoded as follows
	// Bit 0 (1) : Bus not free, i.e. either SDA or SCL is low
	// Bit 1 (2) : Not ACK ("NACK") by slave during write: Either the slave did not respond to the given slave address; 
	//             or it did not ACK a byte transferred by the master.
	// Bit 2 (4) : Not ACK ("NACK") by slave during read, i.e. slave did not respond to the given slave address
	// Bit 3 (8) : Clock Stretching by slave exceeded maximum clock stretching time
	// Bit 4 (16): Function called with 0 bytes to be read by the master. 
	//            Command not sent to the slave, since this could yield to a bus stall (SDA remains 0)

	uint8_t dummy = i2c_error;
	// clear i2c_error for next transaction
	i2c_error = 0;
	return dummy;
}

#ifdef ARDUINO
void ICACHE_FLASH_ATTR brzo_i2c_setup(uint8_t sda, uint8_t scl, uint32_t clock_stretch_time_out_usec)
#else
void ICACHE_FLASH_ATTR brzo_i2c_setup(uint32_t clock_stretch_time_out_usec)
#endif
{
	// SDA pin, SCL pin
	// maximum time (usec) a slave is allowed to stretch the clock, min. 100 usec

	// Assembler Variables
	uint32_t a_set = 0, a_temp1 = 0;

	if (system_get_cpu_freq() == 160) {
		iteration_remove_spike = 15;
		if (clock_stretch_time_out_usec < 100) iteration_scl_clock_stretch = 730;
		else iteration_scl_clock_stretch = 730 * clock_stretch_time_out_usec / 100;
	}
	else {
		// 80 MHz
		iteration_remove_spike = 7;
		if (clock_stretch_time_out_usec < 100) iteration_scl_clock_stretch = 470;
		else iteration_scl_clock_stretch = 470 * clock_stretch_time_out_usec / 100;
	}

	#ifdef ARDUINO
	pinMode(sda, OUTPUT_OPEN_DRAIN);
	pinMode(scl, OUTPUT_OPEN_DRAIN);
	sda_bitmask = (uint16_t)(1 << sda);
	scl_bitmask = (uint16_t)(1 << scl);
	#else
	ETS_GPIO_INTR_DISABLE();

	PIN_FUNC_SELECT(BRZO_I2C_SDA_MUX, BRZO_I2C_SDA_FUNC);
	PIN_FUNC_SELECT(BRZO_I2C_SCL_MUX, BRZO_I2C_SCL_FUNC);

	GPIO_REG_WRITE(GPIO_PIN_ADDR(GPIO_ID_PIN(BRZO_I2C_SDA_GPIO)), GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(BRZO_I2C_SDA_GPIO))) | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE)); //open drain;
	GPIO_REG_WRITE(GPIO_ENABLE_ADDRESS, GPIO_REG_READ(GPIO_ENABLE_ADDRESS) | (1 << BRZO_I2C_SDA_GPIO));
	GPIO_REG_WRITE(GPIO_PIN_ADDR(GPIO_ID_PIN(BRZO_I2C_SCL_GPIO)), GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(BRZO_I2C_SCL_GPIO))) | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE)); //open drain;
	GPIO_REG_WRITE(GPIO_ENABLE_ADDRESS, GPIO_REG_READ(GPIO_ENABLE_ADDRESS) | (1 << BRZO_I2C_SCL_GPIO));

	ETS_GPIO_INTR_ENABLE();

	sda_bitmask = (uint16_t)(1 << BRZO_I2C_SDA_GPIO);
	scl_bitmask = (uint16_t)(1 << BRZO_I2C_SCL_GPIO);
#endif

	// After setting the pins to open drain, their initial value is LOW
	//   therefore, we have to set SDA and SCL to high and wait a little bit
	asm volatile (
		"MOVI   %[r_set], 0x60000304;"
		"MOVI.N %[r_temp1], 30;"
		"S16I   %[r_scl_bitmask], %[r_set], 0;"
		"MEMW;"
		"S16I   %[r_sda_bitmask], %[r_set], 0;"
		"l_s01:"
		"addi   %[r_temp1], %[r_temp1], -1;"
		"NOP;"
		"BNEZ   %[r_temp1], l_s01;"

		: [r_set] "+r" (a_set), [r_temp1] "+r" (a_temp1)
		: [r_sda_bitmask] "r" (sda_bitmask), [r_scl_bitmask] "r" (scl_bitmask)
		: "memory"
	);
}

void ICACHE_FLASH_ATTR brzo_i2c_reset_bus()
{
	// Not yet implemented
}
