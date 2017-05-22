/* Copyright (C) 2014-2016 by Jacob Alexander
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#ifndef I2C_LIB
#define I2C_LIB
// ----- Defines -----

#define I2C_TxBufferLength 300
#define I2C_RxBufferLength 16




// ----- Includes -----

// Compiler Includes
#include <stdint.h>


// ----- Structs -----

typedef struct I2C_Buffer {
  uint16_t  head;
  uint16_t  tail;
  uint8_t   sequencePos;
  uint16_t  size;
  uint8_t  *buffer;
} I2C_Buffer;

// ----- Functions -----
void I2C_clearTxBuffer();

void I2C_setup();

void I2C_writeReg( uint8_t reg, uint8_t val, uint8_t page );
void I2C_readPage( uint8_t len, uint8_t page );

uint8_t I2C_Send( uint8_t *data, uint8_t sendLen, uint8_t recvLen );
uint8_t I2C_GetRxBuffer(uint8_t sequence);

void I2C_writeReg( uint8_t reg, uint8_t val, uint8_t page );

void I2C_readPage( uint8_t len, uint8_t page );

void setReceiving(int val);

#endif
