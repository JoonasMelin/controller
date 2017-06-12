#pragma once

#ifndef MCP23018_H
#define MCP23018_H

/* Copyright (C) 2014-2016 by Jacob Alexander
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

// ----- Includes -----

// KLL Generated Defines
#include <kll_defs.h>



// ----- Defines -----
#define Mcp_colsNum 7
#define Mcp_rowsNum 5
#define Mcp_maxKeys Mcp_colsNum*Mcp_rowsNum

#if   ( DebounceDivThreshold_define < 0xFF + 1 )
#define DebounceCounter uint8_t
#elif ( DebounceDivThreshold_define < 0xFFFF + 1 )
#define DebounceCounter uint16_t
#elif ( DebounceDivThreshold_define < 0xFFFFFFFF + 1 )
#define DebounceCounter uint32_t
#else
#error "Debounce threshold is too high... 32 bit max. Check .kll defines."
#endif

#if   ( MinDebounceTime_define > 0xFF )
#error "MinDebounceTime is a maximum of 255 ms"
#elif ( MinDebounceTime_define < 0x00 )
#error "MinDebounceTime is a minimum 0 ms"
#endif

// The MCP address is 0x20 when the addr pin is grounded
#define MCP_ADDR 0x20



#define IODIRA 0x00 // From [1] Read Table 1-1 and 1.6.1 I/O DIRECTION REGISTER (page 18)
#define IODIRB 0x01 // From [1] Read Table 1-1 and 1.6.1 I/O DIRECTION REGISTER (page 18)
#define IOCONA 0x0A // From [1] Read Table 1-1 and 1.6.6 CONFIGURATION REGISTER
#define IOCONB 0x0B // From [1] Read Table 1-1 and 1.6.6 CONFIGURATION REGISTER
#define IOCON  IOCONA // From [1] Read Table 1-1 and 1.6.6 CONFIGURATION REGISTER
#define GPPUA  0x0C // From [1] Read Table 1-1 and 1.6.7 pull-up resistor configuration register
#define GPPUB  0x0D // From [1] Read Table 1-1 and 1.6.7 pull-up resistor configuration register
#define GPIOA  0x12 // From [1] Read Table 1-1 and 1.6.10 PORT REGISTER (page 28)
#define GPIOB  0x13 // From [1] Read Table 1-1 and 1.6.10 PORT REGISTER (page 28)
#define OLATA  0x14 // From [1] Read Table 1-1 and 1.6.11 OUTPUT LATCH REGISTER (page 29)
#define OLATB  0x15 // From [1] Read Table 1-1 and 1.6.11 OUTPUT LATCH REGISTER (page 29)
// MCP23018 registers

// ----- Enums -----

// Freescale MK20s have GPIO ports A...E
typedef enum McpPort {
  McpPort_A = 0,
  McpPort_B = 1,
} McpPort;

// Each port has a possible 8 pins
typedef enum McpPin {
  McpPin_0  = 0,
  McpPin_1  = 1,
  McpPin_2  = 2,
  McpPin_3  = 3,
  McpPin_4  = 4,
  McpPin_5  = 5,
  McpPin_6  = 6,
  McpPin_7  = 7,
} McpPin;



typedef enum McpType {
  OpenDrain,
  HighImpedance,
} McpType;


// Keypress States
typedef enum McpKeyPosition {
  McpKeyState_Off     = 0,
  McpKeyState_Press   = 1,
  McpKeyState_Hold    = 2,
  McpKeyState_Release = 3,
  McpKeyState_Invalid,
} McpKeyPosition;



// ----- Structs -----

// Debounce Element
typedef struct McpKeyState {
  DebounceCounter activeCount;
  DebounceCounter inactiveCount;
  McpKeyPosition     prevState;
  McpKeyPosition     curState;
  uint8_t         prevDecisionTime;
} __attribute__((packed)) McpKeyState;


// utility
inline uint8_t McpKeyOn(/*KeyPosition*/uint8_t st)
{
  return (st == McpKeyState_Press || st == McpKeyState_Hold) ? 1 : 0;
}


// ----- Functions -----

void Mcp_scan( uint16_t scanNum );

void Mcp_currentChange( unsigned int current );

void Mcp_setup();

void Mcp_read_pins();


#endif // MCP23018_H
