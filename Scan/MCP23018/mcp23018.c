const unsigned int ADDR = 0x20;                      //MCP23018 I2C address when ADDR pin is grounded
//const uint8_t I2C_MCP23018 = B0100000;

// Memory
#define MEM_LEN 256
char databuf[MEM_LEN];
int count;
int loopNo = 0;

/********MCP23018 pinout***/
/* GPIOA-COLS  |GPIOB-ROWS*/
/**************|***********/
/* GPA7 ?      |GPB7 ?    */
/* GPA6 col6   |GPB6 ?    */
/* GPA5 col5   |GPB5 row0 */
/* GPA4 col4   |GPB4 row1 */
/* GPA3 col3   |GPB3 row2 */
/* GPA2 col2   |GPB2 row3 */
/* GPA1 col1   |GPB1 rwo4 */
/* GPA0 col0   |GPB0 row5 */
/**************************/

// [1] http://ww1.microchip.com/downloads/en/DeviceDoc/22103a.pdf

#define I2C_TIMEOUT_US 150
#define I2C_BUS_NO 0

// I2C address A2 A1 A0 are 000 in ergodox



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

// ----- Includes -----

// Compiler Includes
#include <Lib/ScanLib.h>

// Project Includes
#include <cli.h>
#include <kll_defs.h>
#include <led.h>
#include <print.h>
#include <macro.h>
#include <Lib/delay.h>

// Local Includes
#include "mcp23018.h"
#include "i2c_connect.h"



// ----- Defines -----

#if ( DebounceThrottleDiv_define > 0 )
nat_ptr_t Mcp_divCounter = 0;
#endif

#if StrobeDelay_define > 0 && !defined( STROBE_DELAY )
#define STROBE_DELAY StrobeDelay_define
#endif



// ----- Function Declarations -----

// CLI Functions
void cliFunc_McpDebug( char* args );
void cliFunc_McpInfo( char* args );
void cliFunc_McpState( char* args );



// ----- Variables -----

// Scan Module command dictionary
CLIDict_Entry( McpDebug,  "Enables Mcp debug mode, prints out each scan code." NL "\t\tIf argument \033[35mT\033[0m is given, prints out each scan code state transition." );
CLIDict_Entry( McpInfo,   "Print info about the configured Mcp." );
CLIDict_Entry( McpState,  "Prints out the current scan table N times." NL "\t\t \033[1mO\033[0m - Off, \033[1;33mP\033[0m - Press, \033[1;32mH\033[0m - Hold, \033[1;35mR\033[0m - Release, \033[1;31mI\033[0m - Invalid" );

CLIDict_Def( McpCLIDict, "Mcp Module Commands" ) = {
  CLIDict_Item( McpDebug ),
  CLIDict_Item( McpInfo ),
  CLIDict_Item( McpState ),
  { 0, 0, 0 } // Null entry for dictionary end
};

// Debounce Array
McpKeyState Mcp_scanArray[ Mcp_colsNum * Mcp_rowsNum ];
McpPin Mcp_cols[] = {McpPin_0, McpPin_1, McpPin_2, McpPin_3, McpPin_4, McpPin_5, McpPin_6};
McpPin Mcp_rows[] = {McpPin_0, McpPin_1, McpPin_2, McpPin_3, McpPin_4, McpPin_5, McpPin_6};
McpPort McpStrobePort = McpPort_B;
McpPort McpSensePort = McpPort_A;


// Mcp debug flag - If set to 1, for each keypress the scan code is displayed in hex
//                     If set to 2, for each key state change, the scan code is displayed along with the state
uint8_t McpDebugMode = 0;

// Mcp State Table Debug Counter - If non-zero display state table after every Mcp scan
uint16_t McpDebugStateCounter = 0;

// Mcp Scan Counters
uint16_t McpMaxScans  = 0;
uint16_t McpCurScans  = 0;
uint16_t McpPrevScans = 0;

// System Timer used for delaying debounce decisions
extern volatile uint32_t systick_millis_count;



// ----- Functions -----

void Mcp_read_register(uint16_t address, uint16_t register_addr, uint8_t *data){
  // The sequence that defines what is going to be done
  // Send address, send the register to be read, repeated start, address with
  // write bit, request data byte
  uint16_t address_7bit = (address << 1);
  uint16_t read_data_sequence[] = { address_7bit, register_addr,
                          I2C_RESTART, (address_7bit | 0x1), I2C_READ};


  uint16_t wait_loops_done = 0;
  i2c_read(I2C_BUS_NO, read_data_sequence, 5, data);
}

void Mcp_write_register(uint16_t address, uint16_t register_addr, uint8_t data){
  uint16_t address_7bit = (address << 1);
  uint16_t write_data_sequence[] = { address_7bit, register_addr, data};

  i2c_send(I2C_BUS_NO, write_data_sequence, 3);

}

uint8_t Mcp_read_port(McpPort port){
  uint8_t data[] = {0};
  switch(port)
  {
    case McpPort_A:
      Mcp_read_register(MCP_ADDR, GPIOA, &data);
      return data[0];
    case McpPort_B:
      Mcp_read_register(MCP_ADDR, GPIOB, &data);
      return data[0];
    default:
      warn_msg("Attempting to read unknown port");
      return 0;
  }
}

void Mcp_write_port(McpPort port, uint8_t data){
  switch(port)
  {
    case McpPort_A:
      Mcp_write_register(MCP_ADDR, GPIOA, data);
      return;
    case McpPort_B:
      Mcp_write_register(MCP_ADDR, GPIOB, data);
      return;
    default:
      warn_msg("Attempting to write unknown port");
      return 0;
  }
}

uint8_t Mcp_strobe_pin(McpPort port, McpPin pin, McpType mode){
  // NOTE This will zero out all other pins, more complex operation needs
  //      a way of storing the state, or doing a costly read
  uint8_t reg_value = (1 << pin);

  if(mode == HighImpedance){
    reg_value = 0x00;
  }

  Mcp_write_port(port, reg_value);
}

// Setup GPIO pins for Mcp scanning
void Mcp_setup()
{
  i2c_setup();

  // Register Mcp CLI dictionary
  CLI_registerDictionary( McpCLIDict, McpCLIDictName );

  // Setup portA as an input
  Mcp_write_register(MCP_ADDR, 0x00, 0xff);

  // Setup portB as an output
  Mcp_write_register(MCP_ADDR, 0x01, 0x00);

  // Clear out Debounce Array
  for ( uint8_t item = 0; item < Mcp_maxKeys; item++ )
  {
    Mcp_scanArray[ item ].prevState        = McpKeyState_Off;
    Mcp_scanArray[ item ].curState         = McpKeyState_Off;
    Mcp_scanArray[ item ].activeCount      = 0;
    Mcp_scanArray[ item ].inactiveCount    = DebounceDivThreshold_define; // Start at 'off' steady state
    Mcp_scanArray[ item ].prevDecisionTime = 0;
  }

  // Clear scan stats counters
  McpMaxScans  = 0;
  McpPrevScans = 0;
}

void Mcp_keyPositionDebug( McpKeyPosition pos )
{
  // Depending on the state, use a different flag + color
  switch ( pos )
  {
  case McpKeyState_Off:
    print("\033[1mO\033[0m");
    break;

  case McpKeyState_Press:
    print("\033[1;33mP\033[0m");
    break;

  case McpKeyState_Hold:
    print("\033[1;32mH\033[0m");
    break;

  case McpKeyState_Release:
    print("\033[1;35mR\033[0m");
    break;

  case McpKeyState_Invalid:
  default:
    print("\033[1;31mI\033[0m");
    break;
  }
}


// Scan the Mcp for keypresses
// NOTE: scanNum should be reset to 0 after a USB send (to reset all the counters)
void Mcp_scan( uint16_t scanNum )
{

  // Increment stats counters
  if ( scanNum > McpMaxScans ) McpMaxScans = scanNum;
  if ( scanNum == 0 )
  {
    McpPrevScans = McpCurScans;
    McpCurScans = 0;
  }
  else
  {
    McpCurScans++;
  }

  // Read systick for event scheduling
  uint8_t currentTime = (uint8_t)systick_millis_count;

  // For each strobe, scan each of the sense pins
  for ( uint8_t strobe = 0; strobe < Mcp_colsNum; strobe++ )
  {
    // Strobe Pin
    Mcp_strobe_pin(McpStrobePort, Mcp_cols[ strobe ], OpenDrain);
    uint8_t pin_register = Mcp_read_port(McpSensePort);

    /*dbug_msg("Pin register: ");
    printHex(pin_register);
    print(NL);
    delayMicroseconds(100);*/

    // Scan each of the sense pins
    for ( uint8_t sense = 0; sense < Mcp_rowsNum; sense++ )
    {
      // Key position
      uint8_t key = Mcp_colsNum * sense + strobe;
      McpKeyState *state = &Mcp_scanArray[ key ];

      // If first scan, reset state
      if ( scanNum == 0 )
      {
        // Set previous state, and reset current state
        state->prevState = state->curState;
        state->curState  = McpKeyState_Invalid;
      }
      //dbug_msg("Reading key\n");

      // Signal Detected
      // Inverting the input register as we are detecting the lack of voltage
      // Increment count and right shift opposing count
      // This means there is a maximum of scan 13 cycles on a perfect off to on transition
      //  (coming from a steady state 0xFFFF off scans)
      // Somewhat longer with switch bounciness
      // The advantage of this is that the count is ongoing and never needs to be reset
      // State still needs to be kept track of to deal with what to send to the Macro module
      if ( !pin_register & (1 << Mcp_rows[ sense ]) )
      {
        //dbug_msg("Detected key!\n");

        // Only update if not going to wrap around
        if ( state->activeCount < DebounceDivThreshold_define ) state->activeCount += 1;
        state->inactiveCount >>= 1;
      }
      // Signal Not Detected
      else
      {
        // Only update if not going to wrap around
        if ( state->inactiveCount < DebounceDivThreshold_define ) state->inactiveCount += 1;
        state->activeCount >>= 1;
      }

      // Check for state change if it hasn't been set
      // But only if enough time has passed since last state change
      // Only check if the minimum number of scans has been met
      //   the current state is invali
      //   and either active or inactive count is over the debounce threshold
      if ( state->curState == McpKeyState_Invalid )
      {
        // Determine time since last decision
        uint8_t lastTransition = currentTime - state->prevDecisionTime;

        // Attempt state transition
        switch ( state->prevState )
        {
        case McpKeyState_Press:
        case McpKeyState_Hold:
          if ( state->activeCount > state->inactiveCount )
          {
            state->curState = McpKeyState_Hold;
          }
          else
          {
            // If not enough time has passed since Hold
            // Keep previous state
            if ( lastTransition < MinDebounceTime_define )
            {
              //warn_print("FAST Release stopped");
              state->curState = state->prevState;
              continue;
            }

            state->curState = McpKeyState_Release;
          }
          break;

        case McpKeyState_Release:
        case McpKeyState_Off:
          if ( state->activeCount > state->inactiveCount )
          {
            // If not enough time has passed since Hold
            // Keep previous state
            if ( lastTransition < MinDebounceTime_define )
            {
              //warn_print("FAST Press stopped");
              state->curState = state->prevState;
              continue;
            }

            state->curState = McpKeyState_Press;
          }
          else
          {
            state->curState = McpKeyState_Off;
          }
          break;

        case McpKeyState_Invalid:
        default:
          erro_print("Mcp scan bug!! Report me!");
          break;
        }

        // Update decision time
        state->prevDecisionTime = currentTime;

        // Send keystate to macro module

        // Mcp Debug, only if there is a state change
        if ( McpDebugMode && state->curState != state->prevState )
        {
          // Basic debug output
          if ( McpDebugMode == 1 && state->curState == McpKeyState_Press )
          {
            printHex( key );
            print(" ");
          }
          // State transition debug output
          else if ( McpDebugMode == 2 )
          {
            printHex( key );
            Mcp_keyPositionDebug( state->curState );
            print(" ");
          }
        }
      }
    }

    // Unstrobe Pin
    // NOTE Currently this will set all pins to 0 (high impendace)
    Mcp_strobe_pin(McpStrobePort, Mcp_cols[ strobe ], HighImpedance);
  } // for strobe pins


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .


  // State Table Output Debug
  if ( McpDebugStateCounter > 0 )
  {
    // Decrement counter
    McpDebugStateCounter--;

    // Output stats on number of scans being done per USB send
    print( NL );
    info_msg("Max scans:      ");
    printHex( McpMaxScans );
    print( NL );
    info_msg("Previous scans: ");
    printHex( McpPrevScans );
    print( NL );

    // Output current scan number
    info_msg("Scan Number:    ");
    printHex( scanNum );
    print( NL );

    // Display the state info for each key
    print("<key>:<previous state><current state> <active count> <inactive count>");
    for ( uint8_t key = 0; key < Mcp_maxKeys; key++ )
    {
      // Every 4 keys, put a newline
      if ( key % 4 == 0 )
        print( NL );

      print("\033[1m0x");
      printHex_op( key, 2 );
      print("\033[0m");
      print(":");
      Mcp_keyPositionDebug( Mcp_scanArray[ key ].prevState );
      Mcp_keyPositionDebug( Mcp_scanArray[ key ].curState );
      print(" 0x");
      printHex_op( Mcp_scanArray[ key ].activeCount, 4 );
      print(" 0x");
      printHex_op( Mcp_scanArray[ key ].inactiveCount, 4 );
      print(" ");
    }

    print( NL );
  }
}


// Called by parent scan module whenever the available current changes
// current - mA
void Mcp_currentChange( unsigned int current )
{
  // TODO - Any potential power savings?
}



// ----- CLI Command Functions -----

void cliFunc_McpInfo( char* args )
{
  print( NL );
  info_msg("Columns:  ");
  printHex( Mcp_colsNum );

  print( NL );
  info_msg("Rows:     ");
  printHex( Mcp_rowsNum );

  print( NL );
  info_msg("Max Keys: ");
  printHex( Mcp_maxKeys );
}

void cliFunc_McpDebug( char* args )
{
  // Parse number from argument
  //  NOTE: Only first argument is used
  char* arg1Ptr;
  char* arg2Ptr;
  CLI_argumentIsolation( args, &arg1Ptr, &arg2Ptr );

  // Set the Mcp debug flag depending on the argument
  // If no argument, set to scan code only
  // If set to T, set to state transition
  switch ( arg1Ptr[0] )
  {
  // T as argument
  case 'T':
  case 't':
    McpDebugMode = McpDebugMode != 2 ? 2 : 0;
    break;

  // No argument
  case '\0':
    McpDebugMode = McpDebugMode != 1 ? 1 : 0;
    break;

  // Invalid argument
  default:
    return;
  }

  print( NL );
  info_msg("Mcp Debug Mode: ");
  printInt8( McpDebugMode );
}

void cliFunc_McpState( char* args )
{
  // Parse number from argument
  //  NOTE: Only first argument is used
  char* arg1Ptr;
  char* arg2Ptr;
  CLI_argumentIsolation( args, &arg1Ptr, &arg2Ptr );

  // Default to 1 if no argument is given
  McpDebugStateCounter = 1;

  if ( arg1Ptr[0] != '\0' )
  {
    McpDebugStateCounter = (uint16_t)numToInt( arg1Ptr );
  }
}



void Mcp_read_pins()
{
  //delayMicroseconds( 10 );
  loopNo++;
  uint16_t addr = (0x20 << 1);
  uint8_t rcv[] = {2, 3, 4, 5, 1, 1 , 1, 1, 1, 1, 1, 1, 1};
  /*uint16_t reg = 0x00;
  uint16_t val = 0xFF;
  uint16_t configInput[] = { addr, 0x00, val };
  uint16_t configInput2[] = { addr, 0x12, val };
  uint16_t readDataSeq[] = { (addr | 0x1), I2C_READ, I2C_READ, I2C_READ, I2C_READ, I2C_READ, I2C_READ, I2C_READ, I2C_READ, I2C_READ, I2C_READ, I2C_READ, I2C_READ, I2C_READ, I2C_READ, I2C_READ, I2C_READ};

  uint16_t readData[] = { (addr), 0x12, I2C_RESTART, (addr | 0x1), I2C_READ, I2C_READ, I2C_READ, I2C_READ};

  uint16_t writeLedOn[] = { addr, 0x13, 0xFF };
  uint16_t writeLedOff[] = { addr, 0x13, 0x00 };
  uint16_t configLed[] = { addr, 0x01, 0x00 };

  uint8_t rcv_byte = 2;*/

  // Setup page




  if(loopNo > 800){
      // Write register
      //dbug_msg("Sending.. \n");
      //delayMicroseconds(123);
      //while(i2c_send( 0, configInput, 3) == -1){
      //  delayMicroseconds( 100 );
      //}
      /*while(i2c_send( 0, configInput2, 3) == -1){
        delayMicroseconds( 100 );
      }*/
      /*while(i2c_send( 0, configLed, 3) == -1){
          //dbug_msg("Write Channel is busy\n");
          delayMicroseconds( 100 );
        }
      delayMicroseconds( 5000 );

      while(i2c_send( 0, writeData, 3) == -1){
          //dbug_msg("Write Channel is busy\n");
          delayMicroseconds( 100 );
        }

      delayMicroseconds( 5000 );

      while(i2c_send( 0, writeLedOn, 3) == -1){
          //dbug_msg("Write Channel is busy\n");
          delayMicroseconds( 100 );
        }
      delayMicroseconds( 5000 );

      while(i2c_send( 0, writeLedOff, 3) == -1){
          //dbug_msg("Write Channel is busy\n");
          delayMicroseconds( 100 );
        }*/
    Mcp_write_register(addr, 0x00, 0xff);
    Mcp_write_register(addr, 0x01, 0x00);
    Mcp_write_register(addr, 0x13, 0x00);

      print("|");
      Mcp_read_register(addr, 0x12, &rcv);

      if(i2c_get_read_valid(0)){
        print("|");
        for(int loop = 0; loop < 1; loop++){
          printHex(rcv[loop]);
            //printHex(rcv_byte);
          print("|");
        }
        printInt8(i2c_get_read_valid(0));
        print(NL);

      }
      Mcp_write_register(addr, 0x01, 0xff);

      //delayMicroseconds( 7);

}
  else{
      delayMicroseconds(2000);
    }
}


