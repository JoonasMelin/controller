/* Copyright (C) 2014-2017 by Jacob Alexander
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

// ----- Includes -----

// Compiler Includes
#include <Lib/ScanLib.h>

// Project Includes
#include <cli.h>
#include <kll_defs.h>
#include <print.h>

// KLL Include
#include <kll.h>

#include <i2c_connect.h>




// Before sending the sequence, I2C_TxBuffer_CurLen is assigned and as each byte is sent, it is decremented
// Once I2C_TxBuffer_CurLen reaches zero, a STOP on the I2C bus is sent
volatile uint8_t I2C_TxBufferPtr[ I2C_TxBufferLength ];
volatile uint8_t I2C_RxBufferPtr[ I2C_TxBufferLength ];

volatile I2C_Buffer I2C_TxBuffer = { 0, 0, 0, I2C_TxBufferLength, (uint8_t*)I2C_TxBufferPtr };
volatile I2C_Buffer I2C_RxBuffer = { 0, 0, 0, I2C_RxBufferLength, (uint8_t*)I2C_RxBufferPtr };



// ----- Function Declarations -----

// CLI Functions
void cliFunc_i2cRecv ( char* args );
void cliFunc_i2cSend ( char* args );
void cliFunc_ledCtrl ( char* args );
void cliFunc_ledRPage( char* args );
void cliFunc_ledStart( char* args );
void cliFunc_ledTest ( char* args );
void cliFunc_ledWPage( char* args );
void cliFunc_ledZero ( char* args );

uint8_t I2C_TxBufferPop();
void I2C_BufferPush( uint8_t byte, I2C_Buffer *buffer );
uint16_t I2C_BufferLen( I2C_Buffer *buffer );
uint8_t I2C_Send( uint8_t *data, uint8_t sendLen, uint8_t recvLen );



// ----- Variables -----

// Scan Module command dictionary
/*CLIDict_Entry( i2cRecv,     "Send I2C sequence of bytes and expect a reply of 1 byte on the last sequence." NL "\t\tUse |'s to split sequences with a stop." );
CLIDict_Entry( i2cSend,     "Send I2C sequence of bytes. Use |'s to split sequences with a stop." );
CLIDict_Entry( ledCtrl,     "Basic LED control. Args: <mode> <amount> [<index>]" );
CLIDict_Entry( ledRPage,    "Read the given register page." );
CLIDict_Entry( ledStart,    "Disable software shutdown." );
CLIDict_Entry( ledTest,     "Test out the led pages." );
CLIDict_Entry( ledWPage,    "Write to given register page starting at address. i.e. 0x2 0x24 0xF0 0x12" );
CLIDict_Entry( ledZero,     "Zero out LED register pages (non-configuration)." );

CLIDict_Def( ledCLIDict, "ISSI LED Module Commands" ) = {
	CLIDict_Item( i2cRecv ),
	CLIDict_Item( i2cSend ),
	CLIDict_Item( ledCtrl ),
	CLIDict_Item( ledRPage ),
	CLIDict_Item( ledStart ),
	CLIDict_Item( ledTest ),
	CLIDict_Item( ledWPage ),
	CLIDict_Item( ledZero ),
	{ 0, 0, 0 } // Null entry for dictionary end
};*/

int receiving = 0;
void setReceiving(int val){
  receiving = val;
}


// ----- Interrupt Functions -----

void i2c0_isr()
{
	cli(); // Disable Interrupts

	uint8_t status = I2C0_S; // Read I2C Bus status

	// Master Mode Transmit
	if ( I2C0_C1 & I2C_C1_TX )
	{
		// Check current use of the I2C bus
		// Currently sending data
		if ( I2C_TxBuffer.sequencePos > 0 )
		{
			// Make sure slave sent an ACK
			if ( status & I2C_S_RXAK )
			{
				// NACK Detected, disable interrupt
        //erro_print("I2C TX NAK detected...");
				I2C0_C1 = I2C_C1_IICEN;

				// Abort Tx Buffer
				I2C_TxBuffer.head = 0;
				I2C_TxBuffer.tail = 0;
				I2C_TxBuffer.sequencePos = 0;
			}
			else
			{
				// Transmit byte
				I2C0_D = I2C_TxBufferPop();
			}
		}
		// Receiving data
    //else if ( I2C_RxBuffer.sequencePos > 0 )
    if(receiving)
		{
			// Master Receive, addr sent
			if ( status & I2C_S_ARBL )
			{
				// Arbitration Lost
				erro_print("Arbitration lost...");
				// TODO Abort Rx

				I2C0_C1 = I2C_C1_IICEN;
				I2C0_S = I2C_S_ARBL | I2C_S_IICIF; // Clear ARBL flag and interrupt
			}
      // FIXME
      if ( status & I2C_S_RXAK )
      //if(0)
			{
				// Slave Address NACK Detected, disable interrupt
        //erro_print("Slave Address I2C NACK detected...");
				// TODO Abort Rx

				I2C0_C1 = I2C_C1_IICEN;
			}
			else
			{
          receiving = 0;
        //dbug_msg("Attempting to read byte - ");
        //printHex( I2C_RxBuffer.sequencePos );
        //print( NL );
				I2C0_C1 = I2C_RxBuffer.sequencePos == 1
					? I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TXAK // Single byte read
					: I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST; // Multi-byte read
			}
		}
		else
		{

      //dbug_msg("STOP - ");
      //printHex( I2C_BufferLen( (I2C_Buffer*)&I2C_TxBuffer ) );
      //print(NL);


			// Delay around STOP to make sure it actually happens...
			delayMicroseconds( 1 );
			I2C0_C1 = I2C_C1_IICEN; // Send STOP
			delayMicroseconds( 7 );

			// If there is another sequence, start sending
			if ( I2C_BufferLen( (I2C_Buffer*)&I2C_TxBuffer ) < I2C_TxBuffer.size )
			{
				// Clear status flags
				I2C0_S = I2C_S_IICIF | I2C_S_ARBL;

        uint16_t waitLoops = 0;
				// Wait...till the master dies
        while ( (I2C0_S & I2C_S_BUSY) && waitLoops < 500  ){
            dbug_msg("So busy out here..");
            waitLoops++;
            delayMicroseconds(10);
          }

				// Enable I2C interrupt
				I2C0_C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TX;

				// Transmit byte
        I2C0_D = I2C_TxBufferPop();
			}
		}
	}
	// Master Mode Receive
	else
	{
		// XXX Do we need to handle 2nd last byte?
    //I2C0_C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TXAK; // No STOP, Rx, NAK on recv

		// Last byte
		if ( I2C_TxBuffer.sequencePos <= 1 )
		{
			// Change to Tx mode
			I2C0_C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;

			// Grab last byte
			I2C_BufferPush( I2C0_D, (I2C_Buffer*)&I2C_RxBuffer );

			delayMicroseconds( 1 ); // Should be enough time before issuing the stop
			I2C0_C1 = I2C_C1_IICEN; // Send STOP
		}
		else
		{
			// Retrieve data
      //dbug_msg("Retrieving data");
			I2C_BufferPush( I2C0_D, (I2C_Buffer*)&I2C_RxBuffer );
		}
	}

	I2C0_S = I2C_S_IICIF; // Clear interrupt

	sei(); // Re-enable Interrupts
}



// ----- Functions -----
void I2C_clearTxBuffer(){
  I2C_TxBuffer.head = 0;
  I2C_TxBuffer.sequencePos = 0;
  I2C_TxBuffer.tail = 0;
}

inline void I2C_setup()
{
	// Enable I2C internal clock
	SIM_SCGC4 |= SIM_SCGC4_I2C0; // Bus 0


	// External pull-up resistor
  // 0, 1
  //(PORT_PCR_MUX(alt)|PORT_PCR_ODE|PORT_PCR_SRE|PORT_PCR_DSE)
  PORTB_PCR0 = PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(2);
  PORTB_PCR1 = PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(2);

	// SCL Frequency Divider
	// 400kHz -> 120 (0x85) @ 48 MHz F_BUS
	I2C0_F = 0x85;
	I2C0_FLT = 4;
	I2C0_C1 = I2C_C1_IICEN;
	I2C0_C2 = I2C_C2_HDRS; // High drive select

	// Enable I2C Interrupt
	NVIC_ENABLE_IRQ( IRQ_I2C0 );
}

void I2C_writeReg( uint8_t reg, uint8_t val, uint8_t page )
{
	// Page Setup
	uint8_t pageSetup[] = { 0xE8, 0xFD, page };

	// Reg Write Setup
	uint8_t writeData[] = { 0xE8, reg, val };

	// Setup page
	while ( I2C_Send( pageSetup, sizeof( pageSetup ), 0 ) == 0 )
		delay(1);

	while ( I2C_Send( writeData, sizeof( writeData ), 0 ) == 0 )
		delay(1);

}

void I2C_readPage( uint8_t len, uint8_t page )
{

	// Page Setup
	uint8_t pageSetup[] = { 0xE8, 0xFD, page };

	// Setup page
	while ( I2C_Send( pageSetup, sizeof( pageSetup ), 0 ) == 0 )
		delay(1);

	// Register Setup
	uint8_t regSetup[] = { 0xE8, 0x00 };

	// Read each register in the page
	for ( uint8_t reg = 0; reg < len; reg++ )
	{
		// Update register to read
		regSetup[1] = reg;

		// Configure register
		while ( I2C_Send( regSetup, sizeof( regSetup ), 0 ) == 0 )
			delay(1);

		// Register Read Command
		uint8_t regReadCmd[] = { 0xE9 };

		// Request single register byte
		while ( I2C_Send( regReadCmd, sizeof( regReadCmd ), 1 ) == 0 )
			delay(1);
		dbug_print("NEXT");
	}

}

inline uint8_t I2C_BufferCopy( uint8_t *data, uint8_t sendLen, uint8_t recvLen, I2C_Buffer *buffer )
{
	uint8_t reTurn = 0;

	// If sendLen is greater than buffer fail right away
  if ( sendLen > buffer->size ){
    erro_msg("Sendlen > buffer->size");
    printHex(sendLen);
    print(" > ");
    printHex(buffer->size);
    print(NL);
		return 0;
   }

	// Calculate new tail to determine if buffer has enough space
	// The first element specifies the expected number of bytes from the slave (+1)
	// The second element in the new buffer is the length of the buffer sequence (+1)
	uint16_t newTail = buffer->tail + sendLen + 2;
	if ( newTail >= buffer->size )
		newTail -= buffer->size;

  if ( I2C_BufferLen( buffer ) < sendLen + 2 ){
      erro_msg("I2C_BufferLen( buffer ) < sendLen + 2 \n");
      printInt8(I2C_BufferLen( buffer ));
      print(" < ");
      printInt8((sendLen + 2));
      print(NL);
      //delay(500);
      return 0;
     }


  print(NL);
  print("sendLen|");
	printHex( sendLen + 2 );
  print("| ");
  printHex( buffer->head );
  print(" <head tail> ");
  printHex( buffer->tail );
  print(" new tail@> ");
	printHex( newTail );
  print(NL);


	// If buffer is clean, return 1, otherwise 2
	reTurn = buffer->head == buffer->tail ? 1 : 2;

	// Add to buffer, already know there is enough room (simplifies adding logic)
	uint8_t bufferHeaderPos = 0;
	for ( uint16_t c = 0; c < sendLen; c++ )
	{
		// Add data to buffer
		switch ( bufferHeaderPos )
		{
		case 0:
			buffer->buffer[ buffer->tail ] = recvLen;
			bufferHeaderPos++;
			c--;
			break;

		case 1:
			buffer->buffer[ buffer->tail ] = sendLen;
			bufferHeaderPos++;
			c--;
			break;

		default:
			buffer->buffer[ buffer->tail ] = data[ c ];
			break;
		}

		// Check for wrap-around case
		if ( buffer->tail + 1 >= buffer->size )
		{
			buffer->tail = 0;
		}
		// Normal case
		else
		{
			buffer->tail++;
		}
	}

  //reTurn = 1;
	return reTurn;
}


inline uint16_t I2C_BufferLen( I2C_Buffer *buffer )
{
	// Tail >= Head
	if ( buffer->tail >= buffer->head )
		return buffer->head + buffer->size - buffer->tail;

	// Head > Tail
	return buffer->head - buffer->tail;
}


void I2C_BufferPush( uint8_t byte, I2C_Buffer *buffer )
{
  //dbug_msg("DATA: ");
  //printHex( byte );

	// Make sure buffer isn't full
	if ( buffer->tail + 1 == buffer->head || ( buffer->head > buffer->tail && buffer->tail + 1 - buffer->size == buffer->head ) )
	{
		warn_msg("I2C_BufferPush failed, buffer full: ");
		printHex( byte );
		print( NL );
		return;
	}

	// Check for wrap-around case
	if ( buffer->tail + 1 >= buffer->size )
	{
		buffer->tail = 0;
	}
	// Normal case
	else
	{
		buffer->tail++;
	}

	// Add byte to buffer
	buffer->buffer[ buffer->tail ] = byte;
}


uint8_t I2C_TxBufferPop()
{
	// Return 0xFF if no buffer left (do not rely on this)
	if ( I2C_BufferLen( (I2C_Buffer*)&I2C_TxBuffer ) >= I2C_TxBuffer.size )
	{
		erro_msg("No buffer to pop an entry from... ");
		printHex( I2C_TxBuffer.head );
		print(" ");
		printHex( I2C_TxBuffer.tail );
		print(" ");
		printHex( I2C_TxBuffer.sequencePos );
		print(NL);
		return 0xFF;
	}

	// If there is currently no sequence being sent, the first entry in the RingBuffer is the length
	if ( I2C_TxBuffer.sequencePos == 0 )
	{
    //dbug_msg("Nothing being sent?!\n");
		I2C_TxBuffer.sequencePos = 0xFF; // So this doesn't become an infinite loop
		I2C_RxBuffer.sequencePos = I2C_TxBufferPop();
		I2C_TxBuffer.sequencePos = I2C_TxBufferPop();
	}

	uint8_t data = I2C_TxBuffer.buffer[ I2C_TxBuffer.head ];

  /*dbug_msg("Popping: ");
  printHex( data );
  print(" <data head> ");
  printInt8( I2C_TxBuffer.head );
  print(" tail> ");
  printInt8( I2C_TxBuffer.tail );
  print(" sqpos> ");
  printInt8( I2C_TxBuffer.sequencePos );
  print(NL);*/

	// Prune head
	I2C_TxBuffer.head++;

	// Wrap-around case
	if ( I2C_TxBuffer.head >= I2C_TxBuffer.size )
		I2C_TxBuffer.head = 0;

	// Decrement buffer sequence (until next stop will be sent)
	I2C_TxBuffer.sequencePos--;

	return data;
}

uint8_t I2C_Send( uint8_t *data, uint8_t sendLen, uint8_t recvLen )
{
	// Check head and tail pointers
	// If full, return 0
	// If empty, start up I2C Master Tx
	// If buffer is non-empty and non-full, just append to the buffer
  //I2C_TxBuffer.tail = I2C_TxBuffer.head;
	switch ( I2C_BufferCopy( data, sendLen, recvLen, (I2C_Buffer*)&I2C_TxBuffer ) )
	{
	// Not enough buffer space...
	case 0:
		
		erro_msg("Not enough Tx buffer space... ");
		printHex( I2C_TxBuffer.head );
		print(":");
		printHex( I2C_TxBuffer.tail );
		print("+");
    printInt8( sendLen );
		print("|");
		printHex( I2C_TxBuffer.size );
		print( NL );

		return 0;

	// Empty buffer, initialize I2C
	case 1:
		// Clear status flags
		I2C0_S = I2C_S_IICIF | I2C_S_ARBL;

		// Check to see if we already have control of the bus
		if ( I2C0_C1 & I2C_C1_MST )
		{
			// Already the master (ah yeah), send a repeated start
			I2C0_C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_RSTA | I2C_C1_TX;
		}
		// Otherwise, seize control
		else
		{
			// Wait...till the master dies
			while ( I2C0_S & I2C_S_BUSY );

			// Now we're the master (ah yisss), get ready to send stuffs
      dbug_msg("Properly a master now!\n");
			I2C0_C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;
		}

		// Enable I2C interrupt
    I2C0_C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TX;

    // Depending on what type of transfer, the first byte is configured for R or W
    uint8_t data = I2C_TxBufferPop();
    I2C0_D = data;
    dbug_msg("Popped ");
    printHex(data);
    print(" from the TX buffer");
    print(NL);
    /*for(uint8_t sentByte = 0; sentByte < sendLen; sentByte++){


      while ( (I2C0_S & I2C_S_BUSY)){
          //dbug_msg("So busy out here..");
          print(NL);
          delayMicroseconds(10);
        }
    }*/



    //if(recvLen > 0){
        //I2C0_C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TXAK;
    //  }

		return 1;
	}
  dbug_msg("Buffer was dirty");
	// Dirty buffer, I2C already initialized
	return 2;
}

uint8_t I2C_GetRxBuffer(uint8_t sequence){
  //sequence = I2C_RxBuffer.sequencePos;
  dbug_msg("\n\nRX buffer tail is at: ");
  printInt8(I2C_RxBuffer.tail);
  print(" RX buffer at tail: ");
  printHex(I2C_RxBuffer.buffer[I2C_RxBuffer.tail]);
  print(NL);
  return I2C_RxBuffer.buffer[sequence];
}

// ----- CLI Command Functions -----

// TODO Currently not working correctly
void cliFunc_i2cSend( char* args )
{
	char* curArgs;
	char* arg1Ptr;
	char* arg2Ptr = args;

	// Buffer used after interpretting the args, will be sent to I2C functions
	// NOTE: Limited to 8 bytes currently (can be increased if necessary
	#define i2cSend_BuffLenMax 8
	uint8_t buffer[ i2cSend_BuffLenMax ];
	uint8_t bufferLen = 0;

	// No \r\n by default after the command is entered
	print( NL );
	info_msg("Sending: ");

	// Parse args until a \0 is found
	while ( bufferLen < i2cSend_BuffLenMax )
	{
		curArgs = arg2Ptr; // Use the previous 2nd arg pointer to separate the next arg from the list
		CLI_argumentIsolation( curArgs, &arg1Ptr, &arg2Ptr );

		// Stop processing args if no more are found
		if ( *arg1Ptr == '\0' )
			break;

		// If | is found, end sequence and start new one
		if ( *arg1Ptr == '|' )
		{
			print("| ");
			I2C_Send( buffer, bufferLen, 0 );
			bufferLen = 0;
			continue;
		}

		// Interpret the argument
		buffer[ bufferLen++ ] = (uint8_t)numToInt( arg1Ptr );

		// Print out the arg
		dPrint( arg1Ptr );
		print(" ");
	}

	print( NL );

	I2C_Send( buffer, bufferLen, 0 );
}

void cliFunc_i2cRecv( char* args )
{
	char* curArgs;
	char* arg1Ptr;
	char* arg2Ptr = args;

	// Buffer used after interpretting the args, will be sent to I2C functions
	// NOTE: Limited to 8 bytes currently (can be increased if necessary
	#define i2cSend_BuffLenMax 8
	uint8_t buffer[ i2cSend_BuffLenMax ];
	uint8_t bufferLen = 0;

	// No \r\n by default after the command is entered
	print( NL );
	info_msg("Sending: ");

	// Parse args until a \0 is found
	while ( bufferLen < i2cSend_BuffLenMax )
	{
		curArgs = arg2Ptr; // Use the previous 2nd arg pointer to separate the next arg from the list
		CLI_argumentIsolation( curArgs, &arg1Ptr, &arg2Ptr );

		// Stop processing args if no more are found
		if ( *arg1Ptr == '\0' )
			break;

		// If | is found, end sequence and start new one
		if ( *arg1Ptr == '|' )
		{
			print("| ");
			I2C_Send( buffer, bufferLen, 0 );
			bufferLen = 0;
			continue;
		}

		// Interpret the argument
		buffer[ bufferLen++ ] = (uint8_t)numToInt( arg1Ptr );

		// Print out the arg
		dPrint( arg1Ptr );
		print(" ");
	}

	print( NL );

	I2C_Send( buffer, bufferLen, 1 ); // Only 1 byte is ever read at a time with the ISSI chip
}

