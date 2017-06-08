/*
 * Copyright (C) 2014 Jan Rychter
 * Modifications (C) 2015-2017 Jacob Alexander
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files ( the "Software" ), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// ----- Includes ----

// Compiler Includes
#include <Lib/ScanLib.h>

// Project Includes
#include <print.h>
#include <kll_defs.h>

// Local Includes
#include "i2c_connect.h"

#define I2C_F_DIV20			((uint8_t)0x00)
#define I2C_F_DIV22			((uint8_t)0x01)
#define I2C_F_DIV24			((uint8_t)0x02)
#define I2C_F_DIV26			((uint8_t)0x03)
#define I2C_F_DIV28			((uint8_t)0x04)
#define I2C_F_DIV30			((uint8_t)0x05)
#define I2C_F_DIV32			((uint8_t)0x09)
#define I2C_F_DIV34			((uint8_t)0x06)
#define I2C_F_DIV36			((uint8_t)0x0A)
#define I2C_F_DIV40			((uint8_t)0x07)
#define I2C_F_DIV44			((uint8_t)0x0C)
#define I2C_F_DIV48			((uint8_t)0x0D)
#define I2C_F_DIV56			((uint8_t)0x0E)
#define I2C_F_DIV64			((uint8_t)0x12)
#define I2C_F_DIV68			((uint8_t)0x0F)
#define I2C_F_DIV72			((uint8_t)0x13)
#define I2C_F_DIV80			((uint8_t)0x14)
#define I2C_F_DIV88			((uint8_t)0x15)
#define I2C_F_DIV96			((uint8_t)0x19)
#define I2C_F_DIV104			((uint8_t)0x16)
#define I2C_F_DIV112			((uint8_t)0x1A)
#define I2C_F_DIV128			((uint8_t)0x17)
#define I2C_F_DIV144			((uint8_t)0x1C)
#define I2C_F_DIV160			((uint8_t)0x1D)
#define I2C_F_DIV192			((uint8_t)0x1E)
#define I2C_F_DIV224			((uint8_t)0x22)
#define I2C_F_DIV240			((uint8_t)0x1F)
#define I2C_F_DIV256			((uint8_t)0x23)
#define I2C_F_DIV288			((uint8_t)0x24)
#define I2C_F_DIV320			((uint8_t)0x25)
#define I2C_F_DIV384			((uint8_t)0x26)
#define I2C_F_DIV480			((uint8_t)0x27)
#define I2C_F_DIV448			((uint8_t)0x2A)
#define I2C_F_DIV512			((uint8_t)0x2B)
#define I2C_F_DIV576			((uint8_t)0x2C)
#define I2C_F_DIV640			((uint8_t)0x2D)
#define I2C_F_DIV768			((uint8_t)0x2E)
#define I2C_F_DIV896			((uint8_t)0x32)
#define I2C_F_DIV960			((uint8_t)0x2F)
#define I2C_F_DIV1024			((uint8_t)0x33)
#define I2C_F_DIV1152			((uint8_t)0x34)
#define I2C_F_DIV1280			((uint8_t)0x35)
#define I2C_F_DIV1536			((uint8_t)0x36)
#define I2C_F_DIV1920			((uint8_t)0x37)
#define I2C_F_DIV1792			((uint8_t)0x3A)
#define I2C_F_DIV2048			((uint8_t)0x3B)
#define I2C_F_DIV2304			((uint8_t)0x3C)
#define I2C_F_DIV2560			((uint8_t)0x3D)
#define I2C_F_DIV3072			((uint8_t)0x3E)
#define I2C_F_DIV3840			((uint8_t)0x3F)

#define I2C_F_DIV52  ((uint8_t)0x43)
#define I2C_F_DIV60  ((uint8_t)0x45)
#define I2C_F_DIV136 ((uint8_t)0x4F)
#define I2C_F_DIV176 ((uint8_t)0x55)
#define I2C_F_DIV352 ((uint8_t)0x95)
// ----- Variables -----

volatile I2C_Channel i2c_channels[ISSI_I2C_Buses_define];

int32_t abs(int32_t val){
  if(val < 0){
      return -val;
    }
  return val;
}

uint32_t i2c_offset[] = {
  0x0,    // Bus 0
  0x1000, // Bus 1
};



// ----- Functions -----

void i2c_setup()
{
  for ( uint8_t ch = 0; ch < ISSI_I2C_Buses_define; ch++ )
  {
      uint32_t busFreq = 48000000;
      uint32_t i2cFreq = 400000;
    volatile uint8_t *I2C_F   = (uint8_t*)(&I2C0_F) + i2c_offset[ch];
    volatile uint8_t *I2C_FLT = (uint8_t*)(&I2C0_FLT) + i2c_offset[ch];
    volatile uint8_t *I2C_C1  = (uint8_t*)(&I2C0_C1) + i2c_offset[ch];
    volatile uint8_t *I2C_C2  = (uint8_t*)(&I2C0_C2) + i2c_offset[ch];

    switch ( ch )
    {
    case 0:
      // Enable I2C internal clock
      SIM_SCGC4 |= SIM_SCGC4_I2C0; // Bus 0

      // External pull-up resistor
      PORTB_PCR0 = PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(2);
      PORTB_PCR1 = PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(2);

      break;

#if defined(_mk20dx256_) || defined(_mk20dx256vlh7_)
    case 1:
      // Enable I2C internal clock
      SIM_SCGC4 |= SIM_SCGC4_I2C1; // Bus 1

      // External pull-up resistor
      PORTC_PCR10 = PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(2);
      PORTC_PCR11 = PORT_PCR_ODE | PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(2);

      break;
#endif
    }

    // SCL Frequency Divider
#if ISSI_Chip_31FL3731_define == 1
    // 0x40 -> 36 MHz / (4 * 28) = 321.428 kBaud
    // 0x80 => mul(4)
    // 0x00 => ICL(28)
    *I2C_F = 0x84;
    *I2C_FLT = 0x03;
#elif ISSI_Chip_31FL3732_define == 1 || ISSI_Chip_31FL3733_define
    /*
    // Works
    // 0x40 -> 36 MHz / (4 * 28) = 321.428 kBaud
    // 0x80 => mul(4)
    // 0x00 => ICL(28)
    *I2C_F = 0x84;
    *I2C_FLT = 0x02;

    // Also works, 80 fps, no errors (flicker?)
    // 0x40 -> 36 MHz / (1 * 44) = 818.181 kBaud
    // 0x80 => mul(1)
    // 0x0C => ICL(44)
    *I2C_F = 0x0C;
    *I2C_FLT = 0x02; // Glitch protection, reduce if you see bus errors
    */

    // Also works, 86 fps, no errors, using frame delay of 50 us
    // 0x40 -> 36 MHz / (2 * 20) = 900 kBaud
    // 0x40 => mul(2)
    // 0x00 => ICL(20)
    *I2C_F = 0x40;
    *I2C_FLT = 0x02;
#endif
    *I2C_F = 0x84;
    *I2C_FLT = 0x02;

    /*int32_t i2c_div_num[] =
        {20,22,24,26,28,30,32,34,36,40,44,48,52,56,60,64,68,72,
         80,88,96,104,112,128,136,144,160,176,192,224,240,256,288,
         320,352,384,448,480,512,576,640,768,896,960,1024,1152,
         1280,1536,1920,1792,2048,2304,2560,3072,3840};

    uint8_t i2c_div_ratio[] =
        {I2C_F_DIV20,I2C_F_DIV22,I2C_F_DIV24,I2C_F_DIV26,
         I2C_F_DIV28,I2C_F_DIV30,I2C_F_DIV32,I2C_F_DIV34,
         I2C_F_DIV36,I2C_F_DIV40,I2C_F_DIV44,I2C_F_DIV48,
         I2C_F_DIV52,I2C_F_DIV56,I2C_F_DIV60,I2C_F_DIV64,
         I2C_F_DIV68,I2C_F_DIV72,I2C_F_DIV80,I2C_F_DIV88,
         I2C_F_DIV96,I2C_F_DIV104,I2C_F_DIV112,I2C_F_DIV128,
         I2C_F_DIV136,I2C_F_DIV144,I2C_F_DIV160,I2C_F_DIV176,
         I2C_F_DIV192,I2C_F_DIV224,I2C_F_DIV240,I2C_F_DIV256,
         I2C_F_DIV288,I2C_F_DIV320,I2C_F_DIV352,I2C_F_DIV384,
         I2C_F_DIV448,I2C_F_DIV480,I2C_F_DIV512,I2C_F_DIV576,
         I2C_F_DIV640,I2C_F_DIV768,I2C_F_DIV896,I2C_F_DIV960,
         I2C_F_DIV1024,I2C_F_DIV1152,I2C_F_DIV1280,I2C_F_DIV1536,
         I2C_F_DIV1920,I2C_F_DIV1792,I2C_F_DIV2048,I2C_F_DIV2304,
         I2C_F_DIV2560,I2C_F_DIV3072,I2C_F_DIV3840};

    int32_t target_div = ((busFreq/1000)<<8)/(i2cFreq/1000);
    unsigned int idx;
    int i2 = abs(-1);
    // find closest divide ratio
    for(idx=0; idx < sizeof(i2c_div_num)/sizeof(i2c_div_num[0]) && (i2c_div_num[idx]<<8) <= target_div; idx++);
    if(idx && abs(target_div-(i2c_div_num[idx-1]<<8)) <= abs(target_div-(i2c_div_num[idx]<<8))) idx--;
    // Set divider to set rate
    *I2C_F = i2c_div_ratio[idx];
    // save current rate setting
    //i2c->currentRate = busFreq/i2c_div_num[idx];

    // Set filter
    if(busFreq >= 48000000)
        *I2C_FLT = 4;
    else
        *I2C_FLT = busFreq/12000000;*/

    *I2C_C1 = I2C_C1_IICEN;
    *I2C_C2 = I2C_C2_HDRS; // High drive select

    // FIXME Forcing settings
    //*I2C_F = 0x85;
    *I2C_F = 0x40;
    *I2C_FLT = 0x04;
    //*I2C_C1 = 0x80;
    //*I2C_C2 = 0x20;

    switch ( ch )
    {
    case 0:
      // Enable I2C Interrupt
      NVIC_SET_PRIORITY( IRQ_I2C0, 64 );
      NVIC_ENABLE_IRQ( IRQ_I2C0 );

      break;

#if defined(_mk20dx256_) || defined(_mk20dx256vlh7_)
    case 1:

      // Enable I2C Interrupt
      NVIC_ENABLE_IRQ( IRQ_I2C1 );
      break;
#endif
    }

    volatile I2C_Channel *channel = &( i2c_channels[ch] );
    channel->method = I2C_BLOCKING;
  }
}

void i2c_reset()
{
  // Cleanup after an I2C error
  for ( uint8_t ch = 0; ch < ISSI_I2C_Buses_define; ch++ )
  {
    volatile I2C_Channel *channel = &( i2c_channels[ch] );
    channel->status = I2C_AVAILABLE;
  }

  i2c_setup();
}

uint8_t i2c_busy( uint8_t ch )
{
  volatile I2C_Channel *channel = &( i2c_channels[ch] );
  if ( channel->status == I2C_BUSY )
  {
    return 1;
  }

  return 0;
}

uint8_t i2c_any_busy()
{
  for ( uint8_t ch = 0; ch < ISSI_I2C_Buses_define; ch++ )
  {
    if ( i2c_busy( ch ) )
      return 1;
  }
  return 0;
}

void i2c_cleanup(uint8_t ch){
  volatile I2C_Channel *channel = &( i2c_channels[ch] );
  volatile uint8_t *I2C_C1  = (uint8_t*)(&I2C0_C1) + i2c_offset[ch];

  // Generate STOP and disable further interrupts.
  *I2C_C1 &= ~( I2C_C1_MST | I2C_C1_IICIE );
  channel->status = I2C_AVAILABLE;
  return;
}

uint8_t i2c_get_read_valid(uint8_t ch){
  volatile I2C_Channel *channel = &( i2c_channels[ch] );
  return channel->read_valid;
}

uint8_t get_isr_happened(){
  volatile I2C_Channel *channel = &( i2c_channels[0] );
  return channel->isr_happened;
}

// These are here for readability and correspond to bit 0 of the address byte.
#define I2C_WRITING 0
#define I2C_READING 1

int32_t i2c_send_sequence(
  uint8_t ch,
  uint16_t *sequence,
  uint32_t sequence_length,
  uint8_t *received_data,
  void ( *callback_fn )( void* ),
  void *user_data
) {
  volatile I2C_Channel *channel = &( i2c_channels[ch] );

  volatile uint8_t *I2C_C1  = (uint8_t*)(&I2C0_C1) + i2c_offset[ch];
  volatile uint8_t *I2C_S   = (uint8_t*)(&I2C0_S) + i2c_offset[ch];
  volatile uint8_t *I2C_D   = (uint8_t*)(&I2C0_D) + i2c_offset[ch];

  int32_t result = 0;
  uint8_t status;

  if ( channel->status == I2C_BUSY )
  {
    if(channel->method == I2C_POLLING){
      // Disable interrupts
      *I2C_C1 &= ~( I2C_C1_IICIE );

      if(!(*I2C_S & I2C_S_IICIF)){
        return -1;
      }

      i2c_isr( ch );


    } // POLLING

    return -1;
  }

  // Debug
  /*dbug_msg("Sending I2C: ");
  for ( uint8_t c = 0; c < sequence_length; c++ )
  {
      if(sequence[c] == I2C_READ) print("Din");
      else if(sequence[c] == I2C_RESTART) print("SR");
      else printHex( sequence[c] );
      print(" ");
  }
  print(NL);*/


  channel->sequence = sequence;
  channel->sequence_end = sequence + sequence_length;
  channel->received_data = received_data;
  channel->status = I2C_BUSY;
  channel->txrx = I2C_WRITING;
  channel->callback_fn = callback_fn;
  channel->user_data = user_data;
  channel->read_valid = I2C_READ_INVALID;
  channel->isr_happened = 0;

  // reads_ahead does not need to be initialized

  // Acknowledge the interrupt request, just in case
  *I2C_S |= I2C_S_IICIF;
  *I2C_C1 = ( I2C_C1_IICEN | I2C_C1_IICIE );

  // Generate a start condition and prepare for transmitting.
  *I2C_C1 |= ( I2C_C1_MST | I2C_C1_TX );

  status = *I2C_S;
  if ( status & I2C_S_ARBL )
  {
    warn_print("Arbitration lost while sending");
    result = -1;
    goto i2c_send_sequence_cleanup;
  }

  // Write the first (address) byte.
  *I2C_D = *channel->sequence++;

  if(channel->method == I2C_ISR) return result;

  if(channel->method == I2C_BLOCKING){
    // Disable interrupts
    *I2C_C1 &= ~( I2C_C1_IICIE );
    while(channel->status != I2C_AVAILABLE && channel->status != I2C_ERROR){

      uint8_t blocking_wait_us = 2;
      uint16_t blocking_loops = 0;
      while(!(*I2C_S & I2C_S_IICIF)){
        delayMicroseconds(blocking_wait_us);

        if((blocking_loops * blocking_wait_us) > I2C_BLOCKING_TIMEOUT_US){
          warn_print("Blocking i2c timed out after ");
          printInt16((uint16_t)I2C_BLOCKING_TIMEOUT_US);
          print(" us");
          print(NL);

          goto i2c_send_sequence_cleanup;
        }

        blocking_loops++;
      }

      // The actual sending/receiving logic
      i2c_isr( ch );
    }
  } // BLOCKING

  // Everything is OK.
  return result;




i2c_send_sequence_cleanup:
  *I2C_C1 &= ~( I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TX );
  channel->status = I2C_ERROR;
  return result;
}

void i2c_isr( uint8_t ch )
{
  volatile I2C_Channel* channel = &i2c_channels[ch];

  volatile uint8_t *I2C_C1  = (uint8_t*)(&I2C0_C1) + i2c_offset[ch];
  volatile uint8_t *I2C_S   = (uint8_t*)(&I2C0_S) + i2c_offset[ch];
  volatile uint8_t *I2C_D   = (uint8_t*)(&I2C0_D) + i2c_offset[ch];

  uint16_t element;
  uint8_t status;

  status = *I2C_S;

  // Acknowledge the interrupt request
  *I2C_S |= I2C_S_IICIF;

  // Arbitration problem
  if ( status & I2C_S_ARBL )
  {
    warn_msg("Arbitration error. Bus: ");
    printHex( ch );
    print(NL);

    *I2C_S |= I2C_S_ARBL;
    goto i2c_isr_error;
  }

  if ( channel->txrx == I2C_READING )
  {
    switch( channel->reads_ahead )
    {
    // All the reads in the sequence have been processed ( but note that the final data register read still needs to
    // be done below! Now, the next thing is either a restart or the end of a sequence. In any case, we need to
    // switch to TX mode, either to generate a repeated start condition, or to avoid triggering another I2C read
    // when reading the contents of the data register.
    case 0:
      *I2C_C1 |= I2C_C1_TX;

      // Perform the final data register read now that it's safe to do so.
      *channel->received_data++ = *I2C_D;

      channel->read_valid = I2C_READ_VALID;
      // Do we have a repeated start?
      if ( ( channel->sequence < channel->sequence_end ) && ( *channel->sequence == I2C_RESTART ) )
      {

        // Generate a repeated start condition.
        *I2C_C1 |= I2C_C1_RSTA;

        // A restart is processed immediately, so we need to get a new element from our sequence. This is safe, because
        // a sequence cannot end with a RESTART: there has to be something after it. Note that the only thing that can
        // come after a restart is an address write.
        channel->txrx = I2C_WRITING;
        channel->sequence++;
        element = *channel->sequence;
        *I2C_D = element;
      }
      else
      {
        goto i2c_isr_stop;
      }
      break;

    case 1:
      // do not ACK the final read
      *I2C_C1 |= I2C_C1_TXAK;
      *channel->received_data++ = *I2C_D;
      break;

    default:
      *channel->received_data++ = *I2C_D;
      break;
    }

    channel->reads_ahead--;

  }
  // channel->txrx == I2C_WRITING
  else
  {
    // First, check if we are at the end of a sequence.
    if ( channel->sequence == channel->sequence_end )
      goto i2c_isr_stop;

    // We received a NACK. Generate a STOP condition and abort.
    if ( status & I2C_S_RXAK )
    {
      //warn_print("NACK Received");
      //goto i2c_isr_error;
    }

    // check next thing in our sequence
    element = *channel->sequence;

    // Do we have a restart? If so, generate repeated start and make sure TX is on.
    if ( element == I2C_RESTART )
    {
      *I2C_C1 |= I2C_C1_RSTA | I2C_C1_TX;

      // A restart is processed immediately, so we need to get a new element from our sequence.
      // This is safe, because a sequence cannot end with a RESTART: there has to be something after it.
      channel->sequence++;
      element = *channel->sequence;

      // Note that the only thing that can come after a restart is a write.
      *I2C_D = element;
    }
    else
    {
      if ( element == I2C_READ ) {
        channel->txrx = I2C_READING;
        // How many reads do we have ahead of us ( not including this one )?
        // For reads we need to know the segment length to correctly plan NACK transmissions.
        // We already know about one read
        channel->reads_ahead = 1;
        while (
          (  ( channel->sequence + channel->reads_ahead ) < channel->sequence_end ) &&
          ( *( channel->sequence + channel->reads_ahead ) == I2C_READ )
        ) {
          channel->reads_ahead++;
        }

        // Switch to RX mode.
        *I2C_C1 &= ~I2C_C1_TX;

        // do not ACK the final read
        if ( channel->reads_ahead == 1 )
        {
          *I2C_C1 |= I2C_C1_TXAK;
        }
        // ACK all but the final read
        else
        {
          *I2C_C1 &= ~( I2C_C1_TXAK );
        }

        // Dummy read comes first, note that this is not valid data!
        // This only triggers a read, actual data will come in the next interrupt call and overwrite this.
        // This is why we do not increment the received_data pointer.
        *channel->received_data = *I2C_D;
        channel->reads_ahead--;
      }
      // Not a restart, not a read, must be a write.
      else
      {
        *I2C_D = element;
      }
    }
  }

  channel->sequence++;
  return;

i2c_isr_stop:
  // Generate STOP ( set MST=0 ), switch to RX mode, and disable further interrupts.
  *I2C_C1 &= ~( I2C_C1_MST | I2C_C1_IICIE | I2C_C1_TXAK );
  channel->status = I2C_AVAILABLE;

  // Call the user-supplied callback function upon successful completion (if it exists).
  if ( channel->callback_fn )
  {
    // Delay 10 microseconds before starting linked function
    // TODO, is this chip dependent? -HaaTa
    delayMicroseconds(10);
    ( *channel->callback_fn )( channel->user_data );
  }
  return;

i2c_isr_error:
  // Generate STOP and disable further interrupts.
  *I2C_C1 &= ~( I2C_C1_MST | I2C_C1_IICIE );
  channel->status = I2C_ERROR;
  return;
}

void i2c0_isr()
{
  i2c_isr( 0 );
}

void i2c1_isr()
{
  i2c_isr( 1 );
}
