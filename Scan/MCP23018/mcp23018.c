#include "mcp23018.h"
#include "i2c_connect.h"
#include <Lib/ScanLib.h>
#include <cli.h>
#include <print.h>

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




void Mcp_setup()
{
    i2c_setup();

}

void Mcp_read_register(uint16_t address, uint16_t register_addr, uint8_t *data){
  // The sequence that defines what is going to be done
  // Send address, send the register to be read, repeated start, address with
  // write bit, request data byte
  uint16_t read_data_sequence[] = { (address), register_addr,
                          I2C_RESTART, (address | 0x1), I2C_READ};

  uint8_t wait_step_us = 2;

  uint16_t wait_loops_done = 0;
  while(i2c_read(I2C_BUS_NO, read_data_sequence, 5, data) == -1){
      // Checking if things are not progressing as they should
      if((wait_loops_done * wait_step_us) > I2C_TIMEOUT_US){
          warn_print("I2C data request timeout, resetting the bus.");
          //i2c_reset();
          break;
        }

      delayMicroseconds(wait_step_us);
      wait_loops_done++;
  }

  /*while(get_isr_happened() == 0){

    if((wait_loops_done * wait_step_us) > I2C_TIMEOUT_US){
        print("X");
        //i2c_reset();
        i2c_cleanup();
        return;
      }

      wait_loops_done++;
      delayMicroseconds(wait_step_us);
  }*/

  while((i2c_get_read_valid(I2C_BUS_NO) == I2C_READ_INVALID)){
      //dbug_print("Waiting2..");
      // Checking if things are not progressing as they should
      if((wait_loops_done * wait_step_us) > I2C_TIMEOUT_US){
          //warn_print("I2C receive timeout, resetting the bus.");
          //i2c_reset();
          //i2c_setup();
          i2c_cleanup();
          //printInt8(get_isr_happened());
          break;
        }

      delayMicroseconds(wait_step_us);
      wait_loops_done++;
    }

}

void Mcp_write_register(){

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
      //print("|");
      Mcp_read_register(addr, 0x12, &rcv);

      /*if(i2c_get_read_valid(0)){
        print("|");
        for(int loop = 0; loop < 4; loop++){
          printHex(rcv[loop]);
            //printHex(rcv_byte);
          print("|");
        }
        printInt8(i2c_get_read_valid(0));
        print(NL);

      }*/

      //delayMicroseconds( 7);

}
  else{
      delayMicroseconds(2000);
    }
}


