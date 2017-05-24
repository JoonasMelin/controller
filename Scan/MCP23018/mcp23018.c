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

// I2C address A2 A1 A0 are 000 in ergodox
// 0 1 0 0 A2 A1 A0 RW  from [1] section 1.4.2 Figure 1.6
// 0 1 0 0 is always the same for all MCP23018
// A2 A1 A0 are set to 0 by bringin the ADDR pin to low (hard wired)
// R = 1 / W = 0
#define TWI_MCP23018_CONTROLBYTEREAD  0b01000001 // 0 1 0 0 A2 A1 A0 RW  from [1] section 1.4.2 Figure 1.6
#define TWI_MCP23018_CONTROLBYTEWRITE 0b01000000 // 0 1 0 0 A2 A1 A0 RW  from [1] section 1.4.2 Figure 1.6

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


/*void MCP23018::writeToRegister(uint8_t address, uint8_t data)
{
        Wire.beginTransmission(i2c_address);
        Wire.send(address);
        Wire.send(data);
        Wire.endTransmission();

        readFromRegister(address);
}

void MCP23018::writePairToRegister(uint8_t address, uint8_t first_data, uint8_t second_data)
{
        Wire.beginTransmission(i2c_address);
        Wire.send(address);
        Wire.send(first_data);
        Wire.send(second_data);
        Wire.endTransmission();

        readFromRegister(address);
}

uint8_t MCP23018::readFromRegister(uint8_t address)
{
        uint8_t received_data = 0;

        // Establish connection, select receipt address
        Wire.beginTransmission(i2c_address);
        Wire.send(address);
        Wire.endTransmission();

        // Request one data byte
        Wire.requestFrom(i2c_address, (uint8_t)1);


        // Fill variables when ready
        if(Wire.available())
        {
                received_data = Wire.receive();
}


        return received_data;
}

void MCP23018::setBitInRegister(const uint8_t address_bit[], bool bitState)
{
        uint8_t temp;
        uint8_t address = address_bit[0];
        uint8_t bit = address_bit[1];

        // Use bitState to decide which masking to use (to 1 or to 0)
        if(bitState)
                temp = readFromRegister(address) | (1 << bit);
        else
                temp = readFromRegister(address) & ~(1 << bit);

        writeToRegister(address, temp);
}

void MCP23018::setBitGroupInRegister(const uint8_t address, const uint8_t data, const uint8_t mask)
{
#ifdef DEBUG
    Serial.print("()setBitGroupInRegister(");
    Serial.print(address,HEX);
    Serial.print(",");
    Serial.print(data,HEX);
    Serial.print(",");
    Serial.print(mask,HEX);
    Serial.println(")");
#endif

        uint8_t temp = readFromRegister(address) & ( mask ^ 0xff) | ( data & mask );
        writeToRegister(address, temp);
}

MCP23018::MCP23018(uint8_t _address)
{
        i2c_address = ( _address & B111 ) | I2C_MCP23018;
}

void MCP23018::begin(void)
{
        // Set all pins to outputs
        writePairToRegister(IODIRA,0,0);
}

void MCP23018::SetPullups(uint8_t _a, uint8_t _b)
{
        writePairToRegister(GPPUA,_a,_b);
}

void MCP23018::SetPortA(uint8_t _data)
{
        writeToRegister(GPIOA,_data);
}

void MCP23018::SetPortB(uint8_t _data)
{
        writeToRegister(GPIOB,_data);
}

void MCP23018::SetPorts(uint8_t _a, uint8_t _b)
{
        writePairToRegister(GPIOA,_a,_b);
}

uint8_t MCP23018::GetPortA(void)
{
        return readFromRegister(GPIOA);
}

uint8_t MCP23018::GetPortB(void)
{
        return readFromRegister(GPIOB);
}

uint8_t MCP23018::GetLatchPortA(void)
{
        return readFromRegister(OLATA);
}

uint8_t MCP23018::GetLatchPortB(void)
{
        return readFromRegister(OLATB);
}

void MCP23018::SetPortB_bits(uint8_t _data, uint8_t _mask)
{
        setBitGroupInRegister(OLATB,_data,_mask);
}*/

void Mcp_setup()
{
  i2c_setup();
    //LED_setup();
    //Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
    /*pinMode(LED_BUILTIN,OUTPUT);    // LED
    digitalWrite(LED_BUILTIN,LOW);  // LED off
    pinMode(12,INPUT_PULLUP);       // Control for Send
    pinMode(11,INPUT_PULLUP);       // Control for Receive

    // Setup for Master mode, pins 18/19, external pullups, 400kHz, 200ms default timeout
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
    Wire.setDefaultTimeout(200000); // 200ms

    // Data init
    memset(databuf, 0, sizeof(databuf));
    count = 0;

    Serial.begin(115200);*/
}

void Mcp_read_pins()
{
  delayMicroseconds( 10000 );
  loopNo++;
  uint16_t addr = (0x20);
  uint16_t reg = 0x00;
  uint16_t val = 0xFF;
  uint16_t writeData[] = { addr, reg, val };
  uint16_t bus[16];

  // Setup page

  // Write register
  dbug_msg("Sending.. ");
  while ( i2c_send( bus, writeData, sizeof( writeData ) / 2 ) == -1 )
    delayMicroseconds( 1000 );
  dbug_msg("Send OK ");

  /*if(loopNo > 400){

    uint8_t writeData[] = {(0x20 << 1), 0x00, 0xFF};

    I2C_Send(writeData, 3, 0);
    delay(10);


    setReceiving(1);
    uint8_t writeDataRq[] = {((0x20 << 1) | 1), 0x13};
    I2C_Send(writeDataRq, 2, 1);

    uint8_t seq = 0;
    uint8_t rxBuffer = I2C_GetRxBuffer(seq);
    print(NL);
    dbug_msg("Rx buffer: ");
    printHex(rxBuffer);
    print("|");
    printHex(I2C_GetRxBuffer(1));
    print("|");
    printHex(I2C_GetRxBuffer(2));
    print(NL);
    setReceiving(0);*/


  //I2C_clearTxBuffer();
}

void loop()
{

}


