/**************************************************************************/
/*! 
    @file     Adafruit_MCP9808.cpp
    @author   K.Townsend (Adafruit Industries)
	@license  BSD (see license.txt)
	
	I2C Driver for Microchip's MCP9808 I2C Temp sensor

	This is a library for the Adafruit MCP9808 breakout
	----> http://www.adafruit.com/products/1782
		
	Adafruit invests time and resources providing this open source code, 
	please support Adafruit and open-source hardware by purchasing 
	products from Adafruit!

	@section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#ifdef __AVR_ATtiny85__
 #include "TinyWireM.h"
 #define Wire TinyWireM
#else
 #include "./Wire.h"
#endif

inline void wiresend(uint8_t x) {
#if ARDUINO >= 100
  Wire.write((uint8_t)x);
#else
  Wire.send(x);
#endif
}

inline uint8_t wirerecv(void) {
#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}

#include "Adafruit_MCP9808.h"

/**************************************************************************/
/*! 
    @brief  Instantiates a new MCP9808 class
*/
/**************************************************************************/
Adafruit_MCP9808::Adafruit_MCP9808() {
}

/**************************************************************************/
/*! 
    @brief  Setups the HW
*/
/**************************************************************************/
boolean Adafruit_MCP9808::begin(uint8_t addr) {
  _i2caddr = addr;
  //don't need - open_evse.ino does it  Wire.begin();

  if (read16(MCP9808_REG_MANUF_ID) != 0x0054) return false;
  if (read16(MCP9808_REG_DEVICE_ID) != 0x0400) return false;

  return true;
}
 
/**************************************************************************/
/*! 
    @brief  Reads the 16-bit temperature register and returns Centigrade*10

*/
/**************************************************************************/
int16_t Adafruit_MCP9808::readTempC10( void )
{
  uint16_t t = read16(MCP9808_REG_AMBIENT_TEMP);

  uint32_t temp = ((t & 0x0FFF)*10)/16;
  if (t & 0x1000) temp -= 256;

  return (int16_t) temp;
}

/**************************************************************************/
/*! 
    @brief  Low level 16 bit read and write procedures!
*/
/**************************************************************************/

void Adafruit_MCP9808::write16(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(_i2caddr);
    wiresend((uint8_t)reg);
    wiresend(value >> 8);
    wiresend(value & 0xFF);
    Wire.endTransmission();
}

uint16_t Adafruit_MCP9808::read16(uint8_t reg) {
  uint16_t val;

  Wire.beginTransmission(_i2caddr);
  wiresend((uint8_t)reg);
  Wire.endTransmission();
  
  Wire.requestFrom((uint8_t)_i2caddr, (uint8_t)2);
  val = wirerecv();
  val <<= 8;
  val |= wirerecv();  
  return val;  
}
