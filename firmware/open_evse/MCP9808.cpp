/**************************************************************************/
/*! 
 * 2016 hacked from Adafruit_MCP9808.cpp by Sam C. Lin
 
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
#include "open_evse.h"

#ifdef MCP9808_IS_ON_I2C

inline uint8_t wirerecv(void) {
#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}

inline void wiresend(uint8_t x) {
#if ARDUINO >= 100
  Wire.write((uint8_t)x);
#else
  Wire.send(x);
#endif
}

int8_t MCP9808::begin()
{
  if ((read16(MCP9808_REG_MANUF_ID) == 0x0054) &&
      (read16(MCP9808_REG_DEVICE_ID) == 0x0400)) isPresent = 1;
  else isPresent = 0;
  return isPresent;
}


int16_t MCP9808::read16(uint8_t reg) {
  int16_t val;
  Wire.beginTransmission(MCP9808_ADDRESS);
  wiresend(reg);
  Wire.endTransmission();
  
  Wire.requestFrom((uint8_t)MCP9808_ADDRESS, (uint8_t)2);
  val = wirerecv();
  val <<= 8;
  val |= wirerecv();  

  return val;  
}

// return C*10
int16_t MCP9808::readAmbient()
{
  if (isPresent) {
    int16_t temp = read16(MCP9808_REG_AMBIENT_TEMP) & 0x1FFF;
    if (temp & 0x1000) temp |= 0xF000; // sign extend negative number
    return (temp * 10) / 16;
  }
  else {
    return TEMPERATURE_NOT_INSTALLED;
  }
}
#endif // MCP9808_IS_ON_I2C
