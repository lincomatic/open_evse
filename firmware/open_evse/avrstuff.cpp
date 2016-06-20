// Copyright (C) 2015 Sam C. Lin
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// See LICENSE for a copy of the GNU General Public License or see
// it online at <http://www.gnu.org/licenses/>.

#include "avrstuff.h"

void DigitalPin::init(volatile uint8_t* _reg,uint8_t idx,PinMode _mode)
{
  reg = _reg;
  bit = 1 << idx;
  mode(_mode);
}

void DigitalPin::mode(PinMode mode)
{
  switch(mode) {
  case INP_PU:
    *port() |= bit;
    // fall through
  case INP:
    *ddr() &= ~bit;
    break;
  default: // OUT
    *ddr() |= bit;
  }
}

//
// AdcPin
// 
#include "wiring_private.h"
#include "pins_arduino.h"

uint8_t AdcPin::refMode = DEFAULT;

void AdcPin::init(uint8_t _adcNum)
{
  channel = _adcNum;
#if defined(analogChannelToChannel)
 #if defined(__AVR_ATmega32U4__)
  if (channel >= 18) channel -= 18; // allow for pin or channel numbers
 #endif
  channel = analogChannelToChannel(channel);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  if (channel >= 54) channel -= 54; // allow for pin or channel numbers
#elif defined(__AVR_ATmega32U4__)
  if (channel >= 18) channel -= 18; // allow for pin or channel numbers
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
  if (channel >= 24) channel -= 24; // allow for pin or channel numbers
#else
  if (channel >= 14) channel -= 14; // allow for pin or channel numbers
#endif
}

uint16_t AdcPin::read()
{
  uint8_t low, high;
  
  
#if defined(ADCSRB) && defined(MUX5)
  // the MUX5 bit of ADCSRB selects whether we're reading from channels
  // 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
  ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((channel >> 3) & 0x01) << MUX5);
#endif
  
  // set the analog reference (high two bits of ADMUX) and select the
  // channel (low 4 bits).  this also sets ADLAR (left-adjust result)
  // to 0 (the default).
#if defined(ADMUX)
  ADMUX = (refMode << 6) | (channel & 0x07);
#endif
  
  // without a delay, we seem to read from the wrong channel
  //delay(1);
  
#if defined(ADCSRA) && defined(ADCL)
  // start the conversion
  sbi(ADCSRA, ADSC);
  
  // ADSC is cleared when the conversion finishes
  while (bit_is_set(ADCSRA, ADSC));
  
  // we have to read ADCL first; doing so locks both ADCL
  // and ADCH until ADCH is read.  reading ADCL second would
  // cause the results of each conversion to be discarded,
  // as ADCL and ADCH would be locked when it completed.
  low  = ADCL;
  high = ADCH;
#else
  // we dont have an ADC, return 0
  low  = 0;
  high = 0;
#endif
  
  // combine the two bytes
  return (high << 8) | low;
}
