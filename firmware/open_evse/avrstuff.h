// -*- C++ -*-
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

#pragma once

#include <avr/io.h>
#include <avr/interrupt.h>
#if defined(ARDUINO) && (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h" // shouldn't need this but arduino sometimes messes up and puts inside an #ifdef
#endif // ARDUINO

#ifdef __cplusplus
class CriticalSection {
  uint8_t sreg;
public:
  CriticalSection() { start(); }
  void start() { sreg = SREG; cli(); }
  void end() { SREG = sreg; }
};

class AutoCriticalSection {
  uint8_t sreg;
public:
  AutoCriticalSection() { sreg = SREG; cli(); }
  ~AutoCriticalSection() { SREG = sreg; }
};

#endif // __cplusplus


// n.b. NONE OF THE PIN FUNCTIONS BELOW HANDLE PORTS > 0x100.. must be wrapped
// in critical section.. see Marlin fastio.h for details


//
// digital pin macros .. ugly, but no RAM usage
//
// don't use _xxx() versions
#define _DPIN_READ(reg,idx) (PIN ## reg & (1 << idx))
#define _DPIN_SET(reg,idx) {PORT ## reg |= (1<<idx);}
#define _DPIN_CLR(reg,idx) {PORT ## reg &= ~(1<<idx);}
#define _DPIN_WRITE(reg,idx,val) {if (val) DPIN_SET(reg,idx); else DPIN_CLR(reg,idx); }
#define _DPIN_MODE_INPUT(reg,idx) {DDR ## reg &= ~(1<<idx)}
#define _DPIN_MODE_PULLUP(reg,idx) { DPIN_SET(reg,idx);DPIN_MODE_INPUT(reg,idx);}
#define _DPIN_MODE_OUTPUT(reg,idx) {DDR ## reg |= (1<<idx);}

// use these
#define DPIN_READ(reg,idx) _DPIN_READ(reg,idx)
#define DPIN_SET(reg,idx) _DPIN_SET(reg,idx)
#define DPIN_CLR(reg,idx) _DPIN_CLR(reg,idx) 
#define DPIN_WRITE(reg,idx,val) _DPIN_WRITE(reg,idx,val) 
 // for (PINx > (uint8_t *)0x100 *must* use DPIN_WRITE_ATOMIC() instead of DPIN_WRITE()
#define DPIN_WRITE_ATOMIC(reg,idx,val) {AutoCriticalSection acs;_DPIN_WRITE(reg,idx,val)}
#define DPIN_MODE_INPUT(reg,idx) _DPIN_MODE_INPUT(reg,idx)
#define DPIN_MODE_PULLUP(reg,idx) _DPIN_MODE_PULLUP(reg,idx)
#define DPIN_MODE_OUTPUT(reg,idx) _DPIN_MODE_OUTPUT(reg,idx)
/*
example
#define LED_PORT B
#define LED_IDX  5

  DPIN_MODE_OUTPUT(B,LED_IDX);
  DPIN_SET(LED_PORT,LED_IDX);
  DPIN_CLR(LED_PORT,LED_IDX);
 */

//
// end digital pin macros
//


//
// pin macros .. ugly, but no RAM usage
//

#ifdef __cplusplus

//
// begin digitalPin class
// using this class beautifies the code, but wastes 3 bytes per pin
//
class DigitalPin {
  volatile uint8_t* reg;
  uint8_t bit;
  
public:
  enum PinMode { INP,INP_PU,OUT };

  DigitalPin() {}
  DigitalPin(volatile uint8_t* _reg,uint8_t idx,PinMode _mode) {
    init(_reg,idx,_mode);
  }

  void init(volatile uint8_t* _reg,uint8_t idx,PinMode _mode);

  void mode(PinMode mode);
  uint8_t read() {
#ifdef HIGH
    return (*pin() & bit) ? HIGH : LOW;
#else
    return *pin() & bit;
#endif
  }
  void write(uint8_t state) {
    if (state) *port() |= bit;
    else *port() &= ~bit;
  }
  // for (PINx > (uint8_t *)0x100 *must* use writeAtomic() instead of write()
  void writeAtomic(uint8_t state) {
    AutoCriticalSection acs;
    write(state);
  }  

  volatile uint8_t* pin() { return reg; }
  volatile uint8_t* ddr() { return reg+1; }
  volatile uint8_t* port() { return reg+2; }

};


//
// begin AdcPin class
// using this class beautifies the code, but wastes 1 byte per pin
//
class AdcPin {
  static uint8_t refMode;
  uint8_t channel;
  
public:
  enum PinMode { INP,INP_PU,OUT };

  AdcPin(uint8_t _adcNum) {
    init(_adcNum);
  }

  void init(uint8_t _adcNum);
  uint16_t read();

  static void referenceMode(uint8_t mode) {
    refMode = mode;
  }
};

//  why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html
// don't call the _xxx() version directly
#define _DIGITAL_PIN(name,reg,idx,mode) name(&PIN ## reg,idx,mode)
#define DIGITAL_PIN(name,reg,idx,mode) _DIGITAL_PIN(name,reg,idx,mode)

/*
examples:
// pin B6: reg=B,pin=6
DigitalPin DIGITAL_PIN(mypin,B,6,DigitalPin::INP);
mypin.write(HIGH);
uint8_t val = mypin.read();

DigitalPin mypin2(&PINB,6,DigitalPin::INP);

class Led {
  DigitalPin pin;
public:
  Led() : DIGITAL_PIN(pin,B,5,DigitalPin::OUT) {}
  void set(uint8_t onoff) { write(onoff); {
};
*/

//
// end digital pin class
//

#endif // __cplusplus
