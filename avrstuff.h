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


// n.b. NONE OF THE PIN FUNCTIONS BELOW HANDLE PORTS > 0x100.. must be wrapped
// in critical section.. see Marlin fastio.h for details


//
// digital pin macros .. ugly, but no RAM usage
//
// don't use _xxx() versions
#define _DPIN_READ(port,idx) (PIN ## port & (1 << idx))
#define _DPIN_SET(port,idx) {PORT ## port |= (1<<idx);}
#define _DPIN_CLR(port,idx) {PORT ## port &= ~(1<<idx);}
#define _DPIN_WRITE(port,idx,val) {if (val) DPIN_SET(port,idx); else DPIN_CLR(port,idx); }
#define _DPIN_MODE_INPUT(port,idx) {DDR ## port &= ~(1<<idx)}
#define _DPIN_MODE_PULLUP(port,idx) { DPIN_SET(port,idx);DPIN_MODE_INPUT(port,idx);}
#define _DPIN_MODE_OUTPUT(port,idx) {DDR ## port |= (1<<idx);}

// use these
#define DPIN_READ(port,idx) _DPIN_READ(port,idx)
#define DPIN_SET(port,idx) _DPIN_SET(port,idx)
#define DPIN_CLR(port,idx) _DPIN_CLR(port,idx) 
#define DPIN_WRITE(port,idx,val) _DPIN_WRITE(port,idx,val) 
#define DPIN_MODE_INPUT(port,idx) _DPIN_MODE_INPUT(port,idx)
#define DPIN_MODE_PULLUP(port,idx) _DPIN_MODE_PULLUP(port,idx)
#define DPIN_MODE_OUTPUT(port,idx) _DPIN_MODE_OUTPUT(port,idx)
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


//
// begin digitalPin class
// using this class beautifies the code, but wastes 3 bytes per pin
//
class DigitalPin {
  volatile uint8_t* pinReg;
  uint8_t bit;
  
public:
  enum PinMode { INP,INP_PU,OUT };

  DigitalPin() {}
  DigitalPin(volatile uint8_t* _pinReg,uint8_t idx,PinMode _mode) {
    init(_pinReg,idx,_mode);
  }

  void init(volatile uint8_t* _pinReg,uint8_t idx,PinMode _mode);

  void mode(PinMode mode);
  uint8_t read() {
    return *pin() & bit;
  }
  void write(uint8_t state) {
    if (state) *port() |= bit;
    else *port() &= ~bit;
  }

  volatile uint8_t* pin() { return pinReg; }
  volatile uint8_t* ddr() { return pinReg+1; }
  volatile uint8_t* port() { return pinReg+2; }

};

//  why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html
// don't call the _xxx() version directly
#define _DIGITAL_PIN(name,port,idx,mode) name(&PIN ## port,idx,mode)
#define DIGITAL_PIN(name,port,idx,mode) _DIGITAL_PIN(name,port,idx,mode)

/*
examples:
// pin B6: port=B,pin=6
DigitalPin DIGITAL_PIN(mypin,B,6,DigitalPin::INP);
mypin.write(HIGH);
uint8_t val = mypin.read();

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
