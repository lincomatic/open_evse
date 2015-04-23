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

void DigitalPin::init(volatile uint8_t* _pinReg,uint8_t idx,PinMode _mode)
{
  pinReg = _pinReg;
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
