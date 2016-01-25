// -*- C++ -*-
/*
 * Copyright (c) 2015 Sam C. Lin
 *
 * Open EVSE Firmware
 *
 * This file is part of Open EVSE.

 * Open EVSE is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.

 * Open EVSE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with Open EVSE; see the file COPYING.  If not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef _I2CADDR_H_
#define _I2CADDR_H_
//
// I2C addresses
//
// LiquidTWI2
#define MCP23017_ADDRESS 0x20
#define MCP23008_ADDRESS 0x20

//RTClib
#define DS1307_ADDRESS 0x68

// MCP9808
#define MCP9808_ADDRESS 0x18


//RAPI_I2C
#define RAPI_I2C_LOCAL_ADDR 0x05
#define RAPI_I2C_REMOTE_ADDR 0x06


#endif // _I2CADDR_H_
