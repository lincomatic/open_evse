/*
 * Copyright (c) 2014 Sam C. Lin <lincomatic@gmail.com>
 * This file is part of ammeter_cal

 * ammeter_cal is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.

 * ammeter_cal is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with ammeter_cal; see the file COPYING.  If not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,

 * Boston, MA 02111-1307, USA.
 */

#ifndef _AMMETER_CAL_H_
#define _AMMETER_CAL_H_

#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "serialib.h"

typedef char int8;
typedef unsigned char uint8;
typedef short int16;
typedef unsigned short uint16;
typedef int int32;
typedef unsigned int uint32;

#define ESRAPI_SOC '$'
#define ESRAPI_EOC 0xd // CR

// J1772EVSEController m_wFlags bits - saved to EEPROM
#define ECF_L2                 0x0001 // service level 2
#define ECF_DIODE_CHK_DISABLED 0x0002 // no diode check
#define ECF_VENT_REQ_DISABLED  0x0004 // no vent required state
#define ECF_GND_CHK_DISABLED   0x0008 // no chk for ground fault
#define ECF_STUCK_RELAY_CHK_DISABLED 0x0010 // no chk for stuck relay
#define ECF_AUTO_SVC_LEVEL_DISABLED  0x0020 // auto detect svc level - requires ADVPWR
// Ability set the EVSE for manual button press to start charging - GoldServe
#define ECF_AUTO_START_DISABLED 0x0040  // no auto start charging
#define ECF_SERIAL_DBG         0x0080 // enable debugging messages via serial
#define ECF_MONO_LCD           0x0100 // monochrome LCD backlight
#define ECF_GFI_TEST_DISABLED  0x0200 // no GFI self test
#define ECF_DEFAULT            0x0000

// EVSE states for m_EvseState
#define EVSE_STATE_UNKNOWN 0x00
#define EVSE_STATE_A       0x01 // vehicle state A 12V - not connected
#define EVSE_STATE_B       0x02 // vehicle state B 9V - connected, ready
#define EVSE_STATE_C       0x03 // vehicle state C 6V - charging
#define EVSE_STATE_D       0x04 // vehicle state D 3V - vent required
#define EVSE_STATE_DIODE_CHK_FAILED 0x05 // diode check failed
#define EVSE_STATE_GFCI_FAULT 0x06       // GFCI fault
#define EVSE_STATE_NO_GROUND 0x07 //bad ground
#define EVSE_STATE_STUCK_RELAY 0x08 //stuck relay
#define EVSE_STATE_SLEEPING 0xfe // waiting for timer
#define EVSE_STATE_DISABLED 0xff // disabled

class EvseRapiLink {
  serialib m_serial;
public:
  EvseRapiLink();
  ~EvseRapiLink();

  int8 Open(const char *devname) {
    return (m_serial.Open(devname,115200) == 1) ? 0 : 1;
  }
  void Close() {
    m_serial.Close();
  }

  int8 WriteCmd(const char *str) {
    return (m_serial.WriteString(str) == 1) ? 0 : 1;
  }

  int8 ReadResp(char *buf,int8 buflen,int8 nowait=0);
};

#define ESRAPI_CMDBUF_LEN 40
#define ESRAPI_RESPBUF_LEN 40
class EvseRapi {
  EvseRapiLink m_link;
  char m_cmdBuf[ESRAPI_CMDBUF_LEN];
  char m_respBuf[ESRAPI_RESPBUF_LEN];
  uint8 m_evseState;
  uint16 m_tzOffset; // minutes from UTC *CURRENTLY NOT USED*

  void appendChkSum();
  uint8 calcChkSum();
  int8 runCommand();
  int8 doAsync();
public:
  EvseRapi();

  int8 ResetEvse();

  int8 GetVersion(char *ver);

  int8 SetAmmeter(int scale,int offset);
  int8 GetAmmeter(int *scale,int *offset);
  int8 GetChargingCurrent(int *ma);
  int8 EnableAmmeterCal(int8 tf);

  void CheckAsync();

  int8 OpenLink(const char *devname) {
    return m_link.Open(devname);
  }

  void CloseLink() {
    m_link.Close();
  }

  // for testing
  void GenCmd();
};

#endif // _AMMETER_CAL_
