// -*- C++ -*-
/*
 * Open EVSE Firmware
 *
 * Copyright (c) 2013-2014 Sam C. Lin <lincomatic@gmail.com>
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

#if defined(ARDUINO) && (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h" // shouldn't need this but arduino sometimes messes up and puts inside an #ifdef
#endif // ARDUINO
#include "open_evse.h"

#ifdef RAPI
const char RAPI_VER[] PROGMEM = RAPIVER;


// convert 2-digit hex string to uint8_t
uint8_t htou8(const char *s)
{
  uint8_t u = 0;
  for (int i=0;i < 2;i++) {
    if (i == 1) u <<= 4;
    char c = s[i];
    if ((c >= '0') && (c <= '9')) {
      u += c - '0';
    }
    else if ((c >= 'A') && (c <= 'F')) {
      u += c - 'A' + 10;
    }
  }
  return u;
}

// convert decimal string to uint32_t
uint32_t dtou32(const char *s)
{
  uint32_t u = 0;
  while (*s) {
    u *= 10;
    u += *(s++) - '0';
  }
  return u;
}

EvseRapiProcessor::EvseRapiProcessor()
{
  echo = 0;
  reset();
}

//extern HardwareSerial Serial;
int EvseRapiProcessor::doCmd(int8_t sendstatetrans)
{
  int rc = 1;

  // chk for state transition and send async notification
  if (sendstatetrans && g_EvseController.StateTransition()) {
    sendEvseState();
  }

  int bcnt = available();
  if (bcnt) {
    for (int i=0;i < bcnt;i++) {
      char c = read();
      if (echo) write(c);

      if (c == ESRAPI_SOC) {
	buffer[0] = ESRAPI_SOC;
	bufCnt = 1;
      }
      else if (buffer[0] == ESRAPI_SOC) {
	if (bufCnt < ESRAPI_BUFLEN) {
	  if (c == ESRAPI_EOC) {
	    buffer[bufCnt++] = 0;
	    if (!tokenize()) {
	      rc = processCmd();
	    }
	    else {
	      reset();
	      response(0);
	    }
	  }
	  else {
	    buffer[bufCnt++] = c;
	  }
	}
	else { // too many chars
	  reset();
	}
      }
    }
  }

  return rc;
}

void EvseRapiProcessor::sendEvseState()
{
  sprintf(g_sTmp,"%cST %02x%c",ESRAPI_SOC,g_EvseController.GetState(),ESRAPI_EOC);
  write(g_sTmp);
}

void EvseRapiProcessor::setWifiMode(uint8_t mode)
{
  sprintf(g_sTmp,"%cWF %02x%c",ESRAPI_SOC,(int)mode,ESRAPI_EOC);
  write(g_sTmp);
}

int EvseRapiProcessor::tokenize()
{
  tokens[0] = &buffer[1];
  char *s = &buffer[2];
  tokenCnt = 1;
  uint8_t achkSum = ESRAPI_SOC + buffer[1];
  uint8_t xchkSum = ESRAPI_SOC ^ buffer[1];
  uint8_t hchkSum;
  uint8_t chktype=0; // 0=none,1=additive,2=xor
  while (*s) {
    if (*s == ' ') {
      achkSum += *s;
      xchkSum ^= *s;
      *s = '\0';
      tokens[tokenCnt++] = ++s;
    }
    else if ((*s == '*') ||// additive checksum
	     (*s == '^')) { // XOR checksum
      if (*s == '*') chktype = 1;
      else if (*s == '^') chktype = 2;
      *(s++) = '\0';
      hchkSum = htou8(s);
      break;
    }
    else {
      achkSum += *s;
      xchkSum ^= *(s++);
    }
  }
  
  return ((chktype == 0) ||
	  ((chktype == 1) && (hchkSum == achkSum)) ||
	  ((chktype == 2) && (hchkSum == xchkSum))) ? 0 : 1;
}

int EvseRapiProcessor::processCmd()
{
  UNION4B u1,u2,u3;
  int rc = 0;

  // we use bufCnt as a flag in response() to signify data to write
  bufCnt = 0;

  char *s = tokens[0];
  switch(*(s++)) { 
  case 'F': // function
    switch(*s) {
#ifdef LCD16X2
    case 'B': // LCD backlight
      g_OBD.LcdSetBacklightColor(dtou32(tokens[1]));
      break;
#endif // LCD16X2      
    case 'D': // disable EVSE
      g_EvseController.Disable();
      break;
    case 'E': // enable EVSE
      g_EvseController.Enable();
      break;
#ifdef LCD16X2
    case 'P': // print to LCD
      {
	u1.u = dtou32(tokens[1]); // x
	u2.u = dtou32(tokens[2]); // y
	// now restore the spaces that were replaced w/ nulls by tokenizing
	for (u3.i=4;u3.i < tokenCnt;u3.i++) {
	  *(tokens[u3.i]-1) = ' ';
	}
	g_OBD.LcdPrint(u1.u,u2.u,tokens[3]);
      }
      break;
#endif // LCD16X2      
    case 'R': // reset EVSE
      g_EvseController.Reboot();
      break;
    case 'S': // sleep
      g_EvseController.Sleep();
      break;
    default:
      rc = -1; // unknown
    }
    break;

  case 'S': // set parameter
    switch(*s) {
#ifdef LCD16X2
    case '0': // set LCD type
      if (tokenCnt == 2) {
#ifdef RGBLCD
	rc = g_EvseController.SetBacklightType((*tokens[1] == '0') ? BKL_TYPE_MONO : BKL_TYPE_RGB);
#endif // RGBLCD
      }
      break;
#endif // LCD16X2      
#ifdef RTC      
    case '1': // set RTC
      if (tokenCnt == 7) {
	extern void SetRTC(uint8_t y,uint8_t m,uint8_t d,uint8_t h,uint8_t mn,uint8_t s);
	SetRTC(dtou32(tokens[1]),dtou32(tokens[2]),dtou32(tokens[3]),
	       dtou32(tokens[4]),dtou32(tokens[5]),dtou32(tokens[6]));
      }
      break;
#endif // RTC      
#ifdef AMMETER
    case '2': // ammeter calibration mode
      if (tokenCnt == 2) {
	g_EvseController.EnableAmmeterCal((*tokens[1] == '1') ? 1 : 0);
      }
      break;
#ifdef TIME_LIMIT
    case '3': // set time limit
      if (tokenCnt == 2) {
	g_EvseController.SetTimeLimit(dtou32(tokens[1]));
      }
      break;
#endif // TIME_LIMIT
    case 'A':
      if (tokenCnt == 3) {
	g_EvseController.SetCurrentScaleFactor(dtou32(tokens[1]));
	g_EvseController.SetAmmeterCurrentOffset(dtou32(tokens[2]));
      }
      break;
#endif // AMMETER
    case 'C': // current capacity
      if (tokenCnt == 2) {
	rc = g_EvseController.SetCurrentCapacity(dtou32(tokens[1]),1);
      }
      break;
    case 'D': // diode check
      if (tokenCnt == 2) {
	g_EvseController.EnableDiodeCheck((*tokens[1] == '0') ? 0 : 1);
      }
      break;
    case 'E': // echo
      if (tokenCnt == 2) {
	echo = ((*tokens[1] == '0') ? 0 : 1);
      }
      break;
#ifdef GFI_SELFTEST
    case 'F': // GFI self test
      if (tokenCnt == 2) {
	g_EvseController.EnableGfiSelfTest(*tokens[1] == '0' ? 0 : 1);
      }
      break;
#endif // GFI_SELFTEST
#ifdef ADVPWR
    case 'G': // ground check
      if (tokenCnt == 2) {
	g_EvseController.EnableGndChk(*tokens[1] == '0' ? 0 : 1);
      }
      break;
#endif // ADVPWR
#ifdef CHARGE_LIMIT
    case 'H': // cHarge limit
      if (tokenCnt == 2) {
	g_EvseController.SetChargeLimit(dtou32(tokens[1]));
      }
      break;
#endif // CHARGE_LIMIT
#ifdef KWH_RECORDING
    case 'K': // set accumulated kwh
      g_WattHours_accumulated = dtou32(tokens[1]);
      eeprom_write_dword((uint32_t*)EOFS_KWH_ACCUMULATED,g_WattHours_accumulated); 
      break;
#endif //KWH_RECORDING
    case 'L': // service level
      if (tokenCnt == 2) {
      switch(*tokens[1]) {
	case '1':
	case '2':
	  g_EvseController.SetSvcLevel(*tokens[1] - '0',1);
#ifdef ADVPWR
	  g_EvseController.EnableAutoSvcLevel(0);
#endif
	  break;
#ifdef ADVPWR
	case 'A':
	  g_EvseController.EnableAutoSvcLevel(1);
	  break;
#endif // ADVPWR
	default:
	  rc = -1; // unknown
	}
      }
      break;
#ifdef VOLTMETER
    case 'M':
      if (tokenCnt == 3) {
        g_EvseController.SetVoltmeter(dtou32(tokens[1]),dtou32(tokens[2]));
      }
#endif // VOLTMETER
#ifdef ADVPWR      
    case 'R': // stuck relay check
      if (tokenCnt == 2) {
	g_EvseController.EnableStuckRelayChk(*tokens[1] == '0' ? 0 : 1);
      }
      break;
#endif // ADVPWR      
#ifdef GFI_SELFTEST
    case 'S': // GFI self-test
      if (tokenCnt == 2) {
	g_EvseController.EnableGfiSelfTest(*tokens[1] == '0' ? 0 : 1);
      }
      break;
#endif // GFI_SELFTEST   
#ifdef DELAYTIMER     
    case 'T': // timer
      if (tokenCnt == 5) {
	extern DelayTimer g_DelayTimer;
	if ((*tokens[1] == '0') && (*tokens[2] == '0') && (*tokens[3] == '0') && (*tokens[4] == '0')) {
	  g_DelayTimer.Disable();
	}
	else {
	  g_DelayTimer.SetStartTimer(dtou32(tokens[1]),dtou32(tokens[2]));
	  g_DelayTimer.SetStopTimer(dtou32(tokens[3]),dtou32(tokens[4]));
	  g_DelayTimer.Enable();
	}
      }
      break;
#endif // DELAYTIMER      
    case 'V': // vent required
      if (tokenCnt == 2) {
	g_EvseController.EnableVentReq(*tokens[1] == '0' ? 0 : 1);
      }
      break;
    }
    break;

  case 'G': // get parameter
    switch(*s) {
#ifdef TIME_LIMIT
    case '3': // get time limit
      sprintf(buffer,"%d",(int)g_EvseController.GetTimeLimit());
      bufCnt = 1; // flag response text output
      break;
#endif // TIME_LIMIT
#ifdef AMMETER
    case 'A':
      u1.i = g_EvseController.GetCurrentScaleFactor();
      u2.i = g_EvseController.GetAmmeterCurrentOffset();
      sprintf(buffer,"%d %d",u1.i,u2.i);
      bufCnt = 1; // flag response text output
      break;
#endif // AMMETER
    case 'C': // get current capacity range
      sprintf(buffer,"%d %d",MIN_CURRENT_CAPACITY,(g_EvseController.GetCurSvcLevel() == 2) ? MAX_CURRENT_CAPACITY_L2 : MAX_CURRENT_CAPACITY_L1);
      bufCnt = 1; // flag response text output
      break;
    case 'E': // get settings
      u1.u = g_EvseController.GetCurrentCapacity();
      u2.u = g_EvseController.GetFlags();
      sprintf(buffer,"%d %04x",u1.u,u2.u);
      bufCnt = 1; // flag response text output
      break;
    case 'F': // get fault counters
      u1.u = eeprom_read_byte((uint8_t*)EOFS_GFI_TRIP_CNT);
      u2.u = eeprom_read_byte((uint8_t*)EOFS_NOGND_TRIP_CNT);
      u3.u = eeprom_read_byte((uint8_t*)EOFS_STUCK_RELAY_TRIP_CNT);
      sprintf(buffer,"%x %x %x",u1.u,u2.u,u3.u);
      bufCnt = 1; // flag response text output
      break;
#if defined(AMMETER)||defined(VOLTMETER)
    case 'G':
      u1.i32 = g_EvseController.GetChargingCurrent();
      u2.i32 = (int32_t)g_EvseController.GetVoltage();
      sprintf(buffer,"%ld %ld",u1.i32,u2.i32);
      bufCnt = 1; // flag response text output
      break;
#endif // AMMETER || VOLTMETER
#ifdef CHARGE_LIMIT
    case 'H': // get cHarge limit
      sprintf(buffer,"%d",(int)g_EvseController.GetChargeLimit());
      bufCnt = 1; // flag response text output
      break;
#endif // CHARGE_LIMIT
#ifdef VOLTMETER
    case 'M':
      u1.i = g_EvseController.GetVoltScaleFactor();
      u2.i32 = g_EvseController.GetVoltOffset();
      sprintf(buffer,"%d %ld",u1.i,u2.i32);
      bufCnt = 1; // flag response text output
      break;
#endif // VOLTMETER
#ifdef TEMPERATURE_MONITORING
    case 'P':
      sprintf(buffer,"%d %d %d",(int)g_TempMonitor.m_DS3231_temperature,
	      (int)g_TempMonitor.m_MCP9808_temperature,
	      (int)g_TempMonitor.m_TMP007_temperature);
      /* this is bigger than using sprintf
      strcpy(buffer,u2a(g_TempMonitor.m_DS3231_temperature));
      strcat(buffer,g_sSpace);
      strcat(buffer,u2a(g_TempMonitor.m_MCP9808_temperature));
      strcat(buffer,g_sSpace);
      strcat(buffer,u2a(g_TempMonitor.m_TMP007_temperature));
      */
      bufCnt = 1; // flag response text output
      break;
#endif // TEMPERATURE_MONITORING
    case 'S': // get state
      sprintf(buffer,"%d %ld",g_EvseController.GetState(),g_EvseController.GetElapsedChargeTime());
      bufCnt = 1; // flag response text output

      break;
#ifdef RTC
    case 'T': // get time
      extern void GetRTC(char *buf);
      GetRTC(buffer);
      bufCnt = 1; // flag response text output

      break;
#endif // RTC
#ifdef KWH_RECORDING
    case 'U':
      sprintf(buffer,"%lu %lu",g_WattSeconds,g_WattHours_accumulated);
      bufCnt = 1;
      break;
#endif // KWH_RECORDING
    case 'V': // get version
      GetVerStr(buffer);
      strcat(buffer," ");
      strcat_P(buffer,RAPI_VER);
      bufCnt = 1; // flag response text output
      break;
    default:
      rc = -1; // unknown
    }

  default:
    rc = -1; // unknown
  }

  response((rc == 0) ? 1 : 0);

  reset();

  return rc;
}

void EvseRapiProcessor::response(uint8_t ok)
{
  write(ESRAPI_SOC);
  write(ok ? "OK " : "NK ");

  if (bufCnt) {
    write(buffer);
  }
  write(ESRAPI_EOC);
  if (echo) write('\n');
}


EvseRapiProcessor g_ERP;
#endif // RAPI
