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
extern J1772EVSEController g_EvseController;
extern OnboardDisplay g_OBD;

prog_char RAPI_VER[] PROGMEM = RAPIVER;


// convert 2-digit hex string to uint8
uint8 htou8(const char *s)
{
  uint8 u = 0;
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

// convert decimal string to uint8
uint8 dtou8(const char *s)
{
  uint8 u = 0;
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
int EvseRapiProcessor::doCmd(int8 sendstatetrans)
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
  extern char g_sTmp[64];
  sprintf(g_sTmp,"%cST %02x%c",ESRAPI_SOC,g_EvseController.GetState(),ESRAPI_EOC);
  write(g_sTmp);
}

void EvseRapiProcessor::setWifiMode(uint8_t mode)
{
  extern char g_sTmp[64];
  sprintf(g_sTmp,"%cWF %02x%c",ESRAPI_SOC,(int)mode,ESRAPI_EOC);
  write(g_sTmp);
}

int EvseRapiProcessor::tokenize()
{
  tokens[0] = &buffer[1];
  char *s = &buffer[2];
  tokenCnt = 1;
  uint8 chkSum = ESRAPI_SOC + buffer[1];
  uint8 ichkSum = 0;
  while (*s) {
    if (*s == ' ') {
      chkSum += *s;
      *s = '\0';
      tokens[tokenCnt++] = ++s;
    }
    else if (*s == '*') {
      *(s++) = '\0';
      ichkSum = htou8(s);
	  break;
    }
    else {
      chkSum += *(s++);
    }
  }

  return (chkSum == ichkSum) ? 0 : 1;
}

int EvseRapiProcessor::processCmd()
{
  int rc = -1;
  unsigned u1,u2,u3;
  int i,i2;
  uint8 x,y;

  // we use bufCnt as a flag in response() to signify data to write
  bufCnt = 0;

  char *s = tokens[0];
  switch(*(s++)) { 
  case 'F': // function
    switch(*s) {
#ifdef LCD16X2
    case 'B': // LCD backlight
      g_OBD.LcdSetBacklightColor(dtou8(tokens[1]));
      rc = 0;
      break;
#endif // LCD16X2      
    case 'D': // disable EVSE
      g_EvseController.Disable();
      rc = 0;
      break;
    case 'E': // enable EVSE
      g_EvseController.Enable();
      rc = 0;
      break;
#ifdef LCD16X2
    case 'P': // print to LCD
      {
	u1 = dtou8(tokens[1]); // x
	u2 = dtou8(tokens[2]); // y
	// now restore the spaces that were replaced w/ nulls by tokenizing
	for (i=4;i < tokenCnt;i++) {
	  *(tokens[i]-1) = ' ';
	}
	g_OBD.LcdPrint(u1,u2,tokens[3]);
      }
      rc = 0;
      break;
#endif // LCD16X2      
    case 'R': // reset EVSE
      extern void WatchDogResetEvse();
      WatchDogResetEvse();
      rc = 0;
      break;
    case 'S': // sleep
      g_EvseController.Sleep();
      rc = 0;
      break;
    default:
      ; // do nothing
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
	extern void SetRTC(uint8 y,uint8 m,uint8 d,uint8 h,uint8 mn,uint8 s);
	SetRTC(dtou8(tokens[1]),dtou8(tokens[2]),dtou8(tokens[3]),
	       dtou8(tokens[4]),dtou8(tokens[5]),dtou8(tokens[6]));
	rc = 0;
      }
      break;
#endif // RTC      
#ifdef AMMETER
    case '2': // ammeter calibration mode
      if (tokenCnt == 2) {
	g_EvseController.EnableAmmeterCal((*tokens[1] == '1') ? 1 : 0);
	rc = 0;
      }
      break;
    case 'A':
      if (tokenCnt == 3) {
	sscanf(tokens[1],"%d",&i);
	g_EvseController.SetCurrentScaleFactor(i);
	sscanf(tokens[2],"%d",&i);
	g_EvseController.SetAmmeterCurrentOffset(i);
	rc = 0;
      }
      break;
#endif // AMMETER
    case 'C': // current capacity
      if (tokenCnt == 2) {
	rc = g_EvseController.SetCurrentCapacity(dtou8(tokens[1]),1);
      }
      break;
    case 'D': // diode check
      if (tokenCnt == 2) {
	g_EvseController.EnableDiodeCheck((*tokens[1] == '0') ? 0 : 1);
	rc = 0;
      }
      break;
    case 'E': // echo
      if (tokenCnt == 2) {
	echo = ((*tokens[1] == '0') ? 0 : 1);
	rc = 0;
      }
      break;
#ifdef GFI_SELFTEST
    case 'F': // GFI self test
      if (tokenCnt == 2) {
	g_EvseController.EnableGfiSelfTest(*tokens[1] == '0' ? 0 : 1);
	rc = 0;
      }
      break;
#endif // GFI_SELFTEST
#ifdef ADVPWR
    case 'G': // ground check
      if (tokenCnt == 2) {
	g_EvseController.EnableGndChk(*tokens[1] == '0' ? 0 : 1);
	rc = 0;
      }
      break;
#endif // ADVPWR
    case 'L': // service level
      if (tokenCnt == 2) {
	switch(*tokens[1]) {
	case '1':
	case '2':
	  g_EvseController.SetSvcLevel(*tokens[1] - '0',1);
#ifdef ADVPWR
	  g_EvseController.EnableAutoSvcLevel(0);
#endif
	  rc = 0;
	  break;
#ifdef ADVPWR
	case 'A':
	  g_EvseController.EnableAutoSvcLevel(1);
	  rc = 0;
	  break;
#endif // ADVPWR
	default:
	  ;
	}
      }
      break;
#ifdef ADVPWR      
    case 'R': // stuck relay check
      if (tokenCnt == 2) {
	g_EvseController.EnableStuckRelayChk(*tokens[1] == '0' ? 0 : 1);
	rc = 0;
      }
      break;
#endif // ADVPWR      
#ifdef GFI_SELFTEST
    case 'S': // GFI self-test
      if (tokenCnt == 2) {
	g_EvseController.EnableGfiSelfTest(*tokens[1] == '0' ? 0 : 1);
	rc = 0;
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
	  g_DelayTimer.SetStartTimer(dtou8(tokens[1]),dtou8(tokens[2]));
	  g_DelayTimer.SetStopTimer(dtou8(tokens[3]),dtou8(tokens[4]));
	  g_DelayTimer.Enable();
	}
	rc = 0;
      }
      break;
#endif // DELAYTIMER      
    case 'V': // vent required
      if (tokenCnt == 2) {
	g_EvseController.EnableVentReq(*tokens[1] == '0' ? 0 : 1);
	rc = 0;
      }
      break;
    }
    break;

  case 'G': // get parameter
    switch(*s) {
#ifdef AMMETER
    case 'A':
      i = g_EvseController.GetCurrentScaleFactor();
      i2 = g_EvseController.GetAmmeterCurrentOffset();
      sprintf(buffer,"%d %d",i,i2);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#endif // AMMETER
    case 'C': // get current capacity range
      sprintf(buffer,"%d %d",MIN_CURRENT_CAPACITY,(g_EvseController.GetCurSvcLevel() == 2) ? MAX_CURRENT_CAPACITY_L2 : MAX_CURRENT_CAPACITY_L1);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
    case 'E': // get settings
      u1 = g_EvseController.GetCurrentCapacity();
      u2 = g_EvseController.GetFlags();
      sprintf(buffer,"%d %04x",u1,u2);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
    case 'F': // get fault counters
      u1 = eeprom_read_byte((uint8_t*)EOFS_GFI_TRIP_CNT);
      u2 = eeprom_read_byte((uint8_t*)EOFS_NOGND_TRIP_CNT);
      u3 = eeprom_read_byte((uint8_t*)EOFS_STUCK_RELAY_TRIP_CNT);
      sprintf(buffer,"%x %x %x",u1,u2,u3);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#ifdef AMMETER
    case 'G':
      u1 = g_EvseController.GetChargingCurrent();
      sprintf(buffer,"%u",u1);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#endif // AMMETER
    case 'S': // get state
      sprintf(buffer,"%d %ld",g_EvseController.GetState(),g_EvseController.GetElapsedChargeTime());
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#ifdef RTC
    case 'T': // get time
      extern void GetRTC(char *buf);
      GetRTC(buffer);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
#endif // RTC
    case 'V': // get version
      extern void GetVerStr(char *buf);
      GetVerStr(buffer);
      strcat(buffer," ");
      strcat_P(buffer,RAPI_VER);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
    default:
      ; //do nothing
    }

  default:
    ; // do nothing
  }

  response((rc == 0) ? 1 : 0);

  reset();

  return rc;
}

void EvseRapiProcessor::response(uint8 ok)
{
  write(ESRAPI_SOC);
  write(ok ? "OK " : "NK ");

  if (bufCnt) {
    write(buffer);
  }
  write(ESRAPI_EOC);
}


EvseRapiProcessor g_ERP;
#endif // RAPI
