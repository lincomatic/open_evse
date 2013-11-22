// -*- C++ -*-
/*
 * Open EVSE Firmware
 *
 * Copyright (c) 2013 Sam C. Lin <lincomatic@gmail.com>
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

prog_char RAPI_VER[] PROGMEM = "1.0";

EvseRapiProcessor::EvseRapiProcessor()
{
  echo = 0;
  reset();
}

//extern HardwareSerial Serial;
int EvseRapiProcessor::doCmd()
{
  int rc = 1;

  // chk for state transition and send async notification
  if (g_EvseController.StateTransition()) {
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
  sprintf(g_sTmp,"%cSS %d%c",ESRAPI_SOC,g_EvseController.GetState(),ESRAPI_EOC);
  write(g_sTmp);
}

// convert 2-digit hex string to uint8
uint8 EvseRapiProcessor::htou(const char *s)
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
uint8 EvseRapiProcessor::dtou(const char *s)
{
  uint8 u = 0;
  for (int i=0;i < 2;i++) {
    if (!s[i]) break;
    u *= i*10;
    u += s[i] - '0';
  }
  return u;
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
      ichkSum = htou(s);
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
  unsigned u1,u2;

  // we use bufCnt as a flag in response() to signify data to write
  bufCnt = 0;

  char *s = tokens[0];
  switch(*(s++)) { 
  case 'F': // function
    switch(*s) {
    case 'B': // LCD backlight
      g_OBD.LcdSetBacklightColor(dtou(tokens[1]));
      rc = 0;
      break;
    case 'D': // display
      {
	uint8 x = dtou(tokens[1]);
	uint8 y = dtou(tokens[2]);
	// now restore the spaces that were replaced w/ nulls by tokenizing
	for (int i=4;i < tokenCnt;i++) {
	  *(tokens[i]-1) = ' ';
	}
	g_OBD.LcdPrint(x,y,tokens[3]);
      }
      rc = 0;
      break;
    case 'P': // pause charging
      g_EvseController.Disable();
      rc = 0;
      break;
    case 'R': // reset EVSE
      extern void WatchDogReset();
      WatchDogReset();
      rc = 0;
      break;
    case 'S': // start charging
      g_EvseController.Enable();
      rc = 0;
      break;
    }
    break;

  case 'S': // set parameter
    switch(*s) {
    case 'C': // current capacity
      if (tokenCnt == 2) {
	rc = g_EvseController.SetCurrentCapacity(dtou(tokens[1]));
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
	  g_EvseController.SetSvcLevel(*tokens[1] - '0');
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
    case 'S': // save settings to EEPROM
      extern void SaveSettings();
      SaveSettings();
      rc = 0;
      break;
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
    case 'C': // get current capacity range
      sprintf(buffer,"%d %d",MIN_CURRENT_CAPACITY,(g_EvseController.GetCurSvcLevel() == 2) ? MAX_CURRENT_CAPACITY_L2 : MAX_CURRENT_CAPACITY_L1);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
    case 'E': // get settings
      u1 = g_EvseController.GetCurrentCapacity();
      u2 = g_EvseController.GetFlags();
      sprintf(buffer,"%d %02x",u1,u2);
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
    case 'S': // get state
      sprintf(buffer,"%d %ld",g_EvseController.GetState(),g_EvseController.GetElapsedChargeTime());
      bufCnt = 1; // flag response text output
      rc = 0;
      break;
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
