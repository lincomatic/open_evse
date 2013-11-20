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


 RAPI protocol
Fx - function
Sx - set parameter
Gx - get parameter

command format
$cc pp pp ...*ck\n

cc = 2-letter command
pp = parameters
ck = 2-hex-digit checksum - sum of all characters before '*'

FS - start charging
$FS*BD
FP - pause charging
$FP*BA
FR - reset EVSE
$FR*BC

$SC amps - set current capacity
SL 1|2|A  - set service level L1/L2/Auto
$SL 1*14
$SL 2*15
$SL A*24
$SD 0|1 - disable/enable diode check
$SD 0*0B
SG 0|1 - disable/enable ground check
$SG 0*0E
SV 0|1 - disable/enable vent required
$SV 0*1D
SE - save current settings to EEPROM
$SE*BC

GC - get current capacity range in amps
$GC*AE
response: OK minamps maxamps
GE - get EEPROM settings
$GE*B0
response: OK amps flags
GS - get state
$GS*BE
response: OK state elapsed
state:
#define EVSE_STATE_UNKNOWN 0x00
#define EVSE_STATE_A       0x01 // vehicle state A 12V - not connected
#define EVSE_STATE_B       0x02 // vehicle state B 9V - connected, ready
#define EVSE_STATE_C       0x03 // vehicle state C 6V - charging
#define EVSE_STATE_D       0x04 // vehicle state D 3V - vent required
#define EVSE_STATE_DIODE_CHK_FAILED 0x05 // diode check failed
#define EVSE_STATE_GFCI_FAULT 0x06       // GFCI fault
#define EVSE_STATE_NO_GROUND 0x07 //bad ground
#define EVSE_STATE_STUCK_RELAY 0x08 //stuck relay
#define EVSE_STATE_DISABLED 0xff // disabled
elapsed: elapsed charge time in seconds (valid only when in state C)
 *
 */

#ifdef RAPI
typedef int int32;
typedef unsigned int uint32;
typedef short int16;
typedef unsigned short uint16;
typedef char int8;
typedef unsigned char uint8;

#define ESRAPI_BUFLEN 40
#define ESRAPI_EOC 0xd // CR
#define ESRAPI_MAX_ARGS 10
class EvseRapiProcessor {
  char buffer[ESRAPI_BUFLEN]; // input buffer
  int8 bufCnt; // # valid bytes in buffer
  char *tokens[ESRAPI_MAX_ARGS];
  int8 tokenCnt;

  int available() { return Serial.available(); }
  int read() { return Serial.read(); }
  void write(const char *str) { Serial.write(str); }
  void write(uint8_t u8) { Serial.write(u8); }

  void reset() {
    buffer[0] = 0;
    bufCnt = 0;
  }

  uint8 htou(const char *s);
  uint8 dtou(const char *s);
  int tokenize();
  int processCmd();

  void response(uint8 ok);
  
public:
  EvseRapiProcessor();
  int doCmd();
  
  /*
  void Init();
  void println(char *s) { 
    Serial.println(s); 
  }
  void println_P(prog_char *s);
  void print(char *s) { 
    Serial.print(s); 
  }
  void print_P(prog_char *s);
  void printlnn();
  void flush() { 
    Serial.flush(); 
  }
  uint8_t getInt();
  */
};

#endif // RAPI
