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


 **** RAPI protocol ****

Fx - function
Sx - set parameter
Gx - get parameter

command format
$cc pp pp ...*ck\r

cc = 2-letter command
pp = parameters
ck = 2-hex-digit checksum - 8-bit sum of all characters before '*'

response format
OK [optional parameters]\r - success

NK [optional parameters]\r - failure

asynchronous messages
ST state\r - EVSE state transition - sent whenever EVSE state changes
 state: EVSE_STATE_xxx

commands

FB color - set LCD backlight color
colors:
 OFF 0
 RED 1
 YELLOW 3
 GREEN 2
 TEAL 6
 BLUE 4
 VIOLET 5
 WHITE 7 

 $FB 7*03 - set backlight to white
FD - disable EVSE
 $FD*AE
FE - enable EVSE
 $FE*AF
FP x y text - print text on lcd display
FR - reset EVSE
 $FR*BC
FS - sleep EVSE
 $FS*BD

S0 0|1 - set LCD type
 $S0 0*F7 = monochrome backlight
 $S0 1*F8 = RGB backlight

SC amps - set current capacity
SD 0|1 - disable/enable diode check
 $SD 0*0B
SE 0|1 - disable/enable command echo
 $SE 1*0D
SG 0|1 - disable/enable ground check
 $SG 0*0E
SL 1|2|A  - set service level L1/L2/Auto
 $SL 1*14
 $SL 2*15
 $SL A*24
SR 0|1 - disable/enable stuck relay check
 $SR 0*19
 $SR 1*1A
SV 0|1 - disable/enable vent required
 $SV 0*1D

GC - get current capacity range in amps
 response: OK minamps maxamps
 $GC*AE
GE - get current settings
 response: OK amps flags
 $GE*B0
GS - get state
 response: OK state elapsed
 state: EVSE_STATE_xxx
 elapsed: elapsed charge time in seconds (valid only when in state C)
 $GS*BE
GV - get version
 response: OK firmware_version protocol_version
 $GV*C1

 *
 */

#ifdef RAPI
typedef int int32;
typedef unsigned int uint32;
typedef short int16;
typedef unsigned short uint16;
typedef char int8;
typedef unsigned char uint8;

#define RAPIVER "1.0.0"

#define ESRAPI_BUFLEN 30
#define ESRAPI_SOC '$' // start of command
#define ESRAPI_EOC 0xd // CR end of command
#define ESRAPI_MAX_ARGS 10
class EvseRapiProcessor {
  char buffer[ESRAPI_BUFLEN]; // input buffer
  int8 bufCnt; // # valid bytes in buffer
  char *tokens[ESRAPI_MAX_ARGS];
  int8 tokenCnt;
  char echo;

  int available() { return Serial.available(); }
  int read() { return Serial.read(); }
  void write(const char *str) { Serial.write(str); }
  void write(uint8_t u8) { Serial.write(u8); }

  void reset() {
    buffer[0] = 0;
    bufCnt = 0;
  }

  int tokenize();
  int processCmd();

  void response(uint8 ok);
  
public:
  EvseRapiProcessor();
  int doCmd();
  void sendEvseState();
  
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

extern EvseRapiProcessor g_ERP;

#endif // RAPI
