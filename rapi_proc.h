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


 **** RAPI protocol ****

Fx - function
Sx - set parameter
Gx - get parameter

command formats
1. with XOR checksum (recommended)
$cc pp pp ...^xk\r
2. with additive checksum (legacy)
$cc pp pp ...*ck\r
3. no checksum (FOR TESTING ONLY! DON'T USE FOR APPS)
$cc pp pp ...\r

\r = carriage return = 13d = 0x0D
cc = 2-letter command
pp = parameters
xk = 2-hex-digit checksum - 8-bit XOR of all characters before '^'
ck = 2-hex-digit checksum - 8-bit sum of all characters before '*'


response format
$OK [optional parameters]\r - success

$NK [optional parameters]\r - failure

asynchronous messages
$ST state\r - EVSE state transition - sent whenever EVSE state changes
 state: EVSE_STATE_xxx
$WF mode\r - Request client WiFi mode
 mode: WIFI_MODE_XXX
 (currently very long press (10 sec) of menu btn on OpenEVSE will send WIFI_MODE_AP_DEFAULT

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
S1 yr mo day hr min sec - set clock (RTC) yr=2-digit year
S2 0|1 - disable/enable ammeter calibration mode - ammeter is read even when not charging
 $S2 0*F9
 $S2 1*FA
S3 cnt - set charge time limit to cnt*15 minutes (0=disable, max=255)
SA currentscalefactor currentoffset - set ammeter settings
SC amps - set current capacity
   if amps < minimum current capacity, will set to minimum and return $NK
   if amps > maximum current capacity, will set to maximum and return $NK
SD 0|1 - disable/enable diode check
 $SD 0*0B
 $SD 1*0C
SE 0|1 - disable/enable command echo
 $SE 0*0C
 $SE 1*0D
 use this for interactive terminal sessions with RAPI.
 RAPI will echo back characters as they are typed, and add a <LF> character
 after its replies
SF 0|1 - disable/enable GFI self test
 $SF 0*0D
 $SF 1*0E
SG 0|1 - disable/enable ground check
 $SG 0*0E
 $SG 1*0F
SH kWh - set cHarge limit to kWh
SK - set accumulated Wh (v1.0.3+)
 $SK 0*12 - set accumulated Wh to 0
SL 1|2|A  - set service level L1/L2/Auto
 $SL 1*14
 $SL 2*15
 $SL A*24
SM voltscalefactor voltoffset - set voltMeter settings
SO ambientthresh irthresh - set Overtemperature thresholds
 thresholds are in 10ths of a degree Celcius
SR 0|1 - disable/enable stuck relay check
 $SR 0*19
 $SR 1*1A
SS 0|1 - disable/enable GFI self-test
 $SS 0*1A
 $SS 1*1B
ST starthr startmin endhr endmin - set timer
 $ST 0 0 0 0*0B - cancel timer
SV 0|1 - disable/enable vent required
 $SV 0*1D
 $SV 1*1E

G3 - get time limit
 response: OK cnt
 cnt*15 = minutes
        = 0 = no time limit
GA - get ammeter settings
 response: OK currentscalefactor currentoffset
 $GA*AC
GC - get current capacity range in amps
 response: OK minamps maxamps
 $GC*AE
GE - get settings
 response: OK amps(decimal) flags(hex)
 $GE*B0
GF - get fault counters
 response: OK gfitripcnt nogndtripcnt stuckrelaytripcnt (all values hex)
 maximum trip count = 0xFF for any counter
 $GF*B1
GG - get charging current and voltage
 response: OK milliamps millivolts
 AMMETER must be defined in order to get amps, otherwise returns -1 amps
 VOLTMETER must be defined in order to get voltage, otherwise returns -1 volts
 $GG*B2
GH - get cHarge limit
 response: OK kWh
 kWh = 0 = no charge limit
GM - get voltMeter settings
 response: OK voltcalefactor voltoffset
 $GM^2E
GO get Overtemperature thresholds
 response: OK ambientthresh irthresh
 thresholds are in 10ths of a degree Celcius
 $GO^2C
GP - get temPerature (v1.0.3+)
 $GP*BB
 response: OK ds3231temp mcp9808temp tmp007temp
 ds3231temp - temperature from DS3231 RTC
 mcp9808temp - temperature from MCP9808
 tmp007temp - temperature from TMP007
 all temperatures are in 10th's of a degree Celcius
 if any temperature sensor is not installed, its return value will be 0
GS - get state
 response: OK state elapsed
 state: EVSE_STATE_xxx
 elapsed: elapsed charge time in seconds (valid only when in state C)
 $GS*BE
GT - get time (RTC)
 response OK yr mo day hr min sec       yr=2-digit year
 $GT*BF
GU - get energy usage (v1.0.3+)
 $GU*C0
 response OK Wattseconds Whacc
 Wattseconds - Watt-seconds used this charging session, note you'll divide Wattseconds by 3600 to get Wh
 Whacc - total Wh accumulated over all charging sessions, note you'll divide Wh by 1000 to get kWh
GV - get version
 response: OK firmware_version protocol_version
 $GV*C1

 *
 */

#ifdef RAPI

#define RAPIVER "1.0.3"

#define WIFI_MODE_AP 0
#define WIFI_MODE_CLIENT 1
#define WIFI_MODE_AP_DEFAULT 2

#define ESRAPI_BUFLEN 30
#define ESRAPI_SOC '$' // start of command
#define ESRAPI_EOC 0xd // CR end of command
#define ESRAPI_MAX_ARGS 10
class EvseRapiProcessor {
  char buffer[ESRAPI_BUFLEN]; // input buffer
  int8_t bufCnt; // # valid bytes in buffer
  char *tokens[ESRAPI_MAX_ARGS];
  int8_t tokenCnt;
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

  void response(uint8_t ok);
  
public:
  EvseRapiProcessor();
  int doCmd();
  void sendEvseState();
  void setWifiMode(uint8_t mode); // WIFI_MODE_xxx
};

extern EvseRapiProcessor g_ERP;

#endif // RAPI
