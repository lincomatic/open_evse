// -*- C++ -*-
/*
 * Open EVSE Firmware
 *
 * Copyright (c) 2013-2016 Sam C. Lin <lincomatic@gmail.com>
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
4. checksum + sequence id (v3.0.0+)
$cc pp pp .. :ss^xk\r

\r = carriage return = 13d = 0x0D
cc = 2-letter command
pp = parameters
xk = 2-hex-digit checksum - 8-bit XOR of all characters before '^'
ck = 2-hex-digit checksum - 8-bit sum of all characters before '*'
ss = optional 2-hex-digit sequence id - response will echo the sequence id
     so that receiver can verify that the response matches the command
     ss CANNOT be 00, which is reserved as an invalid value


response format (v1.0.3-)
$OK [optional parameters]\r - success
$NK [optional parameters]\r - failure

response format (v2.0.0+)
$OK [optional parameters]^xk\r - success
$NK [optional parameters]^xk\r - failure
xk = 2-hex-digit checksum - 8-bit XOR of all characters before '^'

response format (v3.0.0+)
$OK [optional parameters] [:ss]^xk\r - success
$NK [optional parameters] [:ss]^xk\r - failure
xk = 2-hex-digit checksum - 8-bit XOR of all characters before '^'
ss = optional 2-hex-digit sequence ID which was sent with the command
     only present if a sequence ID was send with the command

asynchronous notification messages
$ST state\r - EVSE state transition - sent whenever EVSE state changes
 state: EVSE_STATE_xxx
$WF mode\r - Request client WiFi mode
 mode: WIFI_MODE_XXX
 (currently very long press (10 sec) of menu btn on OpenEVSE will send WIFI_MODE_AP_DEFAULT
v2.0.1+: 2-hex-digit XOR checksum appended to asynchronous messages

commands


F0 {1|0}- enable/disable display updates
     enables/disables g_OBD.Update()
 $F0 1^43 - enable display updates and call g_OBD.Update()
 $F0 0^42 - disable display updates
F1 - simulate front panel button short press
 N.B.: it is possible that an asynchronous state change will be sent by the
  EVSE prior to sending the response to $F1
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
FR - restart EVSE
 $FR*BC
FS - sleep EVSE
 $FS*BD
FF - enable/disable feature
 $FF feature_id 0|1
 0|1 0=disable 1=enable
 feature_id:
  D = Diode check
  E = command Echo
   use this for interactive terminal sessions with RAPI.
   RAPI will echo back characters as they are typed, and add a <LF> character
   after its replies. Valid only over a serial connection, DO NOT USE on I2C
  F = GFI self test
  G = Ground check
  R = stuck Relay check
  T = temperature monitoring
  V = Vent required check
 $FF D 0 - disable diode check
 $FF G 1 - enable ground check

S0 0|1 - set LCD type
 $S0 0*F7 = monochrome backlight
 $S0 1*F8 = RGB backlight
S1 yr mo day hr min sec - set clock (RTC) yr=2-digit year
S2 0|1 - disable/enable ammeter calibration mode - ammeter is read even when not charging
 $S2 0*F9
 $S2 1*FA
S3 cnt - set charge time limit to cnt*15 minutes (0=disable, max=255)
 NOTES:
  - allowed only when EV connected in State B or C
  - temporarily disables delay timer until EV disconnected or limit reached
  - actually *extends* the current charging session. So if current session
    has already charged for 2hrs, then $S3 2 ends charging after total 2:30
 response:
  $OK - accepted
  $NK - invalid EVSE state
S4 0|1 - set auth lock (needs AUTH_LOCK defined and AUTH_LOCK_REG undefined)
   0 = unlocked
   1 = locked - EVSE won't charge until unlocked
   when auth lock is on, will not transition to State C and a lock icon is
   displayed in States A & B.
SA currentscalefactor currentoffset - set ammeter settings
SC amps [V]- set current capacity
   default action is to save new current capacity to EEPROM.
   if V is specified, then new current capacity is volatile, and will be
     reset to EEPROM value at next reboot
 response:
   if amps < minimum current capacity (6A), will set to minimum and return $NK ampsset
   if amps > maximum allowed current capacity, will set to maximum and return $NK ampsset
   if in over temperature status, raising current capacity will fail and return $NK ampsset
   otherwise return $OK ampsset
   ampsset: the resultant current capacity, which may be < requested amps
(DEPRECATED) SD 0|1 - disable/enable diode check
 $SD 0*0B
 $SD 1*0C
(DEPRECATED) SE 0|1 - disable/enable command echo
 $SE 0*0C
 $SE 1*0D
 use this for interactive terminal sessions with RAPI.
 RAPI will echo back characters as they are typed, and add a <LF> character
 after its replies. Valid only over a serial connection, DO NOT USE on I2C
(DEPRECATED) SF 0|1 - disable/enable GFI self test
 $SF 0*0D
 $SF 1*0E
(DEPRECATED) SG 0|1 - disable/enable ground check
 $SG 0*0E
 $SG 1*0F
SH kWh - set cHarge limit to kWh
 NOTES:
  - allowed only when EV connected in State B or C
  - temporarily disables delay timer until EV disconnected or limit reached
  - actually *extends* the charge to the limit. So say, current session has
    already charged 10kWh, $SH 5 will charge until 15kWh
 response:
  $OK - accepted
  $NK - invalid EVSE state
SK - set accumulated Wh (v1.0.3+)
 $SK 0*12 - set accumulated Wh to 0
SL 1|2|A  - set service level L1/L2/Auto
 $SL 1*14
 $SL 2*15
 $SL A*24
SM voltscalefactor voltoffset - set voltMeter settings
(DEPRECATED) SR 0|1 - disable/enable stuck relay check
 $SR 0*19
 $SR 1*1A
ST starthr startmin endhr endmin - set timer
 $ST 0 0 0 0*0B - cancel timer
(DEPRECATED)SV 0|1 - disable/enable vent required
 $SV 0*1D
 $SV 1*1E

G0 - get EV connect state
 response: $OK connectstate
 connectstate: 0=not connected, 1=connected, 2=unknown
 -> connectstate is unknown when EVSE pilot is -12VDC
G3 - get time limit
 response: $OK cnt
 cnt*15 = minutes
        = 0 = no time limit
G4 - get auth lock (needs AUTH_LOCK defined and AUTH_LOCK_REG undefined)
 response: $OK lockstate
  lockstate = 0=unlocked, =1=locked
GA - get ammeter settings
 response: $OK currentscalefactor currentoffset
 $GA*AC
GC - get current capacity range in amps
 response: $OK minamps maxamps
 $GC*AE
GD - get Delay timer
 response: $OK starthr startmin endhr endmin
   all values decimal
   if timer disabled, starthr=startmin=endhr=endmin=0
GE - get settings
 response: $OK amps(decimal) flags(hex)
 $GE*B0
GF - get fault counters
 response: $OK gfitripcnt nogndtripcnt stuckrelaytripcnt (all values hex)
 maximum trip count = 0xFF for any counter
 $GF*B1
GG - get charging current and voltage
 response: $OK milliamps millivolts
 AMMETER must be defined in order to get amps, otherwise returns -1 amps
 VOLTMETER must be defined in order to get voltage, otherwise returns -1 volts
 $GG*B2
GH - get cHarge limit
 response: $OK kWh
 kWh = 0 = no charge limit
GM - get voltMeter settings
 response: $OK voltcalefactor voltoffset
 $GM^2E
GO get Overtemperature thresholds
 response: $OK ambientthresh irthresh
 thresholds are in 10ths of a degree Celcius
 $GO^2C
GP - get temPerature (v1.0.3+)
 $GP*BB
 response: $OK ds3231temp mcp9808temp tmp007temp
 ds3231temp - temperature from DS3231 RTC
 mcp9808temp - temperature from MCP9808
 tmp007temp - temperature from TMP007
 all temperatures are in 10th's of a degree Celcius
 if any temperature sensor is not installed, its return value is -2560
GS - get state
 response: $OK evsestate elapsed
 evsestate(dec): EVSE_STATE_xxx
 elapsed(dec): elapsed charge time in seconds of current or last charging session
 $GS*BE
GT - get time (RTC)
 response: $OK yr mo day hr min sec       yr=2-digit year
 $GT*BF
GU - get energy usage (v1.0.3+)
 $GU*C0
 response: $OK Wattseconds Whacc
 Wattseconds - Watt-seconds used this charging session, note you'll divide Wattseconds by 3600 to get Wh
 Whacc - total Wh accumulated over all charging sessions, note you'll divide Wh by 1000 to get kWh
GV - get version
 response: $OK firmware_version protocol_version
 $GV*C1

T commands for debugging only #define RAPI_T_COMMMANDS
T0 amps - set fake charging current
 response: $OK
 $T0 75

Z0 FOR TESTING RELAY_AUTO_PWM_PIN ONLY
Z0 closems holdpwm
   closems(dec) = # ms to apply DC to relay pin
   holdpwm(dec) = pwm duty cycle for relay hold 0-255


 *
 */

#ifdef RAPI

#ifdef RAPI_FF
#define RAPIVER "4.0.1"
#elif defined(RAPI_SEQUENCE_ID)
#define RAPIVER "3.0.1"
#elif defined(RAPI_RESPONSE_CHK)
#define RAPIVER "2.0.4"
#else
#define RAPIVER "1.0.5"
#endif

#define WIFI_MODE_AP 0
#define WIFI_MODE_CLIENT 1
#define WIFI_MODE_AP_DEFAULT 2

#define ESRAPI_BUFLEN 32
#define ESRAPI_SOC '$' // start of command
#define ESRAPI_EOC 0xd // CR end of command
#define ESRAPI_SOS ':' // start of sequence id
#define ESRAPI_MAX_ARGS 10
// for RAPI_SENDER
#define RAPIS_TIMEOUT_MS 500
#define RAPIS_BUFLEN 20

#define INVALID_SEQUENCE_ID 0

class EvseRapiProcessor {
#ifdef GPPBUGKLUDGE
  char *buffer;
public:
  void setBuffer(char *buf) { buffer = buf; }
private:
#else
  char buffer[ESRAPI_BUFLEN]; // input buffer
#endif // GPPBUGKLUDGE
  int8_t bufCnt; // # valid bytes in buffer
  char *tokens[ESRAPI_MAX_ARGS];
  int8_t tokenCnt;
  char echo;
#ifdef RAPI_SEQUENCE_ID
  uint8_t curReceivedSeqId;
  void appendSequenceId(char *s,uint8_t seqId);
#ifdef RAPI_SENDER
  uint8_t curSentSeqId;
  uint8_t getSendSequenceId();
  int8_t isAsyncToken();
  int8_t isRespToken();
#endif // RAPI_SENDER
#endif // RAPI_SEQUENCE_ID

  virtual int available() = 0;
  virtual int read() = 0;
  virtual void writeStart() {}
  virtual void writeEnd() {}
  virtual int write(uint8_t u8) = 0;
  virtual int write(const char *str) = 0;

  void reset() {
    buffer[0] = 0;
    bufCnt = 0;
  }

  int tokenize(char *buf);
  int processCmd();

  void response(uint8_t ok);
  void appendChk(char *buf);
  
#ifdef RAPI_SENDER
  char sendbuf[RAPIS_BUFLEN]; // input buffer
  void _sendCmd(const char *cmdstr);
#endif // RAPI_SENDER
  
public:
  EvseRapiProcessor();

  int doCmd();
  void sendEvseState();
  void setWifiMode(uint8_t mode); // WIFI_MODE_xxx
  void writeStr(const char *msg) { writeStart();write(msg);writeEnd(); }

  virtual void init();

#ifdef RAPI_SENDER
  int8_t sendCmd(const char *cmdstr);
  int8_t receiveResp(unsigned long msstart);
#endif // RAPI_SENDER
};

#ifdef RAPI_SERIAL
class EvseSerialRapiProcessor : public EvseRapiProcessor {
  int available() { return Serial.available(); }
  int read() { return Serial.read(); }
  int write(uint8_t u8) { return Serial.write(u8); }
  int write(const char *str) { return Serial.write(str); }

public:
  EvseSerialRapiProcessor();
  void init();
};

extern EvseSerialRapiProcessor g_ESRP;
#endif // RAPI_SERIAL


#ifdef RAPI_I2C
class EvseI2cRapiProcessor : public EvseRapiProcessor {
  int available() { return Wire.available(); }
  int read() { return Wire.read(); }
  void writeStart() { Wire.beginTransmission(RAPI_I2C_REMOTE_ADDR); }
  void writeEnd() { Wire.endTransmission(); }
  int write(uint8_t u8) { return Wire.write(u8); }
  int write(const char *str) { return Wire.write(str); }

public:
  EvseI2cRapiProcessor();
  void init();
};

extern EvseI2cRapiProcessor g_EIRP;
#endif // RAPI_I2C

void RapiInit();
void RapiDoCmd();
void RapiSendEvseState(uint8_t nodupe=1);
void RapiSetWifiMode(uint8_t mode);

#endif // RAPI
