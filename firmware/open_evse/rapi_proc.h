// -*- C++ -*-
/*
 * Open EVSE Firmware
 *
 * Copyright (c) 2013-2019 Sam C. Lin <lincomatic@gmail.com>
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

A-prefix: asynchronous notification messages

Boot Notification
$AB postcode fwrev
 postcode(hex):
   if boot OK then postcode = 00
   if error then postcode
       07 = bad ground
       08 = stuck relay
       09 = GFI test failed
 fwrev(string): firmware revision

EVSE state transition: sent whenever EVSE state changes
$AT evsestate pilotstate currentcapacity vflags
 evsestate(hex): EVSE_STATE_xxx
 pilotstate(hex): EVSE_STATE_xxx
 currentcapacity(decimal): amps
 vflags(hex): m_wVFlags bits

External button press notification - only if RAPI_BTN defined
When the button is disabled ($FF B 0) send the event via RAPI
$AN type
 type: 0 - short press, 1 - long press

Request client WiFi mode - only if RAPI_WF defined
$WF mode\r
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
  substitute character 0xef for spaces within a string. LCD display code
  replaces 0xef with spaces
FR - restart EVSE
 $FR*BC
FS - sleep EVSE
 $FS*BD
FF - enable/disable feature
 $FF feature_id 0|1
 0|1 0=disable 1=enable
 feature_id:
  B = disable/enable front panel button
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
  - current session will stop when time reached. the limit automatically
    gets cancelled when EV disconnected
  - temporarily disables delay timer until EV disconnected or limit reached
 response:
  $OK - accepted
  $NK - invalid EVSE state
S4 0|1 - set auth lock (needs AUTH_LOCK defined and AUTH_LOCK_REG undefined)
   0 = unlocked
   1 = locked - EVSE won't charge until unlocked
   when auth lock is on, will not transition to State C and a lock icon is
   displayed in States A & B.
SA currentscalefactor currentoffset - set ammeter settings
SC amps [V|M]- set current capacity
 response:
   if amps < minimum current capacity, will set to minimum and return $NK ampsset
   if amps > maximum current capacity, will set to maximum and return $NK ampsset
   if in over temperature status, raising current capacity will fail and return $NK ampsset
   otherwise return $OK ampsset
   ampsset: the resultant current capacity
   default action is to save new current capacity to EEPROM for the currently active service level.
   if V is specified, then new current capacity is volatile, and will be
     reset to previous value at next reboot
   if M is specified, sets maximum L2 current capacity for the unit and writes
     to EEPROM. subsequent calls the $SC cannot exceed value set bye $SC M
     the value cannot be changed/erased via RAPI commands. Subsequent calls
     to $SC M will return $NK
SH kWh - set cHarge limit to kWh
 NOTES:
  - allowed only when EV connected in State B or C
  - current session will stop when total kWh reached. the limit automatically
    gets cancelled when EV disconnected
  - temporarily disables delay timer until EV disconnected or limit reached
 response:
  $OK - accepted
  $NK - invalid EVSE state
SK - set accumulated Wh (v1.0.3+)
 $SK 0^2C - set accumulated Wh to 0
SL 1|2|A  - set service level L1/L2/Auto
 $SL 1*14
 $SL 2*15
 $SL A*24
SM voltscalefactor voltoffset - set voltMeter settings
ST starthr startmin endhr endmin - set timer
 $ST 0 0 0 0^23 - cancel timer
SV mv - Set Voltage for power calculations to mv millivolts
 $SV 223576 - set voltage to 223.576
 NOTES:
  - only available if VOLTMETER not defined and KWH_RECORDING defined
  - volatile - value is lost, and replaced with VOLTS_FOR_Lx at boot
SY heartbeatinterval hearbeatcurrentlimit
 Response includes heartbeatinterval hearbeatcurrentlimit hearbeattrigger
 hearbeattrigger: 0 - There has never been a missed pulse, 
 2 - there is a missed pulse, and HS is still in current limit
 1 - There was a missed pulse once, but it has since been acknowledged. Ampacity has been successfully restored to max permitted 
 $SY 100 6  //If no pulse for 100 seconds, set EVE ampacity limit to 6A until missed pulse is acknowledged
 $SY        //This is a heartbeat supervision pulse.  Need one every heartbeatinterval seconds.
 $SY 165    //This is an acknowledgement of a missed pulse.  Magic Cookie = 165 (=0XA5)
 When you send a pulse, an NK response indicates that a previous pulse was missed and has not yet been acked

G0 - get EV connect state
 response: $OK connectstate
 connectstate: 0=not connected, 1=connected, 2=unknown
 -> connectstate is unknown when EVSE pilot is -12VDC
 $G0^53

G3 - get charging time limit
 response: $OK cnt
 cnt*15 = minutes
        = 0 = no time limit
 $G3^50

G4 - get auth lock (needs AUTH_LOCK defined and AUTH_LOCK_REG undefined)
 response: $OK lockstate
  lockstate = 0=unlocked, =1=locked
 $G4^57
GA - get ammeter settings
 response: $OK currentscalefactor currentoffset
 $GA^22

GC - get current capacity info
 response: $OK minamps hmaxamps pilotamps cmaxamps
 all values decimal
 minamps - min allowed current capacity
 hmaxamps - max hardware allowed current capacity MAX_CURRENT_CAPACITY_Ln
 pilotamps - current capacity advertised by pilot
 cmaxamps - max configured allowed current capacity (saved to EEPROM)
 n.b. maxamps,emaxamps values are dependent on the active service level (L1/L2)
 $GC^20

GD - get Delay timer
 response: $OK starthr startmin endhr endmin
   all values decimal
   if timer disabled, starthr=startmin=endhr=endmin=0
 $GD^27

GE - get settings
 response: $OK amps(decimal) flags(hex)
 $GE^26

GF - get fault counters
 response: $OK gfitripcnt nogndtripcnt stuckrelaytripcnt (all values hex)
 maximum trip count = 0xFF for any counter
 $GF^25

GG - get charging current and voltage
 response: $OK milliamps millivolts
 AMMETER must be defined in order to get amps, otherwise returns -1 amps
 $GG^24

GH - get cHarge limit
 response: $OK kWh
 kWh = 0 = no charge limit
 $GH^2B

GM - get voltMeter settings
 response: $OK voltcalefactor voltoffset
 $GM^2E

GO get Overtemperature thresholds
 response: $OK ambientthresh irthresh
 thresholds are in 10ths of a degree Celcius
 $GO^2C
GP - get temPerature (v1.0.3+)
 response: $OK ds3231temp mcp9808temp tmp007temp
 ds3231temp - temperature from DS3231 RTC
 mcp9808temp - temperature from MCP9808
 tmp007temp - temperature from TMP007
 all temperatures are in 10th's of a degree Celcius
 if any temperature sensor is not installed, its return value is -2560
 $GP^33

GS - get state
 response: $OK evsestate elapsed pilotstate vflags
 evsestate(hex): EVSE_STATE_xxx
 elapsed(dec): elapsed charge time in seconds of current or last charging session
 pilotstate(hex): EVSE_STATE_xxx
 vflags(hex): EVCF_xxx
 $GS^30

GT - get time (RTC)
 response: $OK yr mo day hr min sec       yr=2-digit year
 $GT^37

GU - get energy usage (v1.0.3+)
 response: $OK Wattseconds Whacc
 Wattseconds - Watt-seconds used this charging session, note you'll divide Wattseconds by 3600 to get Wh
 Whacc - total Wh accumulated over all charging sessions, note you'll divide Wh by 1000 to get kWh
 $GU^36

GV - get version
 response: $OK firmware_version protocol_version
 NOTE: protocol_version is deprecated. too hard to maintain variants.
 ignore it, and test commands for compatibility, instead.
 $GV^35

T commands for debugging only #define RAPI_T_COMMMANDS
T0 amps - set fake charging current
 response: $OK
 $T0 75
 
GY - Get Hearbeat Supervision Status
 Response includes heartbeatinterval hearbeatcurrentlimit hearbeattrigger
 hearbeattrigger: 0 - There has never been a missed pulse, 
 2 - there is a missed pulse, and HS is still in current limit
 1 - There was a missed pulse once, but it has since been acknkoledged. Ampacity has been successfully restored to max permitted 
 See SY above for worked expamples.

Z0 FOR TESTING RELAY_AUTO_PWM_PIN ONLY
Z0 closems holdpwm
   closems(dec) = # ms to apply DC to relay pin
   holdpwm(dec) = pwm duty cycle for relay hold 0-255


 *
 */

#ifdef RAPI

#define RAPIVER "5.1.3"

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
  uint8_t curReceivedSeqId;
  void appendSequenceId(char *s,uint8_t seqId);
#ifdef RAPI_SENDER
  uint8_t curSentSeqId;
  uint8_t getSendSequenceId();
  int8_t isAsyncToken();
  int8_t isRespToken();
#endif // RAPI_SENDER

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
  void sendBootNotification();
  void setWifiMode(uint8_t mode); // WIFI_MODE_xxx
  void sendButtonPress(uint8_t long_press);
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
uint8_t RapiSendEvseState(uint8_t force=0);
void RapiSetWifiMode(uint8_t mode);
void RapiSendButtonPress(uint8_t long_press);
void RapiSendBootNotification();

#endif // RAPI
