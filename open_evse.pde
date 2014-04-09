// -*- C++ -*-
/*
 * Open EVSE Firmware
 *
 * Copyright (c) 2011-2013 Sam C. Lin <lincomatic@gmail.com>
 * Copyright (c) 2011-2013 Chris Howell <chris1howell@msn.com>
 * timer code Copyright (c) 2013 Kevin L <goldserve1@hotmail.com>
 * Maintainers: SCL/CH

  Revised  Ver	By		Reason
  6/21/13  20b3	Scott Rubin	fixed LCD display bugs with RTC enabled
  6/25/13  20b4	Scott Rubin	fixed LCD display bugs, CLI fixes, when RTC disabled
  6/30/13  20b5	Scott Rubin	added LcdDetected() function, prevents hang if LCD not installed
  7/06/13  20b5	Scott Rubin	rewrote power detection in POST function for 1 or 2 relays
  7/11/13  20b5	Scott Rubin	skips POST if EV is connected, won't charge if open ground or stuck relay
  8/12/13  20b5b Scott Rubin    fix GFI error - changed gfi.Reset() to check for constant GFI signal
  8/26/13  20b6 Scott Rubin     add Stuck Relay State delay, fix Stuck Relay state exit (for Active E)
  9/20/13  20b7 Chris Howell    updated/tweaked/shortened CLI messages   
  
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
#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <pins_arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <FlexiTimer2.h> // Required for RTC and Delay Timer
#include <Time.h>
#if defined(ARDUINO) && (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h" // shouldn't need this but arduino sometimes messes up and puts inside an #ifdef
#endif // ARDUINO

prog_char VERSTR[] PROGMEM = "2.0.0";

//-- begin features

// enable watchdog timer
//#define WATCHDOG

// GFI support
#define GFI
// If you loop a wire from the third GFI pin through the CT a few times and then to ground,
// enable this
#define GFI_SELFTEST

// serial port command line
// For the RTC version, only CLI or LCD can be defined at one time. 
// There is a directive to take care of that if you forget.
#define SERIALCLI

//Adafruit RGBLCD
#define RGBLCD

// Adafruit LCD backpack in I2C mode
//#define I2CLCD

// White LCD - define all colors as white
//#define WHITELCD

// Advanced Powersupply... Ground check, stuck relay, L1/L2 detection.
#define ADVPWR

// single button menus (needs LCD enabled)
// connect an SPST button between BTN_PIN and GND or enable ADAFRUIT_BTN to use the 
// select button of the Adafruit RGB LCD 
// How to use 1-button menu
// Long press activates menu
// When within menus, short press cycles menu items, long press selects and exits current submenu
#define BTN_MENU

// When not in menus, short press instantly stops the EVSE - another short press resumes.  Long press activates menus
// also allows menus to be manipulated even when in State B/C
#define BTN_ENABLE_TOGGLE

#ifdef BTN_MENU
// use Adafruit RGB LCD select button
#ifdef RGBLCD
#define ADAFRUIT_BTN
#endif // RGBLCD
#endif // BTN_MENU

// Option for RTC and DelayTime
//#define RTC // enable RTC & timer functions

#ifdef RTC
// Option for Delay Timer - GoldServe
#define DELAYTIMER
// Option for AutoStart Enable/Disable - GoldServe
#define MANUALSTART
// Option for AutoStart Menu. If defined, ManualStart feature is also defined by default - GoldServe
//#define AUTOSTART_MENU

// AutoStart feature must be defined if Delay Timers are used - GoldServe
#if defined(DELAYTIMER)||defined(AUTOSTART_MENU)
#define MANUALSTART
#endif

#endif // RTC

// for stability testing - shorter timeout/higher retry count
//#define GFI_TESTING

// phase and frequency correct PWM 1/8000 resolution
// when not defined, use fast PWM -> 1/250 resolution
#define PAFC_PWM

//-- end features



#if defined(RGBLCD) || defined(I2CLCD)
#define LCD16X2
//If LCD is not defined, undef BTN_MENU - requires LCD
#else
#undef BTN_MENU
#endif // RGBLCD || I2CLCD

//If LCD and RTC is defined, un-define CLI so we can save ram space.
#if defined(RTC) && defined(LCD16X2)
#undef SERIALCLI
#endif

//-- begin configuration

#define LCD_MAX_CHARS_PER_LINE 16

// n.b. DEFAULT_SERVICE_LEVEL is ignored if ADVPWR defined, since it's autodetected
#define DEFAULT_SERVICE_LEVEL 2 // 1=L1, 2=L2

// current capacity in amps
#define DEFAULT_CURRENT_CAPACITY_L1 12
#define DEFAULT_CURRENT_CAPACITY_L2 16

// minimum allowable current in amps
#define MIN_CURRENT_CAPACITY 6

// maximum allowable current in amps
#define MAX_CURRENT_CAPACITY_L1 16 // J1772 Max for L1 on a 20A circuit
#define MAX_CURRENT_CAPACITY_L2 80 // J1772 Max for L2

//J1772EVSEController
//#define CURRENT_PIN 0 // analog current reading pin A0
#define VOLT_PIN 1 // analog voltage reading pin A1
#define ACLINE1_PIN 3 // TEST PIN 1 for L1/L2, ground and stuck relay
#define ACLINE2_PIN 4 // TEST PIN 2 for L1/L2, ground and stuck relay
#define RED_LED_PIN 5 // Digital pin
#define CHARGING_PIN2 7 // digital Relay trigger pin for second relay
#define CHARGING_PIN 8 // digital Charging LED and Relay Trigger pin

// N.B. if PAFC_PWM is enabled, then PILOT_PIN can be either 9 or 10
// (i.e PORTB pins 1 & 2)
// if using fast PWM (PAFC_PWM disabled) PILOT_PIN *MUST* be digital 10
// and digital 9 may NOT be used for other purposes
#define PILOT_PIN 10

#define GREEN_LED_PIN 13 // Digital pin

#define SERIAL_BAUD 38400

// EEPROM offsets for settings
#define EOFS_CURRENT_CAPACITY_L1 0 // 1 byte
#define EOFS_CURRENT_CAPACITY_L2 1 // 1 byte
#define EOFS_FLAGS               2 // 1 byte

// EEPROM offsets for Delay Timer function - GoldServe
#define EOFS_TIMER_FLAGS         3 // 1 byte
#define EOFS_TIMER_START_HOUR    4 // 1 byte
#define EOFS_TIMER_START_MIN     5 // 1 byte
#define EOFS_TIMER_STOP_HOUR     6 // 1 byte
#define EOFS_TIMER_STOP_MIN      7 // 1 byte
#define EOFS_FLAGS2              8 // 1 byte

// must stay within thresh for this time in ms before switching states
#define DELAY_STATE_TRANSITION 250
// must transition to state A from contacts closed in < 100ms according to spec
// but Leaf sometimes bounces from 3->1 so we will debounce it a little anyway
#define DELAY_STATE_TRANSITION_A 25

// for ADVPWR
#define GROUND_CHK_DELAY  1000 // delay after charging started to test, ms
#define STUCK_RELAY_DELAY 1000 // delay after charging opened to test, ms
#define RelaySettlingTime  250 // time for relay to settle in post, ms

#ifdef GFI
#define GFI_INTERRUPT 0 // interrupt number 0 = D2, 1 = D3
#define GFI_PIN 2  // interrupt number 0 = D2, 1 = D3
#ifdef GFI_SELFTEST
#define GFI_TEST_PIN 6 // D6 is supposed to be wrapped around the GFI CT 5+ times
#define GFI_TEST_CYCLES 50 // 50 cycles
#define GFI_PULSE_DURATION_MS 8000 // of roughly 60 Hz. - 8 ms as a half-cycle
#define GFI_TEST_CLEAR_TIME 250 // Takes the GFCI this long to clear
#endif

#ifdef GFI_TESTING
#define GFI_TIMEOUT ((unsigned long)(15*1000))
#define GFI_RETRY_COUNT  255
#else // !GFI_TESTING
#define GFI_TIMEOUT ((unsigned long)(15*60000)) // 15*60*1000 doesn't work. go figure
#define GFI_RETRY_COUNT  3
#endif // GFI_TESTING
#endif // GFI

// for RGBLCD
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define BLUE 0x4
#define TEAL 0x6
#define VIOLET 0x5
#define WHITE 0x7 

#if defined(RGBLCD) || defined(I2CLCD)
// Using LiquidTWI2 for both types of I2C LCD's
// see http://blog.lincomatic.com/?p=956 for installation instructions
#include <Wire.h>
#include <LiquidTWI2.h>
#define LCD_I2C_ADDR 0x20 // for adafruit shield or backpack
#endif // RGBLCD || I2CLCD

#define BTN_PIN A3 // button sensing pin
#define BTN_PRESS_SHORT 100  // ms
#define BTN_PRESS_LONG 500 // ms


#ifdef RTC
// Default start/stop timers for un-initialized EEPROMs.
// Makes it easy to compile in default time without need to set it up the first time.
#define DEFAULT_START_HOUR    0x00 //Start time: 00:05
#define DEFAULT_START_MIN     0x05
#define DEFAULT_STOP_HOUR     0x06 //End time: 6:55
#define DEFAULT_STOP_MIN      0x37
#endif // RTC

//-- end configuration

//-- begin class definitions

#ifdef SERIALCLI
#define CLI_BUFLEN 20
class CLI {
  char m_CLIinstr[CLI_BUFLEN]; // CLI byte being read in
  int m_CLIstrCount; //CLI string counter
  char *m_strBuf;
  int m_strBufLen;

  void info();
public:
  CLI();
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
  void getInput();
  uint8_t getInt();
};
#endif // SERIALCLI

#ifdef LCD16X2
char *g_BlankLine = "                ";
#endif // LCD16X2

class OnboardDisplay 

{
#if defined(RGBLCD) || defined(I2CLCD)
LiquidTWI2 m_Lcd;
#endif
  char m_strBuf[LCD_MAX_CHARS_PER_LINE+1];


public:
  OnboardDisplay();
  void Init();
  void SetGreenLed(uint8_t state);
  void SetRedLed(uint8_t state);
#ifdef LCD16X2
  void LcdBegin(int x,int y) { 
#ifdef I2CLCD
    m_Lcd.setMCPType(LTI_TYPE_MCP23008);
    m_Lcd.begin(x,y); 
    m_Lcd.setBacklight(HIGH);
#elif defined(RGBLCD)
    m_Lcd.setMCPType(LTI_TYPE_MCP23017);
    m_Lcd.begin(x,y,2); 
    m_Lcd.setBacklight(WHITE);
#endif
  }
  void LcdPrint(const char *s) { 
    if (m_Lcd.LcdDetected()) m_Lcd.print(s); 
  }
  void LcdPrint_P(const prog_char *s);
  void LcdPrint(int y,const char *s);
  void LcdPrint_P(int y,const prog_char *s);
  void LcdPrint(int x,int y,const char *s) { 
    m_Lcd.setCursor(x,y);
    if (m_Lcd.LcdDetected()) m_Lcd.print(s); 
  }
  void LcdPrint_P(int x,int y,const prog_char *s);
  void LcdPrint(int i) { 
    if (m_Lcd.LcdDetected()) m_Lcd.print(i); 
  }
  void LcdSetCursor(int x,int y) { 
    m_Lcd.setCursor(x,y); 
  }
  void LcdClearLine(int y) {
    m_Lcd.setCursor(0,y);
    if (m_Lcd.LcdDetected()) m_Lcd.print(g_BlankLine);
  }
  void LcdClear() { 
    m_Lcd.clear();
  }
  void LcdWrite(uint8_t data) { 
    m_Lcd.write(data);
  }
  void LcdMsg(const char *l1,const char *l2);
  void LcdMsg_P(const prog_char *l1,const prog_char *l2);
  void LcdSetBacklightColor(uint8_t c) {
#ifdef RGBLCD
    m_Lcd.setBacklight(c);
#endif // RGBLCD
  }
#ifdef RGBLCD
  uint8_t readButtons() { return m_Lcd.readButtons(); }
#endif // RGBLCD
#endif // LCD16X2

  void Update();
};

#ifdef GFI
class Gfi {
 uint8_t m_GfiFault;
 uint8_t testSuccess;
 uint8_t testInProgress;
public:
  Gfi() {}
  void Init();
  void Reset();
  void SetFault() { m_GfiFault = 1; }
  uint8_t Fault() { return m_GfiFault; }
#ifdef GFI_SELFTEST
  void SelfTest();
  void SetTestSuccess() { testSuccess = true; }
  boolean SelfTestSuccess() { return testSuccess; }
  boolean SelfTestInProgress() { return testInProgress; }
#endif
  
};
#endif // GFI


typedef enum {
  PILOT_STATE_P12,PILOT_STATE_PWM,PILOT_STATE_N12} 
PILOT_STATE;
class J1772Pilot {
#ifndef PAFC_PWM
  uint8_t m_bit;
  uint8_t m_port;
#endif // PAFC_PWM
  PILOT_STATE m_State;
public:
  J1772Pilot() {
  }
  void Init();
  void SetState(PILOT_STATE pstate); // P12/N12
  PILOT_STATE GetState() { 
    return m_State; 
  }
  int SetPWM(int amps); // 12V 1KHz PWM
};

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
#define EVSE_STATE_GFI_TEST_FAILED 0x09 // GFI self-test failure
#define EVSE_STATE_DISABLED 0xff // disabled

typedef struct threshdata {
  uint16_t m_ThreshAB; // state A -> B
  uint16_t m_ThreshBC; // state B -> C
  uint16_t m_ThreshCD; // state C -> D
  uint16_t m_ThreshD;  // state D
  uint16_t m_ThreshDS; // diode short
} THRESH_DATA,*PTHRESH_DATA;

typedef struct calibdata {
  uint16_t m_pMax;
  uint16_t m_pAvg;
  uint16_t m_pMin;
  uint16_t m_nMax;
  uint16_t m_nAvg;
  uint16_t m_nMin;
} CALIB_DATA,*PCALIB_DATA;



// J1772EVSEController m_bFlags bits - saved to EEPROM
#define ECF_L2                 0x01 // service level 2
#define ECF_DIODE_CHK_DISABLED 0x02 // no diode check
#define ECF_VENT_REQ_DISABLED  0x04 // no vent required state
#define ECF_GND_CHK_DISABLED   0x08 // no chk for ground fault
#define ECF_STUCK_RELAY_CHK_DISABLED 0x10 // no chk for stuck relay
#define ECF_AUTO_SVC_LEVEL_DISABLED  0x20 // auto detect svc level - requires ADVPWR
// Ability set the EVSE for manual button press to start charging - GoldServe
#define ECF_AUTO_START_DISABLED 0x40  // no auto start charging
#define ECF_SERIAL_DBG         0x80 // enable debugging messages via serial
#define ECF_DEFAULT            0x00
// J1772EVSEController m_bFlags2 bits - saved to EEPROM
#define ECF2_GFI_TEST_DISABLED  0x01 // no GFI self test
#define ECF2_DEFAULT            0x00

// J1772EVSEController volatile m_bVFlags bits - not saved to EEPROM
#define ECVF_NOGND_TRIPPED      0x20 // no ground has tripped at least once
#define ECVF_CHARGING_ON        0x40 // charging relay is closed
#define ECVF_GFI_TRIPPED        0x80 // gfi has tripped at least once
#define ECVF_DEFAULT            0x00

class J1772EVSEController {
  J1772Pilot m_Pilot;
#ifdef GFI
  Gfi m_Gfi;
  unsigned long m_GfiTimeout;
  unsigned long m_GfiRetryCnt;
  uint8_t m_GfiTripCnt;
#endif // GFI
#ifdef ADVPWR
  unsigned long m_NoGndTimeout;
  unsigned long m_NoGndRetryCnt;
  uint8_t m_NoGndTripCnt;
  unsigned long m_StuckRelayStartTimeMS;
  uint8_t StuckRelayTripCnt;
#endif // ADVPWR
  uint8_t m_bFlags, m_bFlags2; // ECF_xxx
  uint8_t m_bVFlags; // ECVF_xxx
  THRESH_DATA m_ThreshData;
  uint8_t m_EvseState;
  uint8_t m_PrevEvseState;
  uint8_t m_TmpEvseState;
  unsigned long m_TmpEvseStateStart;
  uint8_t m_CurrentCapacity; // max amps we can output
  unsigned long m_ChargeStartTimeMS;
  unsigned long m_ChargeOffTimeMS;
  time_t m_ChargeStartTime;
  time_t m_ChargeOffTime;
  time_t m_ElapsedChargeTime;
  time_t m_ElapsedChargeTimePrev;

#ifdef ADVPWR
// Define the Power states read from the L1 and L2 test lines
// 00 = both, 01 = L1on, 10 = L2on, 11 = none ( active low )
  enum {both, L1on, L2on, none}
  PowerSTATE;
// Define Service States UD = undefined state, L1 = level 1, L2 = level 2, OG = open ground, SR = stuck Relay, FG = Failed GFI
  enum { UD, L1, L2, OG, SR, FG} 
  SVCSTATE;

  uint8_t doPost();
#endif // ADVPWR
  void chargingOn();
  void chargingOff();
  uint8_t chargingIsOn() { return m_bVFlags & ECVF_CHARGING_ON; }
  void setFlags(uint8_t flags) { 
    m_bFlags |= flags; 
  }
  void clrFlags(uint8_t flags) { 
    m_bFlags &= ~flags; 
  }
  void setFlags2(uint8_t flags) { 
    m_bFlags2 |= flags; 
  }
  void clrFlags2(uint8_t flags) { 
    m_bFlags2 &= ~flags; 
  }

public:
  J1772EVSEController();
  void Init();
  void Update(); // read sensors
  void Enable();
  void Disable();
  void LoadThresholds();

  uint8_t GetFlags() { return m_bFlags; }
  uint8_t GetFlags2() { return m_bFlags2; }
  uint8_t GetState() { 
    return m_EvseState; 
  }
  uint8_t GetPrevState() { 
    return m_PrevEvseState; 
  }
  int StateTransition() { 
    return (m_EvseState != m_PrevEvseState) ? 1 : 0; 
  }
  uint8_t GetCurrentCapacity() { 
    return m_CurrentCapacity; 
  }
  int SetCurrentCapacity(uint8_t amps,uint8_t updatepwm=0);
  //int GetCurrentReading() { return m_CurrentReading; }
  //float GetCurrentAmps();
  time_t GetElapsedChargeTime() { 
    return m_ElapsedChargeTime; 
  }
  time_t GetElapsedChargeTimePrev() { 
    return m_ElapsedChargeTimePrev; 
  }
  time_t GetChargeOffTime() { 
    return m_ChargeOffTime; 
  }
  void Calibrate(PCALIB_DATA pcd);
  uint8_t GetCurSvcLevel() { 
    return (m_bFlags & ECF_L2) ? 2 : 1; 
  }
  void SetSvcLevel(uint8_t svclvl);
  PTHRESH_DATA GetThreshData() { 
    return &m_ThreshData; 
  }
  uint8_t DiodeCheckEnabled() { 
    return (m_bFlags & ECF_DIODE_CHK_DISABLED) ? 0 : 1;
  }
  void EnableDiodeCheck(uint8_t tf);
  uint8_t VentReqEnabled() { 
    return (m_bFlags & ECF_VENT_REQ_DISABLED) ? 0 : 1;
  }
  void EnableVentReq(uint8_t tf);
#ifdef ADVPWR
  uint8_t GndChkEnabled() { 
    return (m_bFlags & ECF_GND_CHK_DISABLED) ? 0 : 1;
  }
  void EnableGndChk(uint8_t tf);
  void EnableStuckRelayChk(uint8_t tf);
  uint8_t StuckRelayChkEnabled() { 
    return (m_bFlags & ECF_STUCK_RELAY_CHK_DISABLED) ? 0 : 1;
  }
  uint8_t AutoSvcLevelEnabled() { return (m_bFlags & ECF_AUTO_SVC_LEVEL_DISABLED) ? 0 : 1; }
  void EnableAutoSvcLevel(uint8_t tf);
  void SetNoGndTripped();
  uint8_t NoGndTripped() { return m_bVFlags & ECVF_NOGND_TRIPPED; }
#endif // ADVPWR
#ifdef GFI
  void SetGfiTripped();
  uint8_t GfiTripped() { return m_bVFlags & ECVF_GFI_TRIPPED; }
#ifdef GFI_SELFTEST
  uint8_t GfiSelfTestEnabled() {
    return (m_bFlags2 & ECF2_GFI_TEST_DISABLED) ? 0 : 1;
  }
  void EnableGfiTest(uint8_t tf);
#endif
#endif // GFI
  uint8_t SerDbgEnabled() { 
    return (m_bFlags & ECF_SERIAL_DBG) ? 1 : 0;
  }
  // Function to suppport Auto Start feature - GoldServe
#ifdef MANUALSTART
  void EnableAutoStart(uint8_t tf);
  uint8_t AutoStartEnabled() { 
    return (m_bFlags & ECF_AUTO_START_DISABLED) ? 0 : 1;
  }
#endif //ifdef MANUALSTART
  void EnableSerDbg(uint8_t tf);
};

#ifdef BTN_MENU
#define BTN_STATE_OFF   0
#define BTN_STATE_SHORT 1 // short press
#define BTN_STATE_LONG  2 // long press
class Btn {
  uint8_t buttonState;
  long lastDebounceTime;  // the last time the output pin was toggled
  
public:
  Btn();
  void init();

  void read();
  uint8_t shortPress();
  uint8_t longPress();
};


class Menu {
public:
  prog_char *m_Title;
  uint8_t m_CurIdx;
  
  void init(const char *firstitem);

  Menu();

  virtual void Init() = 0;
  virtual void Next() = 0;
  virtual Menu *Select() = 0;
};

class SetupMenu : public Menu {
public:
  SetupMenu();
  void Init();
  void Next();
  Menu *Select();
};

class SvcLevelMenu : public Menu {
public:
  SvcLevelMenu();
  void Init();
  void Next();
  Menu *Select();
};

class MaxCurrentMenu  : public Menu {
  uint8_t m_MaxCurrent;
  uint8_t m_MaxIdx;
  uint8_t *m_MaxAmpsList;
public:
  MaxCurrentMenu();
  void Init();
  void Next();
  Menu *Select();
};

class DiodeChkMenu : public Menu {
public:
  DiodeChkMenu();
  void Init();
  void Next();
  Menu *Select();
};

#ifdef GFI_SELFTEST
class GfiTestMenu : public Menu {
public:
  GfiTestMenu();
  void Init();
  void Next();
  Menu *Select();
};
#endif

class VentReqMenu : public Menu {
public:
  VentReqMenu();
  void Init();
  void Next();
  Menu *Select();
};


#ifdef ADVPWR
class GndChkMenu : public Menu {
public:
  GndChkMenu();
  void Init();
  void Next();
  Menu *Select();
};


#endif // ADVPWR
class ResetMenu : public Menu {
public:
  ResetMenu();
  void Init();
  void Next();
  Menu *Select();
};


#ifdef AUTOSTART_MENU
class AutoStartMenu : public Menu {
public:
  AutoStartMenu();
  void Init();
  void Next();
  Menu *Select();
};
#endif //#ifdef AUTOSTART_MENU

#ifdef DELAYTIMER
class RTCMenu : public Menu {
public:
  RTCMenu();
  void Init();
  void Next();
  Menu *Select();
};

class RTCMenuMonth : public Menu {
public:
  RTCMenuMonth();
  void Init();
  void Next();
  Menu *Select();
};

class RTCMenuDay : public Menu {
public:
  RTCMenuDay();
  void Init();
  void Next();
  Menu *Select();
};
class RTCMenuYear : public Menu {
public:
  RTCMenuYear();
  void Init();
  void Next();
  Menu *Select();
};
class RTCMenuHour : public Menu {
public:
  RTCMenuHour();
  void Init();
  void Next();
  Menu *Select();
};
class RTCMenuMinute : public Menu {
public:
  RTCMenuMinute();
  void Init();
  void Next();
  Menu *Select();
};
class DelayMenu : public Menu {
public:
  DelayMenu();
  void Init();
  void Next();
  Menu *Select();
};
class DelayMenuEnableDisable : public Menu {
public:
  DelayMenuEnableDisable();
  void Init();
  void Next();
  Menu *Select();
};
class DelayMenuStartHour : public Menu {
public:
  DelayMenuStartHour();
  void Init();
  void Next();
  Menu *Select();
};
class DelayMenuStartMin : public Menu {
public:
  DelayMenuStartMin();
  void Init();
  void Next();
  Menu *Select();
};
class DelayMenuStopHour : public Menu {
public:
  DelayMenuStopHour();
  void Init();
  void Next();
  Menu *Select();
};
class DelayMenuStopMin : public Menu {
public:
  DelayMenuStopMin();
  void Init();
  void Next();
  Menu *Select();
};
#endif //ifdef DELAYTIMER
class BtnHandler {
  Btn m_Btn;
  Menu *m_CurMenu;
public:
  BtnHandler();
  void init() { m_Btn.init(); }
  void ChkBtn();
  uint8_t InMenu() { return (m_CurMenu == NULL) ? 0 : 1; }
};

prog_char g_psSetup[] PROGMEM = "Setup";
prog_char g_psSvcLevel[] PROGMEM = "Service Level";
prog_char g_psMaxCurrent[] PROGMEM = "Max Current";
prog_char g_psDiodeCheck[] PROGMEM = "Diode Check";
prog_char g_psVentReqChk[] PROGMEM = "Vent Req'd Check";
#ifdef ADVPWR
prog_char g_psGndChk[] PROGMEM = "Ground Check";
#endif // ADVPWR
#ifdef GFI_SELFTEST
prog_char g_psGfiTest[] PROGMEM = "GFI Self Test";
#endif
prog_char g_psReset[] PROGMEM = "Reset";
prog_char g_psExit[] PROGMEM = "Exit";
// Add additional strings - GoldServe
#ifdef AUTOSTART_MENU
prog_char g_psAutoStart[] PROGMEM = "Auto Start";
#endif //#ifdef AUTOSTART_MENU
#ifdef DELAYTIMER
prog_char g_psRTC[] PROGMEM = "Date/Time";
prog_char g_psRTC_Month[] PROGMEM = "Set Month";
prog_char g_psRTC_Day[] PROGMEM = "Set Day";
prog_char g_psRTC_Year[] PROGMEM = "Set Year";
prog_char g_psRTC_Hour[] PROGMEM = "Set Hour";
prog_char g_psRTC_Minute[] PROGMEM = "Set Minute";
prog_char g_psDelayTimer[] PROGMEM = "Delay Timer";
//prog_char g_psDelayTimerEnableDisable[] PROGMEM = "Enable/Disable Timer?";
prog_char g_psDelayTimerStartHour[] PROGMEM = "Set Start Hour";
prog_char g_psDelayTimerStartMin[] PROGMEM = "Set Start Min";
prog_char g_psDelayTimerStopHour[] PROGMEM = "Set Stop Hour";
prog_char g_psDelayTimerStopMin[] PROGMEM = "Set Stop Min";
char *g_sHHMMfmt = "%02d:%02d";
#endif

SetupMenu g_SetupMenu;
SvcLevelMenu g_SvcLevelMenu;
MaxCurrentMenu g_MaxCurrentMenu;
DiodeChkMenu g_DiodeChkMenu;
#ifdef GFI_SELFTEST
GfiTestMenu g_GfiTestMenu;
#endif
VentReqMenu g_VentReqMenu;
#ifdef ADVPWR
GndChkMenu g_GndChkMenu;
#endif // ADVPWR
ResetMenu g_ResetMenu;
// Instantiate additional Menus - GoldServe
#ifdef AUTOSTART_MENU
AutoStartMenu g_AutoStartMenu;
#endif //#ifdef AUTOSTART_MENU
#ifdef DELAYTIMER
RTCMenu g_RTCMenu;
RTCMenuMonth g_RTCMenuMonth;
RTCMenuDay g_RTCMenuDay;
RTCMenuYear g_RTCMenuYear;
RTCMenuHour g_RTCMenuHour;
RTCMenuMinute g_RTCMenuMinute;
DelayMenu g_DelayMenu;
DelayMenuEnableDisable g_DelayMenuEnableDisable;
DelayMenuStartHour g_DelayMenuStartHour;
DelayMenuStopHour g_DelayMenuStopHour;
DelayMenuStartMin g_DelayMenuStartMin;
DelayMenuStopMin g_DelayMenuStopMin;
#endif

BtnHandler g_BtnHandler;
#endif // BTN_MENU

// Start Delay Timer class definition - GoldServe
#ifdef DELAYTIMER
class DelayTimer {
  uint8_t m_DelayTimerEnabled;
  uint8_t m_StartTimerHour;
  uint8_t m_StartTimerMin;
  uint8_t m_StopTimerHour;
  uint8_t m_StopTimerMin;
  uint8_t m_CurrHour;
  uint8_t m_CurrMin;
  uint8_t m_CheckNow;
public:
  DelayTimer(){
    m_CheckNow = 1; //Check as soon as the EVSE initializes
  };
  void Init();
  void CheckTime();
  void CheckNow(){
    m_CheckNow = 1;
  };
  void Enable();
  void Disable();
  uint8_t IsTimerEnabled(){
    return m_DelayTimerEnabled; 
  };
  uint8_t GetStartTimerHour(){
    return m_StartTimerHour; 
  };
  uint8_t GetStartTimerMin(){
    return m_StartTimerMin; 
  };
  uint8_t GetStopTimerHour(){
    return m_StopTimerHour; 
  };
  uint8_t GetStopTimerMin(){
    return m_StopTimerMin; 
  };
  void SetStartTimer(uint8_t hour, uint8_t min){
    m_StartTimerHour = hour;
    m_StartTimerMin = min;
    EEPROM.write(EOFS_TIMER_START_HOUR, m_StartTimerHour);
    EEPROM.write(EOFS_TIMER_START_MIN, m_StartTimerMin);
    SaveSettings();
  };
  void SetStopTimer(uint8_t hour, uint8_t min){
    m_StopTimerHour = hour;
    m_StopTimerMin = min;
    EEPROM.write(EOFS_TIMER_STOP_HOUR, m_StopTimerHour);
    EEPROM.write(EOFS_TIMER_STOP_MIN, m_StopTimerMin);
    SaveSettings();
  };
  uint8_t IsTimerValid(){
     if (m_StartTimerHour || m_StartTimerMin || m_StopTimerHour || m_StopTimerMin){ // Check not all equal 0
       if ((m_StartTimerHour == m_StopTimerHour) && (m_StartTimerMin == m_StopTimerMin)){ // Check start time not equal to stop time
         return 0;
       } else {
         return 1;
       }
     } else {
       return 0; 
     }
  };
  void PrintTimerIcon();
};
#endif
// -- end class definitions
//-- begin global variables

char g_sTmp[64];

THRESH_DATA g_DefaultThreshData = {875,780,690,0,260};
J1772EVSEController g_EvseController;
OnboardDisplay g_OBD;

// Instantiate RTC and Delay Timer - GoldServe
#ifdef DELAYTIMER
RTC_DS1307 g_RTC;
DelayTimer g_DelayTimer;
// Start variables to support RTC and Delay Timer - GoldServe
uint16_t g_year;
uint8_t g_month;
uint8_t g_day;
uint8_t g_hour;
uint8_t g_min;
uint8_t sec = 0;
DateTime g_CurrTime;
#endif

#ifdef SERIALCLI
CLI g_CLI;

prog_char g_psEnabled[] PROGMEM = "enabled";
prog_char g_psDisabled[] PROGMEM = "disabled";
#endif // SERIALCLI

#ifdef LCD16X2
#ifdef ADVPWR
prog_char g_psPwrOn[] PROGMEM = "Power On";
prog_char g_psSelfTest[] PROGMEM = "Self Test";
prog_char g_psAutoDetect[] PROGMEM = "Auto Detect";
prog_char g_psLevel1[] PROGMEM = "Svc Level: L1";
prog_char g_psLevel2[] PROGMEM = "Svc Level: L2";
prog_char g_psStuckRelay[] PROGMEM = "--Stuck Relay--";
prog_char g_psEarthGround[] PROGMEM = "--Earth Ground--";
//prog_char g_psTestPassed[] PROGMEM = "Test Passed";
prog_char g_psTestFailed[] PROGMEM = "TEST FAILED";
#endif // ADVPWR
prog_char g_psEvseError[] PROGMEM =  "EVSE ERROR";
prog_char g_psVentReq[] PROGMEM = "VENT REQUIRED";
prog_char g_psDiodeChkFailed[] PROGMEM = "DIODE CHK FAILED";
prog_char g_psGfciFault[] PROGMEM = "GFCI FAULT";
prog_char g_psNoGround[] PROGMEM = "NO GROUND";
prog_char g_psEStuckRelay[] PROGMEM = "STUCK RELAY";
prog_char g_psStopped[] PROGMEM = "Stopped";
prog_char g_psEvConnected[] PROGMEM = "EV Connected";
prog_char g_psEvNotConnected[] PROGMEM = "EV Not Connected";
#endif // LCD16X2

//-- end global variables

void EvseReset();

void SaveSettings()
{
  // n.b. should we add dirty bits so we only write the changed values? or should we just write them on the fly when necessary?
  EEPROM.write((g_EvseController.GetCurSvcLevel() == 1) ? EOFS_CURRENT_CAPACITY_L1 : EOFS_CURRENT_CAPACITY_L2,(byte)g_EvseController.GetCurrentCapacity());
  EEPROM.write(EOFS_FLAGS,g_EvseController.GetFlags());
}

#ifdef SERIALCLI
CLI::CLI()
{
  m_CLIstrCount = 0; 
  m_strBuf = g_sTmp;
  m_strBufLen = sizeof(g_sTmp);
}

void CLI::info()
{
  println_P(PSTR("OpenEVSE")); // CLI print prompt when serial is ready
  print_P(PSTR("Software - Open EVSE V")); //CLI info
  println_P(VERSTR);
  printlnn();
}

void CLI::Init()
{
  info();
  println_P(PSTR("type help for command list"));
  print_P(PSTR("OpenEVSE> ")); // CLI Prompt
  flush();

}

uint8_t CLI::getInt()
{
  uint8_t c;
  uint8_t num = 0;

  do {
    c = Serial.read(); // read the byte
    if ((c >= '0') && (c <= '9')) {
      num = (num * 10) + c - '0';
    }
  } while (c != 13);
  return num;
}

void CLI::printlnn()
{
  println("");
}

prog_char g_pson[] PROGMEM = "on";
void CLI::getInput()
{
  int currentreading;
  uint8_t amp;
  if (Serial.available()) { // if byte(s) are available to be read
    char inbyte = (char) Serial.read(); // read the byte
    Serial.print(inbyte);
    if (inbyte != 13) { // CR
      if (((inbyte >= 'a') && (inbyte <= 'z')) || ((inbyte >= '0') && (inbyte <= '@') || (inbyte == ' ')) ) { //sar - allow ?
	m_CLIinstr[m_CLIstrCount] = inbyte;
	m_CLIstrCount++;
      }
      else if (m_CLIstrCount && ((inbyte == 8) || (inbyte == 127))) {
	m_CLIstrCount--;
      }
    }

    if ((inbyte == 13) || (m_CLIstrCount == CLI_BUFLEN-1)) { // if enter was pressed or max chars reached
      m_CLIinstr[m_CLIstrCount] = '\0';
      printlnn(); // print a newline
      
      if (strcmp_P(m_CLIinstr, PSTR("show")) == 0){ //if match SHOW 
        info();
        
        println_P(PSTR("Settings"));
	print_P(PSTR("Service level: L"));
	Serial.println((int)g_EvseController.GetCurSvcLevel()); 
        print_P(PSTR("Current capacity (Amps): "));
        Serial.println((int)g_EvseController.GetCurrentCapacity()); 
        print_P(PSTR("Min Current Capacity: "));
        Serial.println(MIN_CURRENT_CAPACITY);
        print_P(PSTR("Max Current Capacity: "));
	Serial.println((g_EvseController.GetCurSvcLevel() == 2) ? MAX_CURRENT_CAPACITY_L2 : MAX_CURRENT_CAPACITY_L1);
	print_P(PSTR("Vent Required: "));
	println_P(g_EvseController.VentReqEnabled() ? g_psEnabled : g_psDisabled);
         print_P(PSTR("Diode Check: "));
	println_P(g_EvseController.DiodeCheckEnabled() ? g_psEnabled : g_psDisabled);

#ifdef ADVPWR
	print_P(PSTR("Ground Check: "));
	println_P(g_EvseController.GndChkEnabled() ? g_psEnabled : g_psDisabled);
	print_P(PSTR("Stuck Relay Check: "));
	println_P(g_EvseController.StuckRelayChkEnabled() ? g_psEnabled : g_psDisabled);
#endif // ADVPWR           
        // Option to disable auto start - GoldServe
#ifdef MANUALSTART
        print_P(PSTR("Auto Start: "));
	println_P(g_EvseController.AutoStartEnabled() ? g_psEnabled : g_psDisabled);
#endif //#ifdef MANUALSTART
        // Start Delay Timer feature - GoldServe
#ifdef DELAYTIMER
        print_P(PSTR("Delay Timer: "));
        if (g_DelayTimer.IsTimerEnabled()){
          println_P(g_psEnabled);
        } else {
          println_P(g_psDisabled);
        }
        print_P(PSTR("Start Time: "));
        Serial.print(g_DelayTimer.GetStartTimerHour(), DEC);
        print_P(PSTR(" hour "));
        Serial.print(g_DelayTimer.GetStartTimerMin(), DEC);
        println_P(PSTR(" min"));
        print_P(PSTR("End Time: "));
        Serial.print(g_DelayTimer.GetStopTimerHour(), DEC);
        print_P(PSTR(" hour "));
        Serial.print(g_DelayTimer.GetStopTimerMin(), DEC);
        println_P(PSTR(" min"));
        print_P(PSTR("System Date/Time: "));
        g_CurrTime = g_RTC.now();
        Serial.print(g_CurrTime.year(), DEC);
        Serial.print('/');
        Serial.print(g_CurrTime.month(), DEC);
        Serial.print('/');
        Serial.print(g_CurrTime.day(), DEC);
        Serial.print(' ');
        Serial.print(g_CurrTime.hour(), DEC);
        Serial.print(':');
        Serial.print(g_CurrTime.minute(), DEC);
        Serial.print(':');
        Serial.print(g_CurrTime.second(), DEC);
        // End Delay Timer feature - GoldServe
#endif //#ifdef DELAYTIMER
      } 
      else if ((strcmp_P(m_CLIinstr, PSTR("help")) == 0) || (strcmp_P(m_CLIinstr, PSTR("?")) == 0)){ // string compare
        println_P(PSTR("Help Commands"));
        printlnn();
        println_P(PSTR("help - Display commands")); // print to the terminal
        println_P(PSTR("set  - Change settings"));
        println_P(PSTR("show - Display settings/values"));
        println_P(PSTR("save - Write to EEPROM"));
        // Start Delay Timer feature - GoldServe
#ifdef DELAYTIMER
        println_P(PSTR("dt - Date/Time commands"));
        println_P(PSTR("timer - Delay timer commands"));
#endif //#ifdef DELAYTIMER
        // End Delay Timer feature - GoldServe
      } 
      else if (strcmp_P(m_CLIinstr, PSTR("set")) == 0) { // string compare
        println_P(PSTR("Set Commands - Usage: set amp"));
        printlnn();
        println_P(PSTR("amp  - Set EVSE Current Capacity")); // print to the terminal
	println_P(PSTR("vntreq on/off - enable/disable vent required state"));
        println_P(PSTR("diochk on/off - enable/disable diode check"));

#ifdef ADVPWR
	println_P(PSTR("gndchk on/off - turn ground check on/off"));
	println_P(PSTR("rlychk on/off - turn stuck relay check on/off"));
#endif // ADVPWR
        // Start Delay Timer feature - GoldServe
#ifdef MANUALSTART
        println_P(PSTR("autostart on/off - enable/disable autostart"));
#endif //#ifdef MANUALSTART
        // End Delay Timer feature - GoldServe
	println_P(PSTR("sdbg on/off - turn serial debugging on/off"));
      }
      else if (strncmp_P(m_CLIinstr, PSTR("set "),4) == 0) {
	char *p = m_CLIinstr + 4;
	if (!strncmp_P(p,PSTR("sdbg "),5)) {
	  p += 5;
	  print_P(PSTR("serial debugging "));
	  if (!strcmp_P(p,g_pson)) {
	    g_EvseController.EnableSerDbg(1);
	    println_P(g_psEnabled);
	  }
	  else {
	    g_EvseController.EnableSerDbg(0);
	    println_P(g_psDisabled);
	  }
	}
	else if (!strncmp_P(p,PSTR("vntreq "),7)) {
	  p += 7;
	  print_P(PSTR("vent required "));
	  if (!strcmp_P(p,g_pson)) {
	    g_EvseController.EnableVentReq(1);
	    println_P(g_psEnabled);
	  }
	  else {
	    g_EvseController.EnableVentReq(0);
	    println_P(g_psDisabled);
	  }
	}
            else if (!strncmp_P(p,PSTR("diochk "),7)) {
	  p += 7;
	  print_P(PSTR("diode check "));
	  if (!strcmp_P(p,g_pson)) {
	    g_EvseController.EnableDiodeCheck(1);
	    println_P(g_psEnabled);
	  }
	  else {
	    g_EvseController.EnableDiodeCheck(0);
	    println_P(g_psDisabled);
	  }
	}
        // Start Delay Timer feature - GoldServe
#ifdef MANUALSTART
        else if (!strncmp_P(p,PSTR("autostart "),10)) {
	  p += 10;
	  print_P(PSTR("autostart "));
	  if (!strcmp_P(p,g_pson)) {
	    g_EvseController.EnableAutoStart(1);
	    println_P(g_psEnabled);
	  }
	  else {
	    g_EvseController.EnableAutoStart(0);
	    println_P(g_psDisabled);
	  }
	}
#endif //#ifdef MANUALSTART
        // End Delay Timer feature - GoldServe
#ifdef ADVPWR
	else if (!strncmp_P(p,PSTR("gndchk "),7)) {
	  p += 7;
	  print_P(PSTR("ground check "));
	  if (!strcmp_P(p,g_pson)) {
	    g_EvseController.EnableGndChk(1);
	    println_P(g_psEnabled);
	  }
	  else {
	    g_EvseController.EnableGndChk(0);
	    println_P(g_psDisabled);
	  }
	}
	else if (!strncmp_P(p,PSTR("rlychk "),7)) {
	  p += 7;
	  print_P(PSTR("stuck relay check "));
	  if (!strcmp_P(p,g_pson)) {
	    g_EvseController.EnableStuckRelayChk(1);
	    println_P(g_psEnabled);
	  }
	  else {
	    g_EvseController.EnableStuckRelayChk(0);
	    println_P(g_psDisabled);
	  }
	}
#endif // ADVPWR
	else if (!strcmp_P(p,PSTR("amp"))){ // string compare
	  println_P(PSTR("WARNING - Do not set higher than 80% of breaker value"));
	  printlnn();
	  print_P(PSTR("Enter amps ("));
	  Serial.print(MIN_CURRENT_CAPACITY);
	  print_P(PSTR("-"));
	  Serial.print((g_EvseController.GetCurSvcLevel()  == 1) ? MAX_CURRENT_CAPACITY_L1 : MAX_CURRENT_CAPACITY_L2);
	  print_P(PSTR("): "));
	  amp = getInt();
	  Serial.println((int)amp);
	  if(g_EvseController.SetCurrentCapacity(amp,1)) {
	    println_P(PSTR("Invalid Setting"));
	  }
	  
	  print_P(PSTR("Max current: ")); // print to the terminal
	  Serial.print((int)g_EvseController.GetCurrentCapacity());
	  print_P(PSTR("A"));
	} 
	else {
	  goto unknown;
	}
      }
      else if (strcmp_P(m_CLIinstr, PSTR("save")) == 0){ // string compare
        println_P(PSTR("Saving to EEPROM")); // print to the terminal
        SaveSettings();
      } 
      // Start Delay Timer feature - GoldServe
#ifdef DELAYTIMER
      else if (strncmp_P(m_CLIinstr, PSTR("dt"), 2) == 0){ // string compare
        char *p = m_CLIinstr + 3;
        
        if (strncmp_P(p,PSTR("set"),3) == 0) {
	  p += 4;
	  println_P(PSTR("Set Date/Time (mm/dd/yy hh:mm)"));
          print_P(PSTR("Month (mm): "));
          g_month = getInt();
          Serial.println(g_month);
          print_P(PSTR("Day (dd): "));
          g_day = getInt();
          Serial.println(g_day);
          print_P(PSTR("Year (yy): "));
          g_year = getInt();
          Serial.println(g_year);
          print_P(PSTR("Hour (hh): "));
          g_hour = getInt();
          Serial.println(g_hour);
          print_P(PSTR("Minute (mm): "));
          g_min = getInt();
          Serial.println(g_min);
          
          if (g_month + g_day + g_year + g_hour + g_min) {
            g_RTC.adjust(DateTime(g_year, g_month, g_day, g_hour, g_min, 0));
            println_P(PSTR("Date/Time Set"));
          } else {
            println_P(PSTR("Date/Time NOT Set")); 
          }
	}
        else {
          g_CurrTime = g_RTC.now();
          Serial.print(g_CurrTime.year(), DEC);
          Serial.print('/');
          Serial.print(g_CurrTime.month(), DEC);
          Serial.print('/');
          Serial.print(g_CurrTime.day(), DEC);
          Serial.print(' ');
          Serial.print(g_CurrTime.hour(), DEC);
          Serial.print(':');
          Serial.print(g_CurrTime.minute(), DEC);
          Serial.print(':');
          Serial.print(g_CurrTime.second(), DEC);
          Serial.println();
          println_P(PSTR("Use 'dt set' to set the system date/time"));
	}
        
      }
      else if (strncmp_P(m_CLIinstr, PSTR("timer"), 5) == 0){ // string compare
        char *p = m_CLIinstr + 6;
        
        if (strncmp_P(p,PSTR("set start"),9) == 0) {
          println_P(PSTR("Set Start Time (hh:mm)"));
          print_P(PSTR("Hour (hh): "));
          g_hour = getInt();
          Serial.println(g_hour);
          print_P(PSTR("Minute (mm): "));
          g_min = getInt();
          Serial.println(g_min);
          g_DelayTimer.SetStartTimer(g_hour, g_min);
        } else if (strncmp_P(p,PSTR("set stop"),8) == 0) {
          println_P(PSTR("Set Stop Time (hh:mm)"));
          print_P(PSTR("Hour (hh): "));
          g_hour = getInt();
          Serial.println(g_hour);
          print_P(PSTR("Minute (mm): "));
          g_min = getInt();
          Serial.println(g_min);
          g_DelayTimer.SetStopTimer(g_hour, g_min);
        } else if (strncmp_P(p,PSTR("enable"),9) == 0) {
          println_P(PSTR("Delay timer enabled, autostart disabled"));
          g_DelayTimer.Enable();
        } else if (strncmp_P(p,PSTR("disable"),9) == 0) {
          println_P(PSTR("Delay timer disabled, autostart enabled"));
          g_DelayTimer.Disable();
        } else {
          print_P(PSTR("Delay Timer: "));
          if (g_DelayTimer.IsTimerEnabled()){
            println_P(PSTR("Enabled"));
          } else {
            println_P(PSTR("Disabled"));
          }
          print_P(PSTR("Start Time: "));
          Serial.print(g_DelayTimer.GetStartTimerHour(), DEC);
          print_P(PSTR(" hour "));
          Serial.print(g_DelayTimer.GetStartTimerMin(), DEC);
          println_P(PSTR(" min"));
          print_P(PSTR("End Time: "));
          Serial.print(g_DelayTimer.GetStopTimerHour(), DEC);
          print_P(PSTR(" hour "));
          Serial.print(g_DelayTimer.GetStopTimerMin(), DEC);
          println_P(PSTR(" min"));
          if (!g_DelayTimer.IsTimerValid()){
            println_P(PSTR("Start and Stop times can not be the same!"));  
          }
          println_P(PSTR(""));
          println_P(PSTR("Use 'timer enable/disable' to enable/disable timer function"));
          println_P(PSTR("Use 'timer set start/stop' to set timer start/stop times"));
        }
      }
#endif //#ifdef DELAYTIMER
      // End Delay Timer feature - GoldServe
      else { // if the input text doesn't match any defined above
      unknown:
        println_P(PSTR("Unknown Command -- type help for command list")); // echo back to the terminal
      } 
      printlnn();
      print_P(PSTR("OpenEVSE> "));
      g_CLI.flush();
      m_CLIstrCount = 0; // get ready for new input... reset strCount
      m_CLIinstr[0] = '\0'; // set to null to erase it
    }
  }
}

void CLI::println_P(prog_char *s)
{
  strncpy_P(m_strBuf,s,m_strBufLen);
  println(m_strBuf);
}

void CLI::print_P(prog_char *s)
{
  strncpy_P(m_strBuf,s,m_strBufLen);
  print(m_strBuf);
}

#endif // SERIALCLI

OnboardDisplay::OnboardDisplay()
#if defined(I2CLCD) || defined(RGBLCD)
  : m_Lcd(LCD_I2C_ADDR,1)
#endif

// for WhiteLCD - all colors defined as white
#ifdef WHITELCD
#undef RED
#define RED 0x7
#undef YELLOW
#define YELLOW 0x7
#undef GREEN
#define GREEN 0x7
#undef BLUE
#define BLUE 0x7
#undef TEAL
#define TEAL 0x7
#undef VIOLET
#define VIOLET 0x7
#undef WHITE
#define WHITE 0x7 
#endif // WHITELCD

{
} 


#ifdef DELAYTIMER
  // custom icons - GoldServe
  prog_char CustomChar_0[8] PROGMEM = {0x0,0xe,0x15,0x17,0x11,0xe,0x0,0x0};
  prog_char CustomChar_1[8] PROGMEM = {0x0,0x0,0xe,0xe,0xe,0x0,0x0,0x0};
  prog_char CustomChar_2[8] PROGMEM = {0x0,0x8,0xc,0xe,0xc,0x8,0x0,0x0};
#endif // DELAYTIMER
void OnboardDisplay::Init()
{
  pinMode (GREEN_LED_PIN, OUTPUT);
  pinMode (RED_LED_PIN, OUTPUT);

  SetGreenLed(LOW);
  SetRedLed(LOW);
  
#ifdef LCD16X2
  LcdBegin(LCD_MAX_CHARS_PER_LINE, 2);

#ifdef DELAYTIMER
  memcpy_P(g_sTmp,CustomChar_0,8);
  m_Lcd.createChar(0, (uint8_t*)g_sTmp);
  memcpy_P(g_sTmp,CustomChar_1,8);
  m_Lcd.createChar(1, (uint8_t*)g_sTmp);
  memcpy_P(g_sTmp,CustomChar_2,8);
  m_Lcd.createChar(2, (uint8_t*)g_sTmp);
  m_Lcd.clear(); // need this to activate the custom chars
#endif //#ifdef DELAYTIMER

  LcdPrint_P(0,PSTR("Open EVSE       "));
  LcdPrint_P(0,1,PSTR("Version "));
  LcdPrint_P(VERSTR);
  LcdPrint_P(PSTR("   "));
  delay(800);
#endif //#ifdef LCD16X2
}


void OnboardDisplay::SetGreenLed(uint8_t state)
{
  digitalWrite(GREEN_LED_PIN,state);
}

void OnboardDisplay::SetRedLed(uint8_t state)
{
  digitalWrite(RED_LED_PIN,state);
}

#ifdef LCD16X2
void OnboardDisplay::LcdPrint_P(const prog_char *s)
{
  if (m_Lcd.LcdDetected()) {
    strcpy_P(m_strBuf,s);
    m_Lcd.print(m_strBuf);
  }
}

void OnboardDisplay::LcdPrint_P(int y,const prog_char *s)
{
  strcpy_P(m_strBuf,s);
  LcdPrint(y,m_strBuf);
}

void OnboardDisplay::LcdPrint_P(int x,int y,const prog_char *s)
{
  strcpy_P(m_strBuf,s);
  LcdPrint(x,y,m_strBuf);
}

void OnboardDisplay::LcdMsg_P(const prog_char *l1,const prog_char *l2)
{
  LcdPrint_P(0,l1);
  LcdPrint_P(1,l2);
}


// print at (0,y), filling out the line with trailing spaces
void OnboardDisplay::LcdPrint(int y,const char *s)
{
  if (m_Lcd.LcdDetected()) {
    m_Lcd.setCursor(0,y);
    char ss[LCD_MAX_CHARS_PER_LINE+1];
// n.b the 16 in the string below needs to be adjusted if LCD_MAX_CHARS_PER_LINE != 16
    sprintf(ss,"%-16s",s);
    ss[LCD_MAX_CHARS_PER_LINE] = '\0';
    m_Lcd.print(ss);
   }
}

void OnboardDisplay::LcdMsg(const char *l1,const char *l2)
{
  LcdPrint(0,l1);
  LcdPrint(1,l2);
}
#endif // LCD16X2

char g_sRdyLAstr[] = "L%d:%dA";
prog_char g_psReady[] PROGMEM = "Ready";
prog_char g_psCharging[] PROGMEM = "Charging";
void OnboardDisplay::Update()
{
  uint8_t curstate = g_EvseController.GetState();
  uint8_t svclvl = g_EvseController.GetCurSvcLevel();
  int i;

#if defined(DELAYTIMER) && defined(LCD16X2)
  g_CurrTime = g_RTC.now();
#endif //#ifdef DELAYTIMER

  if (g_EvseController.StateTransition()) {
    // Optimize function call - GoldServe
    sprintf(g_sTmp,g_sRdyLAstr,(int)svclvl,(int)g_EvseController.GetCurrentCapacity());
    
    switch(curstate) {
    case EVSE_STATE_A: // not connected
      SetGreenLed(HIGH);
      SetRedLed(LOW);
      #ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(GREEN);
      // Display Timer and Stop Icon - GoldServe
      LcdClear();
      LcdSetCursor(0,0);
#ifdef DELAYTIMER
      g_DelayTimer.PrintTimerIcon();
#endif //#ifdef DELAYTIMER
      LcdPrint_P(g_psReady);
      LcdPrint(10,0,g_sTmp);  
      LcdPrint_P(1,g_psEvNotConnected);
      #endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_B: // connected/not charging
      SetGreenLed(HIGH);
      SetRedLed(HIGH);
      #ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(YELLOW);
      LcdClear();
      LcdSetCursor(0,0);
      // Display Timer and Stop Icon - GoldServe
#ifdef DELAYTIMER
      g_DelayTimer.PrintTimerIcon();
#endif //#ifdef DELAYTIMER
      LcdPrint_P(g_psReady);
      LcdPrint(10,0,g_sTmp);
      LcdPrint_P(1,g_psEvConnected);
      #endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_C: // charging
      SetGreenLed(LOW);
      SetRedLed(LOW);
      #ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(TEAL);
      LcdClear();
      LcdSetCursor(0,0);
      // Display Timer and Stop Icon - GoldServe
#ifdef DELAYTIMER
      g_DelayTimer.PrintTimerIcon();
#endif //#ifdef DELAYTIMER
      LcdPrint_P(g_psCharging);
      LcdPrint(10,0,g_sTmp);
      #endif //Adafruit RGB LCD
      // n.b. blue LED is on
      break;
    case EVSE_STATE_D: // vent required
      SetGreenLed(LOW);
      SetRedLed(HIGH);
      #ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      LcdMsg_P(g_psEvseError,g_psVentReq);
      #endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_DIODE_CHK_FAILED:
      SetGreenLed(LOW);
      SetRedLed(HIGH);
      #ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      LcdMsg_P(g_psEvseError,g_psDiodeChkFailed);
      #endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_GFCI_FAULT:
      SetGreenLed(LOW);
      SetRedLed(HIGH);
      #ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      LcdMsg_P(g_psEvseError,g_psGfciFault);
      #endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
     case EVSE_STATE_NO_GROUND:
      SetGreenLed(LOW);
      SetRedLed(HIGH);
      #ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      LcdMsg_P(g_psEvseError,g_psNoGround);
      #endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
     case EVSE_STATE_STUCK_RELAY:
      SetGreenLed(LOW);
      SetRedLed(HIGH);
      #ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      LcdMsg_P(g_psEvseError,g_psEStuckRelay);
      #endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_DISABLED:
      SetGreenLed(LOW);
      SetRedLed(HIGH);
#ifdef LCD16X2
      // Display Timer and Stop Icon - GoldServe
      //sprintf(g_sTmp,"L%d:%dA",(int)svclvl,(int)g_EvseController.GetCurrentCapacity());
      //g_DelayTimer.PrintTimerIcon();
      LcdSetBacklightColor(WHITE);
      LcdClear();
      LcdSetCursor(0,0);
      LcdPrint_P(g_psStopped);
      LcdPrint(10,0,g_sTmp);
#endif // LCD16X2
      break;
    default:
      SetGreenLed(LOW);
      SetRedLed(HIGH);
      // n.b. blue LED is off
    }
  }
#ifdef LCD16X2
  if (curstate == EVSE_STATE_C) {
    time_t elapsedTime = g_EvseController.GetElapsedChargeTime();
    if (elapsedTime != g_EvseController.GetElapsedChargeTimePrev()) {   
      int h = hour(elapsedTime);
      int m = minute(elapsedTime);
      int s = second(elapsedTime);
#ifdef DELAYTIMER
      g_CurrTime = g_RTC.now();
      sprintf(g_sTmp,"%02d:%02d:%02d   %02d:%02d",h,m,s,g_CurrTime.hour(),g_CurrTime.minute());
#else
      sprintf(g_sTmp,"%02d:%02d:%02d",h,m,s);
#endif //#ifdef DELAYTIMER
      LcdPrint(1,g_sTmp);
    }
  }
  // Display a new stopped LCD screen with Delay Timers enabled - GoldServe
#ifdef DELAYTIMER
#ifdef BTN_MENU
  else if (curstate == EVSE_STATE_DISABLED && !g_BtnHandler.InMenu()) {
#else
  else if (curstate == EVSE_STATE_DISABLED) {
#endif //#ifdef BTN_MENU
    LcdSetCursor(0,0);
    g_DelayTimer.PrintTimerIcon();
    LcdPrint_P(g_psStopped);
    g_CurrTime = g_RTC.now();
//    sprintf(g_sTmp,"%02d:%02d \0\1",g_CurrTime.hour(),g_CurrTime.minute());
    sprintf(g_sTmp,"%02d:%02d:%02d",g_CurrTime.hour(),g_CurrTime.minute(),g_CurrTime.second());
    LcdPrint(0,1,g_sTmp);
    if (g_DelayTimer.IsTimerEnabled()){
      LcdSetCursor(9,0);
      LcdWrite(0x2);
      LcdWrite(0x0);
      sprintf(g_sTmp,g_sHHMMfmt,g_DelayTimer.GetStartTimerHour(),g_DelayTimer.GetStartTimerMin());
      LcdPrint(11,0,g_sTmp);
      LcdSetCursor(9,1);
      LcdWrite(0x1);
      LcdWrite(0x0);
      sprintf(g_sTmp,g_sHHMMfmt,g_DelayTimer.GetStopTimerHour(),g_DelayTimer.GetStopTimerMin());
      LcdPrint(11,1,g_sTmp);
      } else {
        sprintf(g_sTmp,"L%d:%dA",(int)svclvl,(int)g_EvseController.GetCurrentCapacity());
        LcdPrint(10,0,g_sTmp);
    }
  }
#endif
#endif
}


#ifdef GFI

// interrupt service routing
void gfi_isr()
{
  g_EvseController.SetGfiTripped();
}

void Gfi::Init()
{
  Reset();
}

//RESET GFI LOGIC
void Gfi::Reset()
{
#ifdef WATCHDOG
  wdt_reset(); // pat the dog
#endif // WATCHDOG

  testInProgress = false;
  testSuccess = false;
  if (digitalRead(GFI_PIN) ) m_GfiFault = 1; // if interrupt pin is high, set fault
  else m_GfiFault = 0;
}

#ifdef GFI_SELFTEST

void Gfi::SelfTest()
{
  testInProgress = true;
  testSuccess = false;
  pinMode(GFI_TEST_PIN, OUTPUT);
  for(int i = 0; i < GFI_TEST_CYCLES; i++) {
    digitalWrite(GFI_TEST_PIN, HIGH);
    delayMicroseconds(GFI_PULSE_DURATION_MS);
    digitalWrite(GFI_TEST_PIN, LOW);
    delayMicroseconds(GFI_PULSE_DURATION_MS);
    if (testSuccess) break; // no need to keep trying.
  }
  delay(GFI_TEST_CLEAR_TIME);
  testInProgress = false;
}
#endif GFI_SELFTEST

#endif // GFI
//-- begin J1772Pilot

#define TOP ((F_CPU / 2000000) * 1000) // for 1KHz (=1000us period)
void J1772Pilot::Init()
{
#ifdef PAFC_PWM
  // set up Timer for phase & frequency correct PWM
  TCCR1A = 0;  // set up Control Register A
  ICR1 = TOP;
  // WGM13 -> select P&F mode CS10 -> prescaler = 1
  TCCR1B = _BV(WGM13) | _BV(CS10);
 
 #if (PILOT_PIN == 9)
  DDRB |= _BV(PORTB1);
  TCCR1A |= _BV(COM1A1);
 #else // PILOT_PIN == 10
  DDRB |= _BV(PORTB2);
  TCCR1A |= _BV(COM1B1);
 #endif // PILOT_PIN
#else // fast PWM
  pinMode(PILOT_PIN,OUTPUT);
  m_bit = digitalPinToBitMask(PILOT_PIN);
  m_port = digitalPinToPort(PILOT_PIN);
#endif

  SetState(PILOT_STATE_P12); // turns the pilot on 12V steady state
}


// no PWM pilot signal - steady state
// PILOT_STATE_P12 = steady +12V (EVSE_STATE_A - VEHICLE NOT CONNECTED)
// PILOT_STATE_N12 = steady -12V (EVSE_STATE_F - FAULT) 
void J1772Pilot::SetState(PILOT_STATE state)
{
#ifdef PAFC_PWM
  if (state == PILOT_STATE_P12) {
#if (PILOT_PIN == 9)
    OCR1A = TOP;
#else
    OCR1B = TOP;
#endif
  }
  else {
#if (PILOT_PIN == 9)
    OCR1A = 0;
#else
    OCR1B = 0;
#endif
  }
#else // fast PWM
  volatile uint8_t *out = portOutputRegister(m_port);

  uint8_t oldSREG = SREG;
  cli();
  TCCR1A = 0; //disable pwm by turning off COM1A1,COM1A0,COM1B1,COM1B0
  if (state == PILOT_STATE_P12) {
    *out |= m_bit;  // set pin high
  }
  else {
    *out &= ~m_bit;  // set pin low
  }
  SREG = oldSREG;
#endif // PAFC_PWM

  m_State = state;
}


// set EVSE current capacity in Amperes
// duty cycle 
// outputting a 1KHz square wave to digital pin 10 via Timer 1
//
int J1772Pilot::SetPWM(int amps)
{
#ifdef PAFC_PWM
  // duty cycle = OCR1A(B) / ICR1 * 100 %

  unsigned cnt;
  if ((amps >= 6) && (amps <= 51)) {
  // amps = (duty cycle %) X 0.6
    cnt = amps * (TOP/60);
  } else if ((amps > 51) && (amps <= 80)) {
    // amps = (duty cycle % - 64) X 2.5
    cnt = (amps * (TOP/250)) + (64*(TOP/100));
  }
  else {
    return 1;
  }


#if (PILOT_PIN == 9)
  OCR1A = cnt;
#else // PILOT_PIN == 10
  OCR1B = cnt;
#endif
  
  m_State = PILOT_STATE_PWM;

  return 0;
#else // fast PWM
  uint8_t ocr1b = 0;
  if ((amps >= 6) && (amps <= 51)) {
    ocr1b = 25 * amps / 6 - 1;  // J1772 states "Available current = (duty cycle %) X 0.6"
   } else if ((amps > 51) && (amps <= 80)) {
     ocr1b = amps + 159;  // J1772 states "Available current = (duty cycle % - 64) X 2.5"
   }
  else {
    return 1; // error
  }

  if (ocr1b) {
    // Timer1 initialization:
    // 16MHz / 64 / (OCR1A+1) / 2 on digital 9
    // 16MHz / 64 / (OCR1A+1) on digital 10
    // 1KHz variable duty cycle on digital 10, 500Hz fixed 50% on digital 9
    // pin 10 duty cycle = (OCR1B+1)/(OCR1A+1)
    uint8_t oldSREG = SREG;
    cli();

    TCCR1A = _BV(COM1A0) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
    OCR1A = 249;

    // 10% = 24 , 96% = 239
    OCR1B = ocr1b;

    SREG = oldSREG;

    m_State = PILOT_STATE_PWM;
    return 0;
  }
  else { // !duty
    // invalid amps
    return 1;
  }
#endif // PAFC_PWM
}

//-- end J1772Pilot

//-- begin J1772EVSEController

J1772EVSEController::J1772EVSEController()
{
}

void J1772EVSEController::chargingOn()
{  // turn on charging current
  digitalWrite(CHARGING_PIN,HIGH); 
  digitalWrite(CHARGING_PIN2,HIGH); 
  m_bVFlags |= ECVF_CHARGING_ON;
  
  m_ChargeStartTime = now();
  m_ChargeStartTimeMS = millis();
}

void J1772EVSEController::chargingOff()
{ // turn off charging current
  digitalWrite(CHARGING_PIN,LOW); 
  digitalWrite(CHARGING_PIN2,LOW); 
  m_bVFlags &= ~ECVF_CHARGING_ON;

  m_ChargeOffTime = now();
  m_ChargeOffTimeMS = millis();
} 

#ifdef GFI
inline void J1772EVSEController::SetGfiTripped()
{
#ifdef GFI_SELFTEST
  if (m_Gfi.SelfTestInProgress()) {
    m_Gfi.SetTestSuccess();
    return;
  }
#endif
  m_bVFlags |= ECVF_GFI_TRIPPED;

  // this is repeated Update(), but we want to keep latency as low as possible
  // for safety so we do it here first anyway
  chargingOff(); // turn off charging current
  // turn off the pilot
  m_Pilot.SetState(PILOT_STATE_N12);

  m_Gfi.SetFault();
  // the rest of the logic will be handled in Update()
}
#endif // GFI

void J1772EVSEController::EnableDiodeCheck(uint8_t tf)
{
  if (tf) {
    m_bFlags &= ~ECF_DIODE_CHK_DISABLED;
  }
  else {
    m_bFlags |= ECF_DIODE_CHK_DISABLED;
  }
}

#ifdef GFI_SELFTEST
void J1772EVSEController::EnableGfiTest(uint8_t tf)
{
  if (tf) {
    m_bFlags2 &= ~ECF2_GFI_TEST_DISABLED;
  }
  else {
    m_bFlags2 |= ECF2_GFI_TEST_DISABLED;
  }
}
#endif

void J1772EVSEController::EnableVentReq(uint8_t tf)
{
  if (tf) {
    m_bFlags &= ~ECF_VENT_REQ_DISABLED;
  }
  else {
    m_bFlags |= ECF_VENT_REQ_DISABLED;
  }
}

#ifdef ADVPWR
void J1772EVSEController::EnableGndChk(uint8_t tf)
{
  if (tf) {
    m_bFlags &= ~ECF_GND_CHK_DISABLED;
  }
  else {
    m_bFlags |= ECF_GND_CHK_DISABLED;
  }
}

void J1772EVSEController::EnableStuckRelayChk(uint8_t tf)
{
  if (tf) {
    m_bFlags &= ~ECF_STUCK_RELAY_CHK_DISABLED;
  }
  else {
    m_bFlags |= ECF_STUCK_RELAY_CHK_DISABLED;
  }
}

void J1772EVSEController::EnableAutoSvcLevel(uint8_t tf)
{
  if (tf) {
    m_bFlags &= ~ECF_AUTO_SVC_LEVEL_DISABLED;
  }
  else {
    m_bFlags |= ECF_AUTO_SVC_LEVEL_DISABLED;
  }
}


#endif // ADVPWR

// Functions to support Auto Start feature - GoldServe
#ifdef MANUALSTART 
void J1772EVSEController::EnableAutoStart(uint8_t tf)
{
  if (tf) {
    m_bFlags &= ~ECF_AUTO_START_DISABLED;
  }
  else {
    m_bFlags |= ECF_AUTO_START_DISABLED;
  }
}
#endif //#ifdef MANUALSTART
void J1772EVSEController::EnableSerDbg(uint8_t tf)
{
  if (tf) {
    m_bFlags |= ECF_SERIAL_DBG;
  }
  else {
    m_bFlags &= ~ECF_SERIAL_DBG;
  }
}

void J1772EVSEController::Enable()
{
  m_PrevEvseState = EVSE_STATE_DISABLED;
  m_EvseState = EVSE_STATE_UNKNOWN;
  m_Pilot.SetState(PILOT_STATE_P12);
}

void J1772EVSEController::Disable()
{
  m_EvseState = EVSE_STATE_DISABLED;
  chargingOff();
  m_Pilot.SetState(PILOT_STATE_N12);
  g_OBD.Update();
  // cancel state transition so g_OBD doesn't keep updating
  m_PrevEvseState = EVSE_STATE_DISABLED;
}

void J1772EVSEController::LoadThresholds()
{
    memcpy(&m_ThreshData,&g_DefaultThreshData,sizeof(m_ThreshData));
}

void J1772EVSEController::SetSvcLevel(uint8_t svclvl)
{
#ifdef SERIALCLI
  if (SerDbgEnabled()) {
    g_CLI.printlnn();
    g_CLI.print_P(PSTR("SetSvcLevel: "));Serial.println((int)svclvl);
  }
#endif //#ifdef SERIALCLI
  if (svclvl == 2) {
    m_bFlags |= ECF_L2; // set to Level 2
  }
  else {
    svclvl = 1;
    m_bFlags &= ~ECF_L2; // set to Level 1
  }

  uint8_t ampacity =  EEPROM.read((svclvl == 1) ? EOFS_CURRENT_CAPACITY_L1 : EOFS_CURRENT_CAPACITY_L2);

  if ((ampacity == 0xff) || (ampacity == 0)) {
    ampacity = (svclvl == 1) ? DEFAULT_CURRENT_CAPACITY_L1 : DEFAULT_CURRENT_CAPACITY_L2;
  }
  
  if (ampacity < MIN_CURRENT_CAPACITY) {
    ampacity = MIN_CURRENT_CAPACITY;
  }
  else {
    if (svclvl == 1) { // L1
      if (ampacity > MAX_CURRENT_CAPACITY_L1) {
        ampacity = MAX_CURRENT_CAPACITY_L1;
      }
    }
    else {
      if (ampacity > MAX_CURRENT_CAPACITY_L2) {
        ampacity = MAX_CURRENT_CAPACITY_L2;
      }
    }
  }


  LoadThresholds();

  SetCurrentCapacity(ampacity);
}

#ifdef ADVPWR
uint8_t J1772EVSEController::doPost()
{

  int RelayOff, Relay1, Relay2; //Relay Power status
  int svcState = UD;	// service state = undefined

  m_Pilot.SetState(PILOT_STATE_P12); //check to see if EV is plugged in - write early so it will stabilize before reading.
  g_OBD.SetRedLed(HIGH); 
#ifdef LCD16X2 //Adafruit RGB LCD
  g_OBD.LcdMsg_P(g_psPwrOn,g_psSelfTest);
#endif //Adafruit RGB LCD 

  if (AutoSvcLevelEnabled()) {
    int reading = analogRead(VOLT_PIN); //read pilot
#ifdef SERIALCLI
  if (SerDbgEnabled()) {
    g_CLI.printlnn();
    g_CLI.print_P(PSTR("Pilot: "));Serial.println((int)reading);
	}
#endif //#ifdef SERIALCLI

    m_Pilot.SetState(PILOT_STATE_N12);
    if (reading > 900) {  // IF EV is not connected its Okay to open the relay the do the L1/L2 and ground Check

// save state with both relays off - for stuck relay state
      RelayOff = (digitalRead(ACLINE1_PIN) << 1) +  digitalRead(ACLINE2_PIN);
          
// save state with Relay 1 on 
      digitalWrite(CHARGING_PIN, HIGH);
      delay(RelaySettlingTime);
      Relay1 = (digitalRead(ACLINE1_PIN) << 1) +  digitalRead(ACLINE2_PIN);
      digitalWrite(CHARGING_PIN, LOW);
      delay(RelaySettlingTime); //allow relay to fully open before running other tests
          
// save state for Relay 2 on
       digitalWrite(CHARGING_PIN2, HIGH); 
      delay(RelaySettlingTime);
      Relay2 = (digitalRead(ACLINE1_PIN) << 1) +  digitalRead(ACLINE2_PIN);
           digitalWrite(CHARGING_PIN2, LOW);
      delay(RelaySettlingTime); //allow relay to fully open before running other tests
        
// decide input power state based on the status read  on L1 and L2
// either 2 SPST or 1 DPST relays can be configured 
// valid svcState is L1 - one hot, L2 both hot, OG - open ground both off, SR - stuck relay when shld be off 
//  
	if (RelayOff == none) { // relay not stuck on when off
	switch ( Relay1 ) {
		case ( both ): //
			if ( Relay2 == none ) svcState = L2;
			if ( Relay2 != none ) svcState = SR;
			break;
		case ( none ): //
			if ( Relay2 == none ) svcState = OG;
			if ( Relay2 == both ) svcState = L2;
			if ( Relay2 == L1 || Relay2 == L2 ) svcState = L1;
			break;
		case ( L1on ): // L1 or L2
		case ( L2on ):
			if ( Relay2 != none ) svcState = SR;
			if ( Relay2 == none ) svcState = L1;
			if ( (Relay1 == L1on) && (Relay2 == L2on)) svcState = L2;
			if ( (Relay1 == L2on) && (Relay2 == L1on)) svcState = L2;
			break;
			} // end switch
          }
		else { // Relay stuck on
		svcState = SR; }
          #ifdef SERIALCLI
            if (SerDbgEnabled()) {
              g_CLI.print_P(PSTR("RelayOff: "));Serial.println((int)RelayOff);
              g_CLI.print_P(PSTR("Relay1: "));Serial.println((int)Relay1);
              g_CLI.print_P(PSTR("Relay2: "));Serial.println((int)Relay2);
              g_CLI.print_P(PSTR("SvcState: "));Serial.println((int)svcState);
 }  
          #endif //#ifdef SERIALCLI

// update LCD
#ifdef LCD16X2 //Adafruit RGB LCD
	  if (svcState == L1) g_OBD.LcdMsg_P(g_psAutoDetect,g_psLevel1);
	  if (svcState == L2) g_OBD.LcdMsg_P(g_psAutoDetect,g_psLevel2);
	  if ((svcState == OG) || (svcState == SR))  g_OBD.LcdSetBacklightColor(RED);
	  if (svcState == OG) g_OBD.LcdMsg_P(g_psEarthGround,g_psTestFailed);
	  if (svcState == SR) g_OBD.LcdMsg_P(g_psStuckRelay,g_psTestFailed);
	  delay(500);
#endif //Adafruit RGB LCD
	} // endif test, no EV is plugged in
  } // endif AutoSvcLevelEnabled
  
#ifdef GFI_SELFTEST
  if (GfiSelfTestEnabled()) {
    m_Gfi.SelfTest();
    if (!m_Gfi.SelfTestSuccess()) {
      g_OBD.LcdSetBacklightColor(RED);
      g_OBD.LcdMsg_P(g_psGfiTest,g_psTestFailed);
      delay(500);
      svcState = FG;
    }
  }
#endif

  g_OBD.SetRedLed(LOW); // Red LED off for ADVPWR
  m_Pilot.SetState(PILOT_STATE_P12);
  return int(svcState);
}
#endif // ADVPWR

void J1772EVSEController::Init()
{
  pinMode(CHARGING_PIN,OUTPUT);
  pinMode(CHARGING_PIN2,OUTPUT);

#ifdef ADVPWR
  pinMode(ACLINE1_PIN, INPUT);
  pinMode(ACLINE2_PIN, INPUT);
  digitalWrite(ACLINE1_PIN, HIGH); // enable pullup
  digitalWrite(ACLINE2_PIN, HIGH); // enable pullup
#endif // ADVPWR

  chargingOff();

  m_Pilot.Init(); // init the pilot

  uint8_t svclvl = (uint8_t)DEFAULT_SERVICE_LEVEL;

  uint8_t rflgs = EEPROM.read(EOFS_FLAGS);
  if (rflgs == 0xff) { // uninitialized EEPROM
    m_bFlags = ECF_DEFAULT;
  }
  else {
    m_bFlags = rflgs;
    svclvl = GetCurSvcLevel();
  }
  
  uint8_t rflgs2 = EEPROM.read(EOFS_FLAGS2);
  if (rflgs2 == 0xff) { // uninitialized EEPROM
    m_bFlags2 = ECF2_DEFAULT;
  }
  else {
    m_bFlags2 = rflgs2;
  }

  m_bVFlags = ECVF_DEFAULT;

#ifdef GFI
  m_GfiRetryCnt = 0;
  m_GfiTripCnt = 0;
#endif // GFI
#ifdef ADVPWR
  m_NoGndRetryCnt = 0;
  m_NoGndTripCnt = 0;
  StuckRelayTripCnt = 0;
#endif // ADVPWR

  m_EvseState = EVSE_STATE_UNKNOWN;
  m_PrevEvseState = EVSE_STATE_UNKNOWN;


#ifdef ADVPWR  // Power on Self Test for Advanced Power Supply
 
  uint8_t fault; 
  do {
     fault = 0; // reset post fault
  uint8_t psvclvl = doPost(); // auto detect service level overrides any saved values
    
    if ((AutoSvcLevelEnabled()) && ((psvclvl == L1) || (psvclvl == L2)))  svclvl = psvclvl; //set service level
    if ((GndChkEnabled()) && (psvclvl == OG))  m_EvseState = EVSE_STATE_NO_GROUND, fault = 1; // set No Ground error
    if ((StuckRelayChkEnabled()) && (psvclvl == SR)) m_EvseState = EVSE_STATE_STUCK_RELAY, fault = 1; // set Stuck Relay error
#ifdef GFI_SELFTEST
    if ((GfiSelfTestEnabled()) && (psvclvl == FG)) m_EvseState = EVSE_STATE_GFI_TEST_FAILED, fault = 1; // set GFI test fail error
#endif
    if  ( fault ) delay(GFI_TIMEOUT); // if fault wait GFI_TIMEOUT till next check
  } while ( fault && ( m_EvseState == EVSE_STATE_GFI_TEST_FAILED || m_EvseState == EVSE_STATE_NO_GROUND ||  m_EvseState == EVSE_STATE_STUCK_RELAY ));
#endif // ADVPWR  

  SetSvcLevel(svclvl);

#ifdef GFI
  m_Gfi.Init();
#endif // GFI

  // Start Manual Start Feature - GoldServe
#ifdef MANUALSTART
  if (AutoStartEnabled()){
    Enable();
  } else {
    Disable();
  }
#endif //#ifdef MANUALSTART
  // End Manual Start Feature - GoldServe

  g_OBD.SetGreenLed(LOW);
}

//TABLE A1 - PILOT LINE VOLTAGE RANGES (recommended.. adjust as necessary
//                           Minimum Nominal Maximum 
//Positive Voltage, State A  11.40 12.00 12.60 
//Positive Voltage, State B  8.36 9.00 9.56 
//Positive Voltage, State C  5.48 6.00 6.49 
//Positive Voltage, State D  2.62 3.00 3.25 
//Negative Voltage - States B, C, D, and F -11.40 -12.00 -12.60 
void J1772EVSEController::Update()
{
  if (m_EvseState == EVSE_STATE_DISABLED) {
    // n.b. don't know why, but if we don't have the delay below
    // then doEnableEvse will always have packetBuf[2] == 0
    // so we can't re-enable the EVSE
    delay(5);
    return;
  }

  uint8_t prevevsestate = m_EvseState;
  uint8_t tmpevsestate = EVSE_STATE_UNKNOWN;
  uint8_t nofault = 1;

  int plow;
  int phigh;

#ifdef ADVPWR
  int PS1state = digitalRead(ACLINE1_PIN);
  int PS2state = digitalRead(ACLINE2_PIN);

 if (chargingIsOn()) { // ground check - can only test when relay closed
    if (GndChkEnabled()) {
      if (((millis()-m_ChargeStartTimeMS) > GROUND_CHK_DELAY) && (PS1state == HIGH) && (PS2state == HIGH)) {
	// bad ground
	
	tmpevsestate = EVSE_STATE_NO_GROUND;
	m_EvseState = EVSE_STATE_NO_GROUND;
	
	chargingOff(); // open the relay

	if (m_NoGndTripCnt < 255) {
	  m_NoGndTripCnt++;
	}
	m_NoGndTimeout = millis() + GFI_TIMEOUT;

	nofault = 0;
      }
    }
  }
  else { // !chargingIsOn() - relay open
    if (prevevsestate == EVSE_STATE_NO_GROUND) {
      if ((m_NoGndRetryCnt < GFI_RETRY_COUNT) &&
	  (millis() >= m_NoGndTimeout)) {
	m_NoGndRetryCnt++;
      }
      else {
	tmpevsestate = EVSE_STATE_NO_GROUND;
	m_EvseState = EVSE_STATE_NO_GROUND;
	
	nofault = 0;
      }
    }
    else if (StuckRelayChkEnabled()) {    // stuck relay check - can test only when relay open
      if ((PS1state == LOW) || (PS2state == LOW)) { // Stuck Relay reading
         if ( (prevevsestate != EVSE_STATE_STUCK_RELAY) && (StuckRelayTripCnt == 0) ) { //check for first occurance
            m_StuckRelayStartTimeMS = millis(); // mark start state
            StuckRelayTripCnt++;
         }   
        if ( ( ((millis()-m_ChargeOffTimeMS) > STUCK_RELAY_DELAY) && //  charge off de-bounce
               ((millis()-m_StuckRelayStartTimeMS) > STUCK_RELAY_DELAY) ) ||  // start delay de-bounce
        	(prevevsestate == EVSE_STATE_STUCK_RELAY) ) { // already in error state
	// stuck relay
	tmpevsestate = EVSE_STATE_STUCK_RELAY;
	m_EvseState = EVSE_STATE_STUCK_RELAY;
	nofault = 0;
      }
      } // end of stuck relay reading
      else StuckRelayTripCnt = 0; // not stuck - reset count
     } // end of StuckRelayChkEnabled
  } // end of !chargingIsOn() - relay open
  
#endif // ADVPWR
   
#ifdef GFI
  if (m_Gfi.Fault()) {
    tmpevsestate = EVSE_STATE_GFCI_FAULT;
    m_EvseState = EVSE_STATE_GFCI_FAULT;

    if (m_GfiRetryCnt < GFI_RETRY_COUNT) {
      if (prevevsestate != EVSE_STATE_GFCI_FAULT) {
	if (m_GfiTripCnt < 255) {
	  m_GfiTripCnt++;
	}
 	m_GfiRetryCnt = 0;
	m_GfiTimeout = millis() + GFI_TIMEOUT;
      }
      else if (millis() >= m_GfiTimeout) {
	m_Gfi.Reset();
	m_GfiRetryCnt++;
	m_GfiTimeout = millis() + GFI_TIMEOUT;
      }
    }

    nofault = 0;
  }
#endif // GFI

  if (nofault) {
    if ((prevevsestate == EVSE_STATE_GFCI_FAULT) ||
        (prevevsestate == EVSE_STATE_NO_GROUND) ||
	(prevevsestate == EVSE_STATE_STUCK_RELAY)) {
      // just got out of GFCI fault state - pilot back on
      m_Pilot.SetState(PILOT_STATE_P12);
      prevevsestate = EVSE_STATE_UNKNOWN;
      m_EvseState = EVSE_STATE_UNKNOWN;
    }
    // Begin Sensor readings
    int reading;
    plow = 1023;
    phigh = 0;
    //  digitalWrite(3,HIGH);
    // 1x = 114us 20x = 2.3ms 100x = 11.3ms
    for (int i=0;i < 100;i++) {
      reading = analogRead(VOLT_PIN);  // measures pilot voltage

      if (reading > phigh) {
        phigh = reading;
      }
      else if (reading < plow) {
        plow = reading;
      }
    }
    if (DiodeCheckEnabled() && (m_Pilot.GetState() == PILOT_STATE_PWM) && (plow >= m_ThreshData.m_ThreshDS)) {
      // diode check failed
      tmpevsestate = EVSE_STATE_DIODE_CHK_FAILED;
    }
    else if (phigh >= m_ThreshData.m_ThreshAB) {
      // 12V EV not connected
      tmpevsestate = EVSE_STATE_A;
    }
    else if (phigh >= m_ThreshData.m_ThreshBC) {
      // 9V EV connected, waiting for ready to charge
      tmpevsestate = EVSE_STATE_B;
    }
    else if ((phigh  >= m_ThreshData.m_ThreshCD) ||
	     (!VentReqEnabled() && (phigh > m_ThreshData.m_ThreshD))) {
      // 6V ready to charge
      tmpevsestate = EVSE_STATE_C;
    }
    else if (phigh > m_ThreshData.m_ThreshD) {
      // 3V ready to charge vent required
      tmpevsestate = EVSE_STATE_D;
    }
    else {
      tmpevsestate = EVSE_STATE_UNKNOWN;
    }

    // debounce state transitions
    if (tmpevsestate != prevevsestate) {
      if (tmpevsestate != m_TmpEvseState) {
        m_TmpEvseStateStart = millis();
      }
      else if ((millis()-m_TmpEvseStateStart) >= ((tmpevsestate == EVSE_STATE_A) ? DELAY_STATE_TRANSITION_A : DELAY_STATE_TRANSITION)) {
        m_EvseState = tmpevsestate;
      }
    }
  }

  m_TmpEvseState = tmpevsestate;
  
  // state transition
  if (m_EvseState != prevevsestate) {
    if (m_EvseState == EVSE_STATE_A) { // connected, not ready to charge
      chargingOff(); // turn off charging current
      m_Pilot.SetState(PILOT_STATE_P12);
    }
    else if (m_EvseState == EVSE_STATE_B) { // connected 
      chargingOff(); // turn off charging current
      m_Pilot.SetPWM(m_CurrentCapacity);
    }
    else if (m_EvseState == EVSE_STATE_C) {
      m_Pilot.SetPWM(m_CurrentCapacity);
      chargingOn(); // turn on charging current
    }
    else if (m_EvseState == EVSE_STATE_D) {
      // vent required not supported
      chargingOff(); // turn off charging current
      m_Pilot.SetState(PILOT_STATE_P12);
    }
    else if (m_EvseState == EVSE_STATE_GFCI_FAULT) {
      // vehicle state F
      chargingOff(); // turn off charging current
      m_Pilot.SetState(PILOT_STATE_N12);
    }
    else if (m_EvseState == EVSE_STATE_DIODE_CHK_FAILED) {
      chargingOff(); // turn off charging current
      // must leave pilot on so we can keep checking
      m_Pilot.SetPWM(m_CurrentCapacity);
    }
    else if (m_EvseState == EVSE_STATE_NO_GROUND) {
      // Ground not detected
      chargingOff(); // turn off charging current
      m_Pilot.SetState(PILOT_STATE_N12);
    }
    else if (m_EvseState == EVSE_STATE_STUCK_RELAY) {
      // Stuck relay detected
      chargingOff(); // turn off charging current
      m_Pilot.SetState(PILOT_STATE_N12);
    }
    else {
      m_Pilot.SetState(PILOT_STATE_P12);
      chargingOff(); // turn off charging current
    }
#ifdef SERIALCLI
    if (SerDbgEnabled()) {
      g_CLI.print_P(PSTR("state: "));
      Serial.print((int)prevevsestate);
      g_CLI.print_P(PSTR("->"));
      Serial.print((int)m_EvseState);
      g_CLI.print_P(PSTR(" p "));
      Serial.print(plow);
      g_CLI.print_P(PSTR(" "));
      Serial.println(phigh);
    }
#endif //#ifdef SERIALCLI
  }

  m_PrevEvseState = prevevsestate;

  // End Sensor Readings

  if (m_EvseState == EVSE_STATE_C) {
    m_ElapsedChargeTimePrev = m_ElapsedChargeTime;
    m_ElapsedChargeTime = now() - m_ChargeStartTime;
  }
}

// read ADC values and get min/max/avg for pilot steady high/low states
void J1772EVSEController::Calibrate(PCALIB_DATA pcd)
{
  uint16_t pmax,pmin,pavg,nmax,nmin,navg;

  for (int l=0;l < 2;l++) {
    int reading;
    uint32_t tot = 0;
    uint16_t plow = 1023;
    uint16_t phigh = 0;
    uint16_t avg = 0;
    m_Pilot.SetState(l ? PILOT_STATE_N12 : PILOT_STATE_P12);

    delay(250); // wait for stabilization

    // 1x = 114us 20x = 2.3ms 100x = 11.3ms
    uint8_t pin = PILOT_PIN;
    int i;
    for (i=0;i < 1000;i++) {
      reading = analogRead(VOLT_PIN);  // measures pilot voltage

      if (reading > phigh) {
        phigh = reading;
      }
      else if (reading < plow) {
        plow = reading;
      }

      tot += reading;
    }
    avg = tot / i;

    if (l) {
      nmax = phigh;
      nmin = plow;
      navg = avg;
    }
    else {
      pmax = phigh;
      pmin = plow;
      pavg = avg;
    }
  }
  pcd->m_pMax = pmax;
  pcd->m_pAvg = pavg;
  pcd->m_pMin = pmin;
  pcd->m_nMax = nmax;
  pcd->m_nAvg = navg;
  pcd->m_nMin = nmin;
}


int J1772EVSEController::SetCurrentCapacity(uint8_t amps,uint8_t updatepwm)
{
  int rc = 0;
  if ((amps >= MIN_CURRENT_CAPACITY) && (amps <= ((GetCurSvcLevel() == 1) ? MAX_CURRENT_CAPACITY_L1 : MAX_CURRENT_CAPACITY_L2))) {
    m_CurrentCapacity = amps;
  }
  else {
    m_CurrentCapacity = MIN_CURRENT_CAPACITY;
    rc = 1;
  }

  if (updatepwm && (m_Pilot.GetState() == PILOT_STATE_PWM)) {
    m_Pilot.SetPWM(m_CurrentCapacity);
  }
  return rc;
}


//-- end J1772EVSEController

#ifdef BTN_MENU
Btn::Btn()
{
  buttonState = BTN_STATE_OFF;
  lastDebounceTime = 0;
}

void Btn::init()
{
  pinMode(BTN_PIN,INPUT);
  digitalWrite(BTN_PIN,HIGH); // turn on pullup
}

void Btn::read()
{
  uint8_t sample;
#ifdef ADAFRUIT_BTN
  sample = (g_OBD.readButtons() & BUTTON_SELECT) ? 1 : 0;
#else //!ADAFRUIT_BTN
  sample = digitalRead(BTN_PIN) ? 0 : 1;
#endif // ADAFRUIT_BTN
  if (!sample && (buttonState == BTN_STATE_LONG) && !lastDebounceTime) {
    buttonState = BTN_STATE_OFF;
  }
  if ((buttonState == BTN_STATE_OFF) ||
      ((buttonState == BTN_STATE_SHORT) && lastDebounceTime)) {
    if (sample) {
      if (!lastDebounceTime && (buttonState == BTN_STATE_OFF)) {
	lastDebounceTime = millis();
      }
      long delta = millis() - lastDebounceTime;

      if (buttonState == BTN_STATE_OFF) {
	if (delta >= BTN_PRESS_SHORT) {
	  buttonState = BTN_STATE_SHORT;
	}
      }
      else if (buttonState == BTN_STATE_SHORT) {
	if (delta >= BTN_PRESS_LONG) {
	  buttonState = BTN_STATE_LONG;
	}
      }
    }
    else { //!sample
      lastDebounceTime = 0;
    }
  }
}

uint8_t Btn::shortPress()
{
  if ((buttonState == BTN_STATE_SHORT) && !lastDebounceTime) {
    buttonState = BTN_STATE_OFF;
    return 1;
  }
  else {
    return 0;
  }
}

uint8_t Btn::longPress()
{
  if ((buttonState == BTN_STATE_LONG) && lastDebounceTime) {
    lastDebounceTime = 0;
    return 1;
  }
  else {
    return 0;
  }
}


Menu::Menu()
{
}

void Menu::init(const char *firstitem)
{
  m_CurIdx = 0;
  g_OBD.LcdPrint_P(0,m_Title);
  g_OBD.LcdPrint(1,firstitem);
}

SetupMenu::SetupMenu()
{
  m_Title = g_psSetup;
}

void SetupMenu::Init()
{
  m_CurIdx = 0;
  g_OBD.LcdPrint_P(0,m_Title);
  g_OBD.LcdPrint_P(1,g_SvcLevelMenu.m_Title);
}

void SetupMenu::Next()
{
  if (++m_CurIdx > 10) {
    m_CurIdx = 0;
  }

  const prog_char *title;
  switch(m_CurIdx) {
  case 0:
    title = g_SvcLevelMenu.m_Title;
    break;
  case 1:
    title = g_MaxCurrentMenu.m_Title;
    break;
  case 2:
    title = g_DiodeChkMenu.m_Title;
    break;
  case 3:
    title = g_VentReqMenu.m_Title;
    break;
  case 4:
#ifdef ADVPWR
    title = g_GndChkMenu.m_Title;
    break;
#else
    m_CurIdx++;
    // fall through
#endif // ADVPWR
  case 5:
#ifdef GFI_SELFTEST
    title = g_GfiTestMenu.m_Title;
    break;
#else
    m_CurIdx++;
    // fall through
#endif
// Add menu items to control Delay Timers - GoldServe
  case 6:
#ifdef DELAYTIMER
    title = g_RTCMenu.m_Title;
    break;
  case 7:
    title = g_DelayMenu.m_Title;
    break;
#else
    m_CurIdx += 2;
    // fall through
#endif //#ifdef DELAYTIMER
  case 8:
#ifdef AUTOSTART_MENU
// Add menu items to control Auto Start - GoldServe
    title = g_AutoStartMenu.m_Title;
    break;
#else
    m_CurIdx++;
    // fall through
#endif //#ifdef AUTOSTART_MENU
  case 9:
    title = g_ResetMenu.m_Title;
    break;
  default:
    title = g_psExit;
    break;
  }
  g_OBD.LcdPrint_P(1,title);
}

Menu *SetupMenu::Select()
{
  if (m_CurIdx == 0) {
    return &g_SvcLevelMenu;
  }
  else if (m_CurIdx == 1) {
    return &g_MaxCurrentMenu;
  }
  else if (m_CurIdx == 2) {
    return &g_DiodeChkMenu;
  }
  else if (m_CurIdx == 3) {
    return &g_VentReqMenu;
  }
#ifdef ADVPWR
  else if (m_CurIdx == 4) {
    return &g_GndChkMenu;
  }
#endif // ADVPWR
#ifdef GFI_SELFTEST
  else if (m_CurIdx == 5) {
    return &g_GfiTestMenu;
  }
#endif
  else if (m_CurIdx == 9) {
    return &g_ResetMenu;
  }
#ifdef DELAYTIMER
// Add menu items to control Delay Timers - GoldServe
  else if (m_CurIdx == 6) {
    return &g_RTCMenu;
  }
  else if (m_CurIdx == 7) {
    return &g_DelayMenu;
  }
#endif //#ifdef DELAYTIMER
#ifdef AUTOSTART_MENU
// Add menu items to control Auto Start - GoldServe
  else if (m_CurIdx == 8) {
    return &g_AutoStartMenu;
  }
#endif //#ifdef AUTOSTART_MENU
  else {
    g_OBD.LcdClear();
    return NULL;
  }
}

#ifdef ADVPWR
#define SVC_LVL_MNU_ITEMCNT 3
#else
#define SVC_LVL_MNU_ITEMCNT 2
#endif // ADVPWR
const char *g_SvcLevelMenuItems[] = {
#ifdef ADVPWR
  "Auto",
#endif // ADVPWR
  "Level 1",
  "Level 2"
};


SvcLevelMenu::SvcLevelMenu()
{
  m_Title = g_psSvcLevel;
}

void SvcLevelMenu::Init()
{
  g_OBD.LcdPrint_P(0,m_Title);
#ifdef ADVPWR
  if (g_EvseController.AutoSvcLevelEnabled()) {
    m_CurIdx = 0;
  }
  else {
    m_CurIdx = g_EvseController.GetCurSvcLevel();
  }
#else
  m_CurIdx = (g_EvseController.GetCurSvcLevel() == 1) ? 0 : 1;
#endif // ADVPWR
  //sprintf(g_sTmp,"+%s",g_SvcLevelMenuItems[m_CurIdx]);
  //g_OBD.LcdPrint(1,g_sTmp);
  g_OBD.LcdClearLine(1);
  g_OBD.LcdPrint(0,1,"+");
  g_OBD.LcdPrint(g_SvcLevelMenuItems[m_CurIdx]);
}

void SvcLevelMenu::Next()
{
  if (++m_CurIdx >= SVC_LVL_MNU_ITEMCNT) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0,1);
  if (g_EvseController.GetCurSvcLevel() == (m_CurIdx+1)) {
    g_OBD.LcdPrint("+");
  }
  g_OBD.LcdPrint(g_SvcLevelMenuItems[m_CurIdx]);
}

Menu *SvcLevelMenu::Select()
{
#ifdef ADVPWR
  if (m_CurIdx == 0) {
    g_EvseController.EnableAutoSvcLevel(1);
  }
  else {
    g_EvseController.SetSvcLevel(m_CurIdx);
    g_EvseController.EnableAutoSvcLevel(0);
  }
#else
  g_EvseController.SetSvcLevel(m_CurIdx+1);
#endif // ADVPWR
  g_OBD.LcdPrint(0,1,"+");
  g_OBD.LcdPrint(g_SvcLevelMenuItems[m_CurIdx]);

  EEPROM.write(EOFS_FLAGS,g_EvseController.GetFlags());

  delay(500);

  return &g_SetupMenu;
}

uint8_t g_L1MaxAmps[] = {6,10,12,14,15,16,0};
uint8_t g_L2MaxAmps[] = {10,16,20,25,30,35,40,45,50,55,60,65,70,75,80,0};
MaxCurrentMenu::MaxCurrentMenu()
{
  m_Title = g_psMaxCurrent;
}


void MaxCurrentMenu::Init()
{
  m_CurIdx = 0;
  uint8_t cursvclvl = g_EvseController.GetCurSvcLevel();
  m_MaxAmpsList = (cursvclvl == 1) ? g_L1MaxAmps : g_L2MaxAmps;
  m_MaxCurrent = 0;
  uint8_t currentlimit = (cursvclvl == 1) ? MAX_CURRENT_CAPACITY_L1 : MAX_CURRENT_CAPACITY_L2;

  for (m_MaxIdx=0;m_MaxAmpsList[m_MaxIdx] != 0;m_MaxIdx++);
  m_MaxCurrent = m_MaxAmpsList[--m_MaxIdx];

  for (uint8_t i=0;i <= m_MaxIdx;i++) {
    if (m_MaxAmpsList[i] == g_EvseController.GetCurrentCapacity()) {
      m_CurIdx = i;
      break;
    }
  }
  
  sprintf(g_sTmp,"%s Max Current",(cursvclvl == 1) ? "L1" : "L2");
  g_OBD.LcdPrint(0,g_sTmp);
  sprintf(g_sTmp,"+%dA",m_MaxAmpsList[m_CurIdx]);
  g_OBD.LcdPrint(1,g_sTmp);
}

void MaxCurrentMenu::Next()
{
  if (++m_CurIdx > m_MaxIdx) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0,1);
  if (g_EvseController.GetCurrentCapacity() == m_MaxAmpsList[m_CurIdx]) {
    g_OBD.LcdPrint("+");
  }
  g_OBD.LcdPrint(m_MaxAmpsList[m_CurIdx]);
  g_OBD.LcdPrint("A");
}

Menu *MaxCurrentMenu::Select()
{
  g_OBD.LcdPrint(0,1,"+");
  g_OBD.LcdPrint(m_MaxAmpsList[m_CurIdx]);
  g_OBD.LcdPrint("A");
  delay(500);
  EEPROM.write((g_EvseController.GetCurSvcLevel() == 1) ? EOFS_CURRENT_CAPACITY_L1 : EOFS_CURRENT_CAPACITY_L2,m_MaxAmpsList[m_CurIdx]);  
  g_EvseController.SetCurrentCapacity(m_MaxAmpsList[m_CurIdx]);
  return &g_SetupMenu;
}

char *g_YesNoMenuItems[] = {"Yes","No"};
DiodeChkMenu::DiodeChkMenu()
{
  m_Title = g_psDiodeCheck;
}

void DiodeChkMenu::Init()
{
  g_OBD.LcdPrint_P(0,m_Title);
  m_CurIdx = g_EvseController.DiodeCheckEnabled() ? 0 : 1;
  sprintf(g_sTmp,"+%s",g_YesNoMenuItems[m_CurIdx]);
  g_OBD.LcdPrint(1,g_sTmp);
}

void DiodeChkMenu::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0,1);
  uint8_t dce = g_EvseController.DiodeCheckEnabled();
  if ((dce && !m_CurIdx) || (!dce && m_CurIdx)) {
    g_OBD.LcdPrint("+");
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
}

Menu *DiodeChkMenu::Select()
{
  g_OBD.LcdPrint(0,1,"+");
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);

  g_EvseController.EnableDiodeCheck((m_CurIdx == 0) ? 1 : 0);

  EEPROM.write(EOFS_FLAGS,g_EvseController.GetFlags());

  delay(500);

  return &g_SetupMenu;
}

#ifdef GFI_SELFTEST
GfiTestMenu::GfiTestMenu()
{
  m_Title = g_psGfiTest;
}

void GfiTestMenu::Init()
{
  g_OBD.LcdPrint_P(0,m_Title);
  m_CurIdx = g_EvseController.GfiSelfTestEnabled() ? 0 : 1;
  sprintf(g_sTmp,"+%s",g_YesNoMenuItems[m_CurIdx]);
  g_OBD.LcdPrint(1,g_sTmp);
}

void GfiTestMenu::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0,1);
  uint8_t dce = g_EvseController.GfiSelfTestEnabled();
  if ((dce && !m_CurIdx) || (!dce && m_CurIdx)) {
    g_OBD.LcdPrint("+");
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
}

Menu *GfiTestMenu::Select()
{
  g_OBD.LcdPrint(0,1,"+");
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);

  g_EvseController.EnableGfiTest((m_CurIdx == 0) ? 1 : 0);

  EEPROM.write(EOFS_FLAGS2,g_EvseController.GetFlags2());

  delay(500);

  return &g_SetupMenu;
}
#endif

VentReqMenu::VentReqMenu()
{
  m_Title = g_psVentReqChk;
}

void VentReqMenu::Init()
{
  g_OBD.LcdPrint_P(0,m_Title);
  m_CurIdx = g_EvseController.VentReqEnabled() ? 0 : 1;
  sprintf(g_sTmp,"+%s",g_YesNoMenuItems[m_CurIdx]);
  g_OBD.LcdPrint(1,g_sTmp);
}

void VentReqMenu::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0,1);
  uint8_t dce = g_EvseController.VentReqEnabled();
  if ((dce && !m_CurIdx) || (!dce && m_CurIdx)) {
    g_OBD.LcdPrint("+");
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
}

Menu *VentReqMenu::Select()
{
  g_OBD.LcdPrint(0,1,"+");
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);

  g_EvseController.EnableVentReq((m_CurIdx == 0) ? 1 : 0);

  EEPROM.write(EOFS_FLAGS,g_EvseController.GetFlags());

  delay(500);

  return &g_SetupMenu;
}

#ifdef ADVPWR
GndChkMenu::GndChkMenu()
{
  m_Title = g_psGndChk;
}

void GndChkMenu::Init()
{
  g_OBD.LcdPrint_P(0,m_Title);
  m_CurIdx = g_EvseController.GndChkEnabled() ? 0 : 1;
  sprintf(g_sTmp,"+%s",g_YesNoMenuItems[m_CurIdx]);
  g_OBD.LcdPrint(1,g_sTmp);
}

void GndChkMenu::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0,1);
  uint8_t dce = g_EvseController.GndChkEnabled();
  if ((dce && !m_CurIdx) || (!dce && m_CurIdx)) {
    g_OBD.LcdPrint("+");
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
}

Menu *GndChkMenu::Select()
{
  g_OBD.LcdPrint(0,1,"+");
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);

  g_EvseController.EnableGndChk((m_CurIdx == 0) ? 1 : 0);

  EEPROM.write(EOFS_FLAGS,g_EvseController.GetFlags());

  delay(500);

  return &g_SetupMenu;
}
#endif // ADVPWR

ResetMenu::ResetMenu()
{
  m_Title = g_psReset;
}

prog_char g_psResetNow[] PROGMEM = "Reset Now?";
void ResetMenu::Init()
{
  m_CurIdx = 0;
  g_OBD.LcdPrint_P(0,g_psResetNow);
  g_OBD.LcdPrint(1,g_YesNoMenuItems[0]);
}

void ResetMenu::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdPrint(0,1,g_YesNoMenuItems[m_CurIdx]);
}

#ifdef DELAYTIMER
// Start Menu Definition for Date/Time, Delay, and AutoStart Menus
char g_DateTimeStr[] = "%02d/%02d/%02d %02d:%02d";
char g_DateTimeStr_Month[] = "+%02d/%02d/%02d %02d:%02d";
char g_DateTimeStr_Day[] = "%02d/+%02d/%02d %02d:%02d";
char g_DateTimeStr_Year[] = "%02d/%02d/+%02d %02d:%02d";
char g_DateTimeStr_Hour[] = "%02d/%02d/%02d +%02d:%02d";
char g_DateTimeStr_Min[] = "%02d/%02d/%02d %02d:+%02d";
RTCMenu::RTCMenu()
{
  m_Title = g_psRTC;
}
prog_char g_psSetDateTime[] PROGMEM = "Set Date/Time?";
void RTCMenu::Init()
{
  m_CurIdx = 0;
  g_OBD.LcdPrint_P(0,g_psSetDateTime);
  g_OBD.LcdPrint(1,g_YesNoMenuItems[0]);
}

void RTCMenu::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdPrint(0,1,g_YesNoMenuItems[m_CurIdx]);
}
Menu *RTCMenu::Select()
{
  g_OBD.LcdPrint(0,1,"+");
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
  delay(500);
  if (m_CurIdx == 0) {
    return &g_RTCMenuMonth;
  } else {
    return &g_SetupMenu; 
  }
}
RTCMenuMonth::RTCMenuMonth()
{
  //m_Title = g_psRTC;
}
void RTCMenuMonth::Init()
{
  g_OBD.LcdPrint_P(0,g_psRTC_Month);
  g_CurrTime = g_RTC.now();
  g_month = g_CurrTime.month();
  g_day = g_CurrTime.day();
  g_year = g_CurrTime.year() - 2000;
  g_hour = g_CurrTime.hour();
  g_min = g_CurrTime.minute();
  m_CurIdx = g_month;

  sprintf(g_sTmp,g_DateTimeStr_Month,m_CurIdx, g_day, g_year, g_hour, g_min);
  g_OBD.LcdPrint(1,g_sTmp);
}
void RTCMenuMonth::Next()
{
  if (++m_CurIdx >= 13) {
    m_CurIdx = 1;
  }
  g_OBD.LcdClearLine(1);
  if (m_CurIdx == g_month) {
    sprintf(g_sTmp,g_DateTimeStr_Month,m_CurIdx, g_day, g_year, g_hour, g_min);
  } else {
    sprintf(g_sTmp,g_DateTimeStr,m_CurIdx, g_day, g_year, g_hour, g_min);
  }
  g_OBD.LcdPrint(1,g_sTmp);
}
Menu *RTCMenuMonth::Select()
{
  g_month = m_CurIdx;
  sprintf(g_sTmp,g_DateTimeStr_Month,m_CurIdx, g_day, g_year, g_hour, g_min);
  g_OBD.LcdPrint(1,g_sTmp);
  delay(500);
  return &g_RTCMenuDay;
}
RTCMenuDay::RTCMenuDay()
{
  m_Title = g_psRTC;
}
void RTCMenuDay::Init()
{
  g_OBD.LcdPrint_P(0,g_psRTC_Day);
  m_CurIdx = g_day;
  sprintf(g_sTmp,g_DateTimeStr_Day,g_month, m_CurIdx, g_year, g_hour, g_min);
  g_OBD.LcdPrint(1,g_sTmp);
}
void RTCMenuDay::Next()
{
  if (++m_CurIdx >= 32) {
    m_CurIdx = 1;
  }
  g_OBD.LcdClearLine(1);
  if (m_CurIdx == g_day) {
    sprintf(g_sTmp,g_DateTimeStr_Day,g_month, m_CurIdx, g_year, g_hour, g_min);
  } else {
    sprintf(g_sTmp,g_DateTimeStr,g_month, m_CurIdx, g_year, g_hour, g_min);
  }
  g_OBD.LcdPrint(1,g_sTmp);
}
Menu *RTCMenuDay::Select()
{
  g_day = m_CurIdx;
  //g_OBD.LcdPrint(0,1,"+");
  sprintf(g_sTmp,g_DateTimeStr_Day,g_month, m_CurIdx, g_year, g_hour, g_min);
  g_OBD.LcdPrint(1,g_sTmp);
  delay(500);
  return &g_RTCMenuYear;
}
RTCMenuYear::RTCMenuYear()
{
  //m_Title = g_psRTC;
}
void RTCMenuYear::Init()
{
  g_OBD.LcdPrint_P(0,g_psRTC_Year);
  m_CurIdx = g_year;
  if (m_CurIdx < 12 || m_CurIdx > 20){
    m_CurIdx = 12;
    g_year = 12;
  }
  sprintf(g_sTmp,g_DateTimeStr_Year,g_month, g_day, m_CurIdx, g_hour, g_min);
  g_OBD.LcdPrint(1,g_sTmp);
}
void RTCMenuYear::Next()
{
  if (++m_CurIdx >= 21) {
    m_CurIdx = 12;
  }
  //g_OBD.LcdClearLine(1);
  if (m_CurIdx == g_year) {
    sprintf(g_sTmp,g_DateTimeStr_Year,g_month, g_day, m_CurIdx, g_hour, g_min);
  } else {
    sprintf(g_sTmp,g_DateTimeStr,g_month, g_day, m_CurIdx, g_hour, g_min);
  }
  g_OBD.LcdPrint(1,g_sTmp);
}
Menu *RTCMenuYear::Select()
{
  g_year = m_CurIdx;
  sprintf(g_sTmp,g_DateTimeStr_Year,g_month, g_day, m_CurIdx, g_hour, g_min);
  g_OBD.LcdPrint(1,g_sTmp);
  delay(500);
  return &g_RTCMenuHour;
}
RTCMenuHour::RTCMenuHour()
{
  //m_Title = g_psRTC;
}
void RTCMenuHour::Init()
{
  g_OBD.LcdPrint_P(0,g_psRTC_Hour);
  m_CurIdx = g_hour;
  sprintf(g_sTmp,g_DateTimeStr_Hour,g_month, g_day, g_year, m_CurIdx, g_min);
  g_OBD.LcdPrint(1,g_sTmp);
}
void RTCMenuHour::Next()
{
  if (++m_CurIdx >= 24) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  if (m_CurIdx == g_hour) {
    sprintf(g_sTmp,g_DateTimeStr_Hour,g_month, g_day, g_year, m_CurIdx, g_min);
  } else {
    sprintf(g_sTmp,g_DateTimeStr,g_month, g_day, g_year, m_CurIdx, g_min);
  }
  g_OBD.LcdPrint(1,g_sTmp);
}
Menu *RTCMenuHour::Select()
{
  g_hour = m_CurIdx;
  sprintf(g_sTmp,g_DateTimeStr_Hour,g_month, g_day, g_year, m_CurIdx, g_min);
  g_OBD.LcdPrint(1,g_sTmp);
  delay(500);
  return &g_RTCMenuMinute;
}
RTCMenuMinute::RTCMenuMinute()
{
  //m_Title = g_psRTC;
}
void RTCMenuMinute::Init()
{
  g_OBD.LcdPrint_P(0,g_psRTC_Minute);
  m_CurIdx = g_min;
  sprintf(g_sTmp,g_DateTimeStr_Min,g_month, g_day, g_year, g_hour, m_CurIdx);
  g_OBD.LcdPrint(1,g_sTmp);
}
void RTCMenuMinute::Next()
{
  if (++m_CurIdx >= 60) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  if (m_CurIdx == g_hour) {
    sprintf(g_sTmp,g_DateTimeStr_Min,g_month, g_day, g_year, g_hour, m_CurIdx);
  } else {
    sprintf(g_sTmp,g_DateTimeStr,g_month, g_day, g_year, g_hour, m_CurIdx);
  }
  g_OBD.LcdPrint(1,g_sTmp);
}
Menu *RTCMenuMinute::Select()
{
  g_min = m_CurIdx;
  sprintf(g_sTmp,g_DateTimeStr_Min,g_month, g_day, g_year, g_hour, m_CurIdx);
  g_OBD.LcdPrint(1,g_sTmp);
  g_RTC.adjust(DateTime(g_year, g_month, g_day, g_hour, g_min, 0));
  delay(500);
  return &g_SetupMenu;
}
char *g_DelayMenuItems[] = {"Yes/No","Set Start","Set Stop"};
char *g_sPHHMMfmt = "+%02d:%02d";
char *g_sHHPMMfmt = "%02d:+%02d";
DelayMenu::DelayMenu()
{
  m_Title = g_psDelayTimer;
}
void DelayMenu::Init()
{
  m_CurIdx = 0;
  //g_OBD.LcdMsg("Delay Timer",g_DelayMenuItems[0]);
  g_OBD.LcdPrint_P(0,g_psDelayTimer);
  g_OBD.LcdPrint(1,g_DelayMenuItems[0]);
}
void DelayMenu::Next()
{
  if (++m_CurIdx >= 3) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdPrint(0,1,g_DelayMenuItems[m_CurIdx]);
}
Menu *DelayMenu::Select()
{
  //g_OBD.LcdPrint(0,1,"+");
  //g_OBD.LcdPrint(g_DelayMenuItems[m_CurIdx]);
  delay(500);
  if (m_CurIdx == 0) {
    return &g_DelayMenuEnableDisable;
  } else if (m_CurIdx == 1) {
    return &g_DelayMenuStartHour;
  } else {
    return &g_DelayMenuStopHour;
  }
}
DelayMenuEnableDisable::DelayMenuEnableDisable()
{
  m_Title = g_psDelayTimer;
}
void DelayMenuEnableDisable::Init()
{
  m_CurIdx = !g_DelayTimer.IsTimerEnabled();
  g_OBD.LcdPrint_P(0,g_psDelayTimer);
  g_OBD.LcdClearLine(1);
  g_OBD.LcdPrint(0,1,"+");
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
}
void DelayMenuEnableDisable::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  if (m_CurIdx == !g_DelayTimer.IsTimerEnabled()){
    g_OBD.LcdPrint(0,1,"+");
    g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
  } else {
    g_OBD.LcdPrint(0,1,g_YesNoMenuItems[m_CurIdx]);
  }
}
Menu *DelayMenuEnableDisable::Select()
{
  g_OBD.LcdPrint(0,1,"+");
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
  delay(500);
  if (m_CurIdx == 0) {
    g_DelayTimer.Enable();
  } else {
    g_DelayTimer.Disable();
  }
  return &g_SetupMenu;
}
DelayMenuStartHour::DelayMenuStartHour()
{
  //m_Title = g_psDelayTimerStartHour;
}
void DelayMenuStartHour::Init()
{
  g_OBD.LcdPrint_P(0,g_psDelayTimerStartHour);
  g_hour = g_DelayTimer.GetStartTimerHour();
  g_min = g_DelayTimer.GetStartTimerMin();
  m_CurIdx = g_hour;
  sprintf(g_sTmp,g_sPHHMMfmt, m_CurIdx, g_min);
  g_OBD.LcdPrint(1,g_sTmp);
}

void DelayMenuStartHour::Next()
{
  if (++m_CurIdx >= 24) {
    m_CurIdx = 0;
  }
  //g_OBD.LcdClearLine(1);
  if (m_CurIdx == g_hour) {
    sprintf(g_sTmp,g_sPHHMMfmt, m_CurIdx, g_min);
  } else {
    sprintf(g_sTmp,g_sHHMMfmt, m_CurIdx, g_min);
  }
  g_OBD.LcdPrint(1,g_sTmp);
}
Menu *DelayMenuStartHour::Select()
{
  g_hour = m_CurIdx;
  sprintf(g_sTmp,g_sPHHMMfmt, m_CurIdx, g_min);
  g_OBD.LcdPrint(1,g_sTmp);
  delay(500);
  return &g_DelayMenuStartMin;
}
DelayMenuStopHour::DelayMenuStopHour()
{
  //m_Title = g_psDelayTimerStopHour;
}
void DelayMenuStopHour::Init()
{
  g_OBD.LcdPrint_P(0,g_psDelayTimerStopHour);
  g_hour = g_DelayTimer.GetStopTimerHour();
  g_min = g_DelayTimer.GetStopTimerMin();
  m_CurIdx = g_hour;
  sprintf(g_sTmp,g_sPHHMMfmt, m_CurIdx, g_min);
  g_OBD.LcdPrint(1,g_sTmp);
}
void DelayMenuStopHour::Next()
{
  if (++m_CurIdx >= 24) {
    m_CurIdx = 0;
  }
  //g_OBD.LcdClearLine(1);
  if (m_CurIdx == g_hour) {
    sprintf(g_sTmp,g_sPHHMMfmt, m_CurIdx, g_min);
  } else {
    sprintf(g_sTmp,g_sHHMMfmt, m_CurIdx, g_min);
  }
  g_OBD.LcdPrint(1,g_sTmp);
}
Menu *DelayMenuStopHour::Select()
{
  g_hour = m_CurIdx;
  sprintf(g_sTmp,g_sPHHMMfmt, m_CurIdx, g_min);
  g_OBD.LcdPrint(1,g_sTmp);
  delay(500);
  return &g_DelayMenuStopMin;
}
DelayMenuStartMin::DelayMenuStartMin()
{
  //m_Title = g_psDelayTimerStartMin;
}
void DelayMenuStartMin::Init()
{
  g_OBD.LcdPrint_P(0,g_psDelayTimerStartMin);
  m_CurIdx = g_min;
  sprintf(g_sTmp,g_sHHPMMfmt, g_hour, m_CurIdx);
  g_OBD.LcdPrint(1,g_sTmp);
}

void DelayMenuStartMin::Next()
{
  if (++m_CurIdx >= 60) {
    m_CurIdx = 0;
  }
  //g_OBD.LcdClearLine(1);
  if (m_CurIdx == g_min) {
    sprintf(g_sTmp,g_sHHPMMfmt, g_hour, m_CurIdx);
  } else {
    sprintf(g_sTmp,g_sHHMMfmt, g_hour, m_CurIdx);
  }
  g_OBD.LcdPrint(1,g_sTmp);
}
Menu *DelayMenuStartMin::Select()
{
  g_min = m_CurIdx;
  sprintf(g_sTmp,g_sHHPMMfmt, g_hour, m_CurIdx);
  g_OBD.LcdPrint(1,g_sTmp);
  g_DelayTimer.SetStartTimer(g_hour, g_min);
  delay(500);
  return &g_SetupMenu;
}
DelayMenuStopMin::DelayMenuStopMin()
{
  //m_Title = g_psDelayTimerStopMin;
}
void DelayMenuStopMin::Init()
{
  g_OBD.LcdPrint_P(0,g_psDelayTimerStopMin);
  m_CurIdx = g_min;
  sprintf(g_sTmp,g_sHHPMMfmt, g_hour, m_CurIdx);
  g_OBD.LcdPrint(1,g_sTmp);
}

void DelayMenuStopMin::Next()
{
  if (++m_CurIdx >= 60) {
    m_CurIdx = 0;
  }
  //g_OBD.LcdClearLine(1);
  if (m_CurIdx == g_min) {
    sprintf(g_sTmp,g_sHHPMMfmt, g_hour, m_CurIdx);
  } else {
    sprintf(g_sTmp,g_sHHMMfmt, g_hour, m_CurIdx);
  }
  g_OBD.LcdPrint(1,g_sTmp);
}
Menu *DelayMenuStopMin::Select()
{
  g_min = m_CurIdx;
  sprintf(g_sTmp,g_sHHPMMfmt, g_hour, m_CurIdx);
  g_OBD.LcdPrint(1,g_sTmp);
  g_DelayTimer.SetStopTimer(g_hour, g_min);  // Set Timers
  delay(500);
  return &g_SetupMenu;
}
#endif //#ifdef DELAYTIMER
#ifdef AUTOSTART_MENU
// Menus for Auto Start feature - GoldServe
AutoStartMenu::AutoStartMenu()
{
  m_Title = g_psAutoStart;
}
void AutoStartMenu::Init()
{
  g_OBD.LcdPrint_P(0,g_psAutoStart);
  m_CurIdx = !g_EvseController.AutoStartEnabled();
  sprintf(g_sTmp,"+%s",g_YesNoMenuItems[m_CurIdx]);
  g_OBD.LcdPrint(1,g_sTmp);
}
void AutoStartMenu::Next()
{
  
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0,1);
  uint8_t dce = g_EvseController.AutoStartEnabled();
  if ((dce && !m_CurIdx) || (!dce && m_CurIdx)) {
    g_OBD.LcdPrint("+");
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
  
}
Menu *AutoStartMenu::Select()
{
  g_OBD.LcdPrint(0,1,"+");
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
  g_EvseController.EnableAutoStart((m_CurIdx == 0) ? 1 : 0);
  EEPROM.write(EOFS_FLAGS,g_EvseController.GetFlags());
  delay(500);
  return &g_SetupMenu;
}
#endif //#ifdef AUTOSTART_MENU

// wdt_init turns off the watchdog timer after we use it
// to reboot
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));
void wdt_init(void)
{
    MCUSR = 0;
    wdt_disable();

    return;
}

Menu *ResetMenu::Select()
{
  g_OBD.LcdPrint(0,1,"+");
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
  delay(500);
  if (m_CurIdx == 0) {
    g_OBD.LcdPrint_P(1,PSTR("Resetting..."));
    // hardware reset by forcing watchdog to timeout
    wdt_enable(WDTO_1S);   // enable watchdog timer
    while (1);
  }
  return NULL;
}

BtnHandler::BtnHandler()
{
  m_CurMenu = NULL;
}

void BtnHandler::ChkBtn()
{
  m_Btn.read();
  if (m_Btn.shortPress()) {
    if (m_CurMenu) {
      m_CurMenu->Next();
    }
    else {
#ifdef BTN_ENABLE_TOGGLE
      if (g_EvseController.GetState() == EVSE_STATE_DISABLED) {
	g_EvseController.Enable();
      }
      else {
	g_EvseController.Disable();
      }
#endif // BTN_ENABLE_TOGGLE
    }
  }
  else if (m_Btn.longPress()) {
    if (m_CurMenu) {
      m_CurMenu = m_CurMenu->Select();
      if (m_CurMenu) {
	uint8_t curidx;
	if (m_CurMenu == &g_SetupMenu) {
	  curidx = g_SetupMenu.m_CurIdx;
	}
	m_CurMenu->Init();
	if (m_CurMenu == &g_SetupMenu) {
	  // restore prev menu item
	  g_SetupMenu.m_CurIdx = curidx-1;
	  g_SetupMenu.Next();
	}
      }
      else {
#if defined(DELAYTIMER)
        if (g_DelayTimer.IsTimerEnabled()){
          g_DelayTimer.CheckNow();
        } else {
          g_EvseController.Enable();
        }
#else  
	g_EvseController.Enable();
#endif        
      }
    }
    else {
#ifndef BTN_ENABLE_TOGGLE
      uint8_t curstate = g_EvseController.GetState();
      if ((curstate != EVSE_STATE_B) && (curstate != EVSE_STATE_C)) {
#endif // !BTN_ENABLE_TOGGLE
	g_EvseController.Disable();
	g_SetupMenu.Init();
	m_CurMenu = &g_SetupMenu;
#ifndef BTN_ENABLE_TOGGLE
      }
#endif // !BTN_ENABLE_TOGGLE
    }
  }
}
#endif // BTN_MENU

// Start Delay Timer Functions - GoldServe
#ifdef DELAYTIMER
void DelayTimer::Init(){
  //Read EEPROM settings
  uint8_t rflgs = EEPROM.read(EOFS_TIMER_FLAGS);
  if (rflgs == 0xff) { // uninitialized EEPROM
    m_DelayTimerEnabled = 0x00;
    EEPROM.write(EOFS_TIMER_FLAGS, m_DelayTimerEnabled);
  }
  else {
    m_DelayTimerEnabled = rflgs;
  }
  uint8_t rtmp = EEPROM.read(EOFS_TIMER_START_HOUR);
  if (rtmp == 0xff) { // uninitialized EEPROM
    m_StartTimerHour = DEFAULT_START_HOUR;
    EEPROM.write(EOFS_TIMER_START_HOUR, m_StartTimerHour);
  }
  else {
    m_StartTimerHour = rtmp;
  }
  rtmp = EEPROM.read(EOFS_TIMER_START_MIN);
  if (rtmp == 0xff) { // uninitialized EEPROM
    m_StartTimerMin = DEFAULT_START_MIN;
    EEPROM.write(EOFS_TIMER_START_MIN, m_StartTimerMin);
  }
  else {
    m_StartTimerMin = rtmp;
  }
  rtmp = EEPROM.read(EOFS_TIMER_STOP_HOUR);
  if (rtmp == 0xff) { // uninitialized EEPROM
    m_StopTimerHour = DEFAULT_STOP_HOUR;
    EEPROM.write(EOFS_TIMER_STOP_HOUR, m_StopTimerHour);
  }
  else {
    m_StopTimerHour = rtmp;
  }
  rtmp = EEPROM.read(EOFS_TIMER_STOP_MIN);
  if (rtmp == 0xff) { // uninitialized EEPROM
    m_StopTimerMin = DEFAULT_STOP_MIN;
    EEPROM.write(EOFS_TIMER_STOP_MIN, m_StopTimerMin);
  }
  else {
    m_StopTimerMin = rtmp;
  }
  
  FlexiTimer2::set(60000, DelayTimerInterrupt); // Set 1 minute interrupt for Delay Timer handling
  FlexiTimer2::start(); // Start interrupt
};
void DelayTimer::CheckTime(){
  if (m_CheckNow == 1){
    g_CurrTime = g_RTC.now();
    m_CurrHour = g_CurrTime.hour();
    m_CurrMin = g_CurrTime.minute();
    
    if (IsTimerEnabled() && IsTimerValid()){
      uint16_t m_StartTimerSeconds = m_StartTimerHour * 360 + m_StartTimerMin * 6;
      uint16_t m_StopTimerSeconds = m_StopTimerHour * 360 + m_StopTimerMin * 6;
      uint16_t m_CurrTimeSeconds = m_CurrHour * 360 + m_CurrMin * 6;
      
      //Serial.println(m_StartTimerSeconds);
      //Serial.println(m_StopTimerSeconds);
      //Serial.println(m_CurrTimeSeconds);
      
      if (m_StopTimerSeconds < m_StartTimerSeconds) {
      //End time is for next day 
        if ( ( (m_CurrTimeSeconds >= m_StartTimerSeconds) && (m_CurrTimeSeconds >= m_StopTimerSeconds) ) || ( (m_CurrTimeSeconds <= m_StartTimerSeconds) && (m_CurrTimeSeconds <= m_StopTimerSeconds) ) ){
           // Within time interval
#ifdef BTN_MENU
          if (g_EvseController.GetState() == EVSE_STATE_DISABLED && !g_BtnHandler.InMenu()){
#else
          if (g_EvseController.GetState() == EVSE_STATE_DISABLED){
#endif //#ifdef BTN_MENU
  	    g_EvseController.Enable();
          }           
        } else {
          if (m_CurrTimeSeconds == m_StopTimerSeconds) {
            // Not in time interval
            g_EvseController.Disable();         
          }
        }
      } else {
        if ((m_CurrTimeSeconds >= m_StartTimerSeconds) && (m_CurrTimeSeconds < m_StopTimerSeconds)) {
          // Within time interval
#ifdef BTN_MENU
          if (g_EvseController.GetState() == EVSE_STATE_DISABLED && !g_BtnHandler.InMenu()){
#else
          if (g_EvseController.GetState() == EVSE_STATE_DISABLED){
#endif //#ifdef BTN_MENU
  	    g_EvseController.Enable();
          }          
        } else {
          if (m_CurrTimeSeconds == m_StopTimerSeconds) {
            // Not in time interval
            g_EvseController.Disable();          
          }
        }
      }
    }
    m_CheckNow = 0;
  }
};
void DelayTimer::Enable(){;
  m_DelayTimerEnabled = 0x01;
  EEPROM.write(EOFS_TIMER_FLAGS, m_DelayTimerEnabled);
  g_EvseController.EnableAutoStart(0);
  SaveSettings();
}
void DelayTimer::Disable(){
  m_DelayTimerEnabled = 0x00;
  EEPROM.write(EOFS_TIMER_FLAGS, m_DelayTimerEnabled);
  g_EvseController.EnableAutoStart(1);
  SaveSettings();
};
void DelayTimer::PrintTimerIcon(){
  //g_OBD.LcdClear();
  //g_OBD.LcdSetCursor(0,0);
  if (IsTimerEnabled() && IsTimerValid()){
#ifdef BTN_MENU
    g_OBD.LcdWrite(0x0);
#endif //#ifdef BTN_MENU
  }
};
void DelayTimerInterrupt(){
  g_DelayTimer.CheckNow();
};
// End Delay Timer Functions - GoldServe
#endif //#ifdef DELAYTIMER

void EvseReset()
{
#ifdef DELAYTIMER
  // Initialize Delay Timer - GoldServe
  Wire.begin();
  g_RTC.begin();
  g_DelayTimer.Init();
#endif //#ifdef DELAYTIMER

#ifdef SERIALCLI
  g_CLI.Init();
#endif // SERIALCLI

  g_OBD.Init();

  g_EvseController.Init();
  
}

void setup()
{
  wdt_disable();
  
  Serial.begin(SERIAL_BAUD);

#ifdef GFI
  // GFI triggers on rising edge
  attachInterrupt(GFI_INTERRUPT,gfi_isr,RISING);
#endif // GFI

  EvseReset();

#ifdef BTN_MENU
  g_BtnHandler.init();
#endif // BTN_MENU

#ifdef WATCHDOG
  // WARNING: ALL DELAYS *MUST* BE SHORTER THAN THIS TIMER OR WE WILL GET INTO
  // AN INFINITE RESET LOOP
  wdt_enable(WDTO_1S);   // enable watchdog timer
#endif // WATCHDOG
} 


void loop()
{ 
#ifdef WATCHDOG
  wdt_reset(); // pat the dog
#endif // WATCHDOG

#ifdef SERIALCLI
  g_CLI.getInput();
#endif // SERIALCLI

  g_EvseController.Update();

  g_OBD.Update();
  
#ifdef BTN_MENU
  g_BtnHandler.ChkBtn();

  //  debugging code
   /*   g_Btn.read();
  if (g_Btn.shortPress()) {
    g_OBD.LcdSetCursor(0,0);
    g_OBD.LcdPrint("short");
    delay(1000);
    g_OBD.LcdSetCursor(0,0);
    g_OBD.LcdPrint("     ");
  }
  else if (g_Btn.longPress()) {
    g_OBD.LcdSetCursor(0,0);
    g_OBD.LcdPrint("long");
    delay(1000);
    g_OBD.LcdSetCursor(0,0);
    g_OBD.LcdPrint("    ");
  }
  */
#endif // BTN_MENU
  // Delay Timer Handler - GoldServe
#ifdef DELAYTIMER
  g_DelayTimer.CheckTime();
#endif //#ifdef DELAYTIMER

}
