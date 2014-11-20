// -*- C++ -*-
/*
 * Open EVSE Firmware
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



#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <pins_arduino.h>
#include <Wire.h>
#include <FlexiTimer2.h> // Required for RTC and Delay Timer
#include <Time.h>
#if defined(ARDUINO) && (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h" // shouldn't need this but arduino sometimes messes up and puts inside an #ifdef
#endif // ARDUINO

#define VERSION "D3.3.0"

//-- begin features

// current measurement
#define AMMETER

// serial remote api
#define RAPI

// serial port command line
// For the RTC version, only CLI or LCD can be defined at one time. 
// There is a directive to take care of that if you forget.
//#define SERIALCLI

// enable watchdog timer
//#define WATCHDOG

// GFI support
#define GFI
// If you loop a wire from the third GFI pin through the CT a few times and then to ground,
// enable this. ADVPWR must also be defined.
#define GFI_SELFTEST


//Adafruit RGBLCD (MCP23017) - can have RGB or monochrome backlight
#define RGBLCD

//select default LCD backlight mode. can be overridden w/CLI/RAPI
#define BKL_TYPE_MONO 0
#define BKL_TYPE_RGB  1
#define DEFAULT_LCD_BKL_TYPE BKL_TYPE_RGB
//#define DEFAULT_LCD_BKL_TYPE BLK_TYPE_MONO

// Adafruit LCD backpack in I2C mode (MCP23008)
//#define I2CLCD
// Support PCF8574* based I2C backpack using F. Malpartida's library
// https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
// note: When enabling I2CLCD_PCF8754, due to stupidity of Arduino, at 
//   top of open_evse.pde must
//   uncomment #include <LiquidCrystal_I2C.h> and
//   comment out #include <LiquidTWI2.h>
//#define I2CLCD_PCF8574

// Advanced Powersupply... Ground check, stuck relay, L1/L2 detection.
#define ADVPWR

// single button menus (needs LCD enabled)
// connect an SPST-NO button between BTN_PIN and GND or enable ADAFRUIT_BTN to use the 
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
#define RTC // enable RTC & timer functions

#ifdef RTC
// Option for Delay Timer - GoldServe
#define DELAYTIMER
// Option for AutoStart Enable/Disable - GoldServe
#define MANUALSTART
// Option for AutoStart Menu. If defined, ManualStart feature is also defined by default - GoldServe
//#define AUTOSTART_MENU

#if defined(DELAYTIMER) && defined(BTN_MENU) && !defined(RAPI)
#define DELAYTIMER_MENU
#endif

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

#ifndef RGBLCD
#define DEFAULT_LCD_BKL_TYPE BKL_TYPE_MONO
#endif

#if defined(RGBLCD) || defined(I2CLCD)
#define LCD16X2
//If LCD is not defined, undef BTN_MENU - requires LCD
#else
#undef BTN_MENU
#endif // RGBLCD || I2CLCD

#ifndef I2CLCD
#undef I2CLCD_PCF8574
#endif

//If LCD and RTC is defined, un-define CLI so we can save ram space.
#if defined(RTC) && defined(LCD16X2)
#if defined(SERIALCLI)
INVALID CONFIG - CANNOT enable SERIALCLI with RTC together - too much RAM USE
#endif
#endif

#if defined(RAPI) && defined(SERIALCLI)
INVALID CONFIG - CANNOT DEFINE SERIALCLI AND RAPI TOGETHER SINCE THEY BOTH USE THE SERIAL PORT
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
#define CURRENT_PIN 0 // analog current reading pin A0
#define VOLT_PIN 1 // analog voltage reading pin A1
#define ACLINE1_PIN 3 // TEST PIN 1 for L1/L2, ground and stuck relay
#define ACLINE2_PIN 4 // TEST PIN 2 for L1/L2, ground and stuck relay
#define RED_LED_PIN 5 // Digital pin
#define CHARGING_PIN2 7 // digital Relay trigger pin for second relay
#define CHARGING_PIN 8 // digital Charging LED and Relay Trigger 
#define CHARGING_PINAC 9 //digital Charging pin for AC relay

// N.B. if PAFC_PWM is enabled, then PILOT_PIN can be either 9 or 10
// (i.e PORTB pins 1 & 2)
// if using fast PWM (PAFC_PWM disabled) PILOT_PIN *MUST* be digital 10
#define PILOT_PIN 10

#define GREEN_LED_PIN 13 // Digital pin

#define SERIAL_BAUD 115200

// EEPROM offsets for settings
#define EOFS_CURRENT_CAPACITY_L1 0 // 1 byte
#define EOFS_CURRENT_CAPACITY_L2 1 // 1 byte
#define EOFS_FLAGS               2 // 2 bytes

// EEPROM offsets for Delay Timer function - GoldServe
#define EOFS_TIMER_FLAGS         4 // 1 byte
#define EOFS_TIMER_START_HOUR    5 // 1 byte
#define EOFS_TIMER_START_MIN     6 // 1 byte
#define EOFS_TIMER_STOP_HOUR     7 // 1 byte
#define EOFS_TIMER_STOP_MIN      8 // 1 byte

#ifdef AMMETER
#define EOFS_CURRENT_SCALE_FACTOR 9 // 2 bytes
#define EOFS_AMMETER_CURR_OFFSET  11 // 2 bytes
#endif // AMMETER

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
#ifdef I2CLCD_PCF8574
#include <LiquidCrystal_I2C.h>
#define LCD_I2C_ADDR 0x27
#else
#include <LiquidTWI2.h>
#define LCD_I2C_ADDR 0x20 // for adafruit shield or backpack
#endif // I2CLCD_PCF8574
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

// for J1772.ReadPilot()
// 1x = 114us 20x = 2.3ms 100x = 11.3ms
#define PILOT_LOOP_CNT 100

#ifdef AMMETER
// This multiplier is the number of milliamps per A/d converter unit.

// First, you need to select the burden resistor for the CT. You choose the largest value possible such that // the maximum peak-to-peak voltage for the current range is 5 volts. To obtain this value, divide the maximum // outlet current by the Te. That value is the maximum CT current RMS. You must convert that to P-P, so multiply // by 2*sqrt(2). Divide 5 by that value and select the next lower standard resistor value. For the reference // design, Te is 1018 and the outlet maximum is 30. 5/((30/1018)*2*sqrt(2)) = 59.995, so a 56 ohm resistor // is called for. Call this value Rb (burden resistor).

// Next, one must use Te and Rb to determine the volts-per-amp value. Note that the readCurrent() // method calculates the RMS value before the scaling factor, so RMS need not be taken into account.
// (1 / Te) * Rb = Rb / Te = Volts per Amp. For the reference design, that's 55.009 mV.

// Each count of the A/d converter is 4.882 mV (5/1024). V/A divided by V/unit is unit/A. For the reference // design, that's 11.26. But we want milliamps per unit, so divide that into 1000 to get 88.7625558. Round near...
//#define DEFAULT_CURRENT_SCALE_FACTOR 106 // for RB = 47 - recommended for 30A max
//#define DEFAULT_CURRENT_SCALE_FACTOR 184 // for RB = 27 - recommended for 50A max 
// Craig K, I arrived at 213 by scaling my previous multiplier of 225 down by the ratio of my panel meter reading of 28 with the OpenEVSE uncalibrated reading of 29.6
// then upped the scale factor to 220 after fixing the zero offset by subtracing 900ma
//#define DEFAULT_CURRENT_SCALE_FACTOR 220 // for RB = 22 - measured by Craig on his new OpenEVSE V3 
// NOTE: setting DEFAULT_CURRENT_SCALE_FACTOR TO 0 will disable the ammeter
#define DEFAULT_CURRENT_SCALE_FACTOR 0

// subtract this from ammeter current reading to correct zero offset
#define DEFAULT_AMMETER_CURRENT_OFFSET 0
//#define DEFAULT_AMMETER_CURRENT_OFFSET -950 // mA //Craig K,  this resolves the zero offset that I observed

// The maximum number of milliseconds to sample an ammeter pin in order to find three zero-crossings.
// one and a half cycles at 50 Hz is 30 ms.
#define CURRENT_SAMPLE_INTERVAL 35

// Once we detect a zero-crossing, we should not look for one for another quarter cycle or so. 1/4 // cycle at 50 Hz is 5 ms.
#define CURRENT_ZERO_DEBOUNCE_INTERVAL 5

// how often to read the ammeter (ms)
//not used yet#define AMMETER_READ_INTERVAL 0

#endif // AMMETER


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
extern char *g_BlankLine;
#endif // LCD16X2

// OnboardDisplay.m_bFlags
#define OBDF_MONO_BACKLIGHT 0x01
#define OBDF_AMMETER_DIRTY  0x80
class OnboardDisplay 

{
#if defined(RGBLCD) || defined(I2CLCD)
#ifdef I2CLCD_PCF8574
#include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C m_Lcd;  
#else
  LiquidTWI2 m_Lcd;
#endif // I2CLCD_PCF8574
#endif // defined(RGBLCD) || defined(I2CLCD)
  uint8_t m_bFlags;
  char m_strBuf[LCD_MAX_CHARS_PER_LINE+1];


public:
  OnboardDisplay();
  void Init();

  void SetGreenLed(uint8_t state);
  void SetRedLed(uint8_t state);
#ifdef LCD16X2
  void LcdBegin(int x,int y) { 
#ifdef I2CLCD
#ifndef I2CLCD_PCF8574
    m_Lcd.setMCPType(LTI_TYPE_MCP23008);
#endif
    m_Lcd.begin(x,y); 
    m_Lcd.setBacklight(HIGH);
#elif defined(RGBLCD)
    m_Lcd.setMCPType(LTI_TYPE_MCP23017);
    m_Lcd.begin(x,y,2); 
    m_Lcd.setBacklight(WHITE);
#endif // I2CLCD
  }
#ifdef I2CLCD_PCF8574
  uint8_t LcdDetected() { return 1; }
#else
  uint8_t LcdDetected() { return m_Lcd.LcdDetected(); }
#endif 
  void LcdPrint(const char *s) { 
    if (LcdDetected()) m_Lcd.print(s); 
  }
  void LcdPrint_P(const prog_char *s);
  void LcdPrint(int y,const char *s);
  void LcdPrint_P(int y,const prog_char *s);
  void LcdPrint(int x,int y,const char *s) { 
    m_Lcd.setCursor(x,y);
    if (LcdDetected()) m_Lcd.print(s); 
  }
  void LcdPrint_P(int x,int y,const prog_char *s);
  void LcdPrint(int i) { 
    if (LcdDetected()) m_Lcd.print(i); 
  }
  void LcdSetCursor(int x,int y) { 
    m_Lcd.setCursor(x,y); 
  }
  void LcdClearLine(int y) {
    m_Lcd.setCursor(0,y);
    if (LcdDetected()) m_Lcd.print(g_BlankLine);
  }
  void LcdClear() { 
    m_Lcd.clear();
  }
  void LcdWrite(uint8_t data) { 
    m_Lcd.write(data);
  }
  void LcdMsg(const char *l1,const char *l2);
  void LcdMsg_P(const prog_char *l1,const prog_char *l2);
  void LcdSetBacklightType(uint8_t t,uint8_t update=1) { // BKL_TYPE_XXX
#ifdef RGBLCD
    if (t == BKL_TYPE_RGB) m_bFlags &= ~OBDF_MONO_BACKLIGHT;
    else m_bFlags |= OBDF_MONO_BACKLIGHT;
    Update(update);
#endif // RGBLCD
  }
  uint8_t IsLcdBacklightMono() {
#ifdef RGBLCD
    return (m_bFlags & OBDF_MONO_BACKLIGHT) ? 1 : 0;
#else
    return 1;
#endif // RGBLCD
  }
  void LcdSetBacklightColor(uint8_t c) {
#ifdef RGBLCD
    if (IsLcdBacklightMono()) {
      if (c) c = WHITE;
    }
    m_Lcd.setBacklight(c);
#endif // RGBLCD
  }
#ifdef RGBLCD
  uint8_t readButtons() { return m_Lcd.readButtons(); }
#endif // RGBLCD
#endif // LCD16X2

#ifdef AMMETER
  void SetAmmeterDirty(uint8_t tf) {
    if (tf) m_bFlags |= OBDF_AMMETER_DIRTY;
    else m_bFlags &= ~OBDF_AMMETER_DIRTY;
  }
  int8_t AmmeterIsDirty() { return (m_bFlags & OBDF_AMMETER_DIRTY) ? 1 : 0; }
#endif // AMMETER

  void Update(int8_t force=0);
};

#ifdef GFI
class Gfi {
 uint8_t m_GfiFault;
#ifdef GFI_SELFTEST
 uint8_t testSuccess;
 uint8_t testInProgress;
#endif // GFI_SELFTEST
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
#define EVSE_STATE_SLEEPING 0xfe // waiting for timer
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

// J1772EVSEController volatile m_bVFlags bits - not saved to EEPROM
#define ECVF_AMMETER_CAL        0x10 // ammeter calibration mode
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
  uint16_t m_wFlags; // ECF_xxx
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
// Define Service States UD = undefined state, L1 = level 1, L2 = level 2, OG = open ground, SR = stuck Relay
  enum { UD, L1, L2, OG, SR, FG} 
  SVCSTATE;

  uint8_t doPost();
#endif // ADVPWR
  void chargingOn();
  void chargingOff();
  uint8_t chargingIsOn() { return m_bVFlags & ECVF_CHARGING_ON; }
  void setFlags(uint16_t flags) { 
    m_wFlags |= flags; 
  }
  void clrFlags(uint16_t flags) { 
    m_wFlags &= ~flags; 
  }

#ifdef AMMETER
  //  long m_LastAmmeterReadMs;
  unsigned long m_AmmeterReading;
  int32_t m_ChargingCurrent;
  int16_t m_AmmeterCurrentOffset;
  int16_t m_CurrentScaleFactor;

  void readAmmeter();
#endif // AMMETER

public:
  J1772EVSEController();
  void Init();
  void Update(); // read sensors
  void Enable();
  void Disable(); // panic stop - open relays abruptly
  void Sleep(); // graceful stop - e.g. waiting for timer to fire- give the EV time to stop charging first
  void LoadThresholds();

  uint16_t GetFlags() { return m_wFlags; }
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
  int SetCurrentCapacity(uint8_t amps,uint8_t updatelcd=0);
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
    return (m_wFlags & ECF_L2) ? 2 : 1; 
  }
  void SetSvcLevel(uint8_t svclvl,uint8_t updatelcd=0);
  PTHRESH_DATA GetThreshData() { 
    return &m_ThreshData; 
  }
  uint8_t DiodeCheckEnabled() { 
    return (m_wFlags & ECF_DIODE_CHK_DISABLED) ? 0 : 1;
  }
  void EnableDiodeCheck(uint8_t tf);
  uint8_t VentReqEnabled() { 
    return (m_wFlags & ECF_VENT_REQ_DISABLED) ? 0 : 1;
  }
  void EnableVentReq(uint8_t tf);
#ifdef ADVPWR
  uint8_t GndChkEnabled() { 
    return (m_wFlags & ECF_GND_CHK_DISABLED) ? 0 : 1;
  }
  void EnableGndChk(uint8_t tf);
  void EnableStuckRelayChk(uint8_t tf);
  uint8_t StuckRelayChkEnabled() { 
    return (m_wFlags & ECF_STUCK_RELAY_CHK_DISABLED) ? 0 : 1;
  }
  uint8_t AutoSvcLevelEnabled() { return (m_wFlags & ECF_AUTO_SVC_LEVEL_DISABLED) ? 0 : 1; }
  void EnableAutoSvcLevel(uint8_t tf);
  void SetNoGndTripped();
  uint8_t NoGndTripped() { return m_bVFlags & ECVF_NOGND_TRIPPED; }
#endif // ADVPWR
#ifdef GFI
  void SetGfiTripped();
  uint8_t GfiTripped() { return m_bVFlags & ECVF_GFI_TRIPPED; }
#ifdef GFI_SELFTEST
  uint8_t GfiSelfTestEnabled() {
    return (m_wFlags & ECF_GFI_TEST_DISABLED) ? 0 : 1;
  }
  void EnableGfiSelfTest(uint8_t tf);
#endif
#endif // GFI
  uint8_t SerDbgEnabled() { 
    return (m_wFlags & ECF_SERIAL_DBG) ? 1 : 0;
  }
  // Function to suppport Auto Start feature - GoldServe
#ifdef MANUALSTART
  void EnableAutoStart(uint8_t tf);
  uint8_t AutoStartEnabled() { 
    return (m_wFlags & ECF_AUTO_START_DISABLED) ? 0 : 1;
  }
#endif //ifdef MANUALSTART
  void EnableSerDbg(uint8_t tf);
#ifdef RGBLCD
  int SetBacklightType(uint8_t t,uint8_t update=1); // BKL_TYPE_XXX
#endif // RGBLCD

#ifdef AMMETER
  int32_t GetChargingCurrent() { return m_ChargingCurrent; }
  int16_t GetAmmeterCurrentOffset() { return m_AmmeterCurrentOffset; }
  int16_t GetCurrentScaleFactor() { return m_CurrentScaleFactor; }
  void SetAmmeterCurrentOffset(int16_t offset) {
    m_AmmeterCurrentOffset = offset;
    EEPROM.write(EOFS_AMMETER_CURR_OFFSET,(offset & 0x0000ff00) >> 8);
    EEPROM.write(EOFS_AMMETER_CURR_OFFSET+1,offset & 0x000000ff);
  }
  void SetCurrentScaleFactor(int16_t scale) {
    m_CurrentScaleFactor = scale;
    EEPROM.write(EOFS_CURRENT_SCALE_FACTOR,(scale & 0x0000ff00) >> 8);
    EEPROM.write(EOFS_CURRENT_SCALE_FACTOR+1,scale & 0x000000ff);
  }
  uint8_t AmmeterCalEnabled() { 
    return (m_bVFlags & ECVF_AMMETER_CAL) ? 1 : 0;
  }
  void EnableAmmeterCal(uint8_t tf) {
    if (tf) {
      m_bVFlags |= ECVF_AMMETER_CAL;
    }
    else {
      m_bVFlags &= ~ECVF_AMMETER_CAL;
    }
  }

#endif
  void ReadPilot(int *plow,int *phigh,int loopcnt=PILOT_LOOP_CNT);
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
  uint8_t m_menuCnt;
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

#ifdef RGBLCD
class BklTypeMenu : public Menu {
public:
  BklTypeMenu();
  void Init();
  void Next();
  Menu *Select();
};
#endif // RGBLCD

#ifdef AUTOSTART_MENU
class AutoStartMenu : public Menu {
public:
  AutoStartMenu();
  void Init();
  void Next();
  Menu *Select();
};
#endif //#ifdef AUTOSTART_MENU

#if defined(DELAYTIMER)
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
#endif // DELAYTIMER

class BtnHandler {
  Btn m_Btn;
  Menu *m_CurMenu;
  uint8_t m_CurLcdMode;
public:
  BtnHandler();
  void init() { m_Btn.init(); }
  void ChkBtn(int8_t notoggle=0);
  uint8_t InMenu() { return (m_CurMenu == NULL) ? 0 : 1; }
  uint8_t GetSavedLcdMode() { return m_CurLcdMode; }
  void SetSavedLcdMode(uint8_t mode ) { m_CurLcdMode = mode; }
};

#endif // BTN_MENU

#ifdef DELAYTIMER
// Start Delay Timer class definition - GoldServe
void SaveSettings();
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

#endif //ifdef DELAYTIMER


// -- end class definitions

#include "rapi_proc.h"
