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

#define VERSION "2.1.A5"

//-- begin features

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


//Adafruit RGBLCD
#define RGBLCD

// Adafruit LCD backpack in I2C mode
//#define I2CLCD

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
#define RTC // enable RTC & timer functions

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

//-- end features



#if defined(RGBLCD) || defined(I2CLCD)
#define LCD16X2
//If LCD is not defined, undef BTN_MENU - requires LCD
#else
#undef BTN_MENU
#endif // RGBLCD || I2CLCD

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
//#define CURRENT_PIN 0 // analog current reading pin A0
#define VOLT_PIN 1 // analog voltage reading pin A1
#define ACLINE1_PIN 3 // TEST PIN 1 for L1/L2, ground and stuck relay
#define ACLINE2_PIN 4 // TEST PIN 2 for L1/L2, ground and stuck relay
#define RED_LED_PIN 5 // Digital pin
#define CHARGING_PIN2 7 // digital Relay trigger pin for second relay
#define CHARGING_PIN 8 // digital Charging LED and Relay Trigger pin
#define PILOT_PIN 10 // n.b. PILOT_PIN *MUST* be digital 10 because SetPWM() assumes it
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

// for J1772.ReadPilot()
// 1x = 114us 20x = 2.3ms 100x = 11.3ms
#define PILOT_LOOP_CNT 100

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
class OnboardDisplay 

{
#if defined(RGBLCD) || defined(I2CLCD)
LiquidTWI2 m_Lcd;
#endif
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
  void LcdSetBacklightType(uint8_t rgb) { // 1=rgb,0=mono
    if (rgb) m_bFlags &= ~OBDF_MONO_BACKLIGHT;
    else m_bFlags |= OBDF_MONO_BACKLIGHT;
    Update(1);
  }
  uint8_t IsLcdBacklightMono() { return (m_bFlags & OBDF_MONO_BACKLIGHT) ? 1 : 0; }
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

  void Update(int8_t force=0);
};

#ifdef GFI
class Gfi {
 uint8_t m_GfiFault;
public:
  Gfi() {}
  void Init();
  void Reset();
  void SetFault() { m_GfiFault = 1; }
  uint8_t Fault() { return m_GfiFault; }
  
};
#endif // GFI


typedef enum {
  PILOT_STATE_P12,PILOT_STATE_PWM,PILOT_STATE_N12} 
PILOT_STATE;
class J1772Pilot {
  uint8_t m_bit;
  uint8_t m_port;
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
#define ECF_MONO_LCD           000100 // monochrome LCD backlight
#define ECF_DEFAULT            0x0000

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
  enum { UD, L1, L2, OG, SR} 
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
  void SetSvcLevel(uint8_t svclvl);
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
  int SetBacklightType(uint8_t t); // 0=mono,1=RGB
#endif // RGBLCD

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
  void ChkBtn(int8_t notoggle=0);
  uint8_t InMenu() { return (m_CurMenu == NULL) ? 0 : 1; }
};

#endif // BTN_MENU

#include "rapi_proc.h"
