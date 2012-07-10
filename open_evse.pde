// -*- C++ -*-
/*
 * Open EVSE Firmware
 *
 * Copyright (c) 2011-2012 Sam C. Lin <lincomatic@gmail.com> and Chris Howell <chris1howell@msn.com>
 * Maintainers: SCL/CH

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
#include <pins_arduino.h>
#include <Wire.h>
#include <Time.h>
#if defined(ARDUINO) && (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h" // shouldn't need this but arduino sometimes messes up and puts inside an #ifdef
#endif // ARDUINO

#define VERSTR "0.6.0RC2"

//-- begin features

// enable watchdog timer
//#define WATCHDOG

// GFI support
#define GFI

// serial port command line
#define SERIALCLI

//Adafruit RGBLCD
#define RGBLCD

 // Adafruit LCD backpack in I2C mode
//#define I2CLCD

 // Advanced Powersupply... Ground check, stuck relay, L1/L2 detection.
#define ADVPWR

// single button menus (needs LCD enabled)
// connect an SPST button between BTN_PIN and GND via a 2K resistor
// How to use 1-button menu
// When not in menus, short press instantly stops the EVSE - another short press resumes.  Long press activates menus
// When within menus, short press cycles menu items, long press selects and exits current submenu
// N.B. CURRENTLY, BTN_MENU AND ADVPWR DON'T WORK TOGETHER.  YOU CAN ENABLE ONLY ONE OF THEM AT AT TIME.  I'M HAVING A LOT OF TROUBLE FIGURING OUT THE PROBLEM - SAM
//#define BTN_MENU

#ifdef BTN_MENU
#define BTN_MENU_KLUDGE
#endif // BTN_MENU

// for stability testing - shorter timeout/higher retry count
//#define GFI_TESTING

//-- end features

#if defined(RGBLCD) || defined(I2CLCD)
#define LCD16X2
#endif // RGBLCD || I2CLCD

//-- begin configuration

// n.b. DEFAULT_SERVICE_LEVEL is ignored if ADVPWR defined, since it's autodetected
#define DEFAULT_SERVICE_LEVEL 1 // 1=L1, 2=L2

// current capacity in amps
#define DEFAULT_CURRENT_CAPACITY_L1 12
#define DEFAULT_CURRENT_CAPACITY_L2 16

// minimum allowable current in amps
#define MIN_CURRENT_CAPACITY 6

// maximum allowable current in amps
#define MAX_CURRENT_CAPACITY_L1 15 // NEMA 5-15
#define MAX_CURRENT_CAPACITY_L2 80

//J1772EVSEController
//#define CURRENT_PIN 0 // analog current reading pin A0
#define VOLT_PIN 1 // analog voltage reading pin A1
#define ACLINE1_PIN 3 // TEST PIN 1 for L1/L2, ground and stuck relay
#define ACLINE2_PIN 4 // TEST PIN 2 for L1/L2, ground and stuck relay
#define RED_LED_PIN 5 // Digital pin
#define CHARGING_PIN 8 // digital Charging LED and Relay Trigger pin
#define PILOT_PIN 10 // n.b. PILOT_PIN *MUST* be digial 10 because initWave() assumes it
#define GREEN_LED_PIN 13 // Digital pin

#define SERIAL_BAUD 38400

// EEPROM offsets for settings
#define EOFS_CURRENT_CAPACITY_L1 0 // 1 byte
#define EOFS_CURRENT_CAPACITY_L2 1 // 1 byte
#define EOFS_FLAGS               2 // 1 byte

// must stay within thresh for this time in ms before switching states
#define DELAY_STATE_TRANSITION 250
// must transition to state A from contacts closed in < 100ms according to spec
// but Leaf sometimes bounces from 3->1 so we will debounce it a little anyway
#define DELAY_STATE_TRANSITION_A 25

// for ADVPWR
#define GROUND_CHK_DELAY  1000 // delay after charging started to test, ms
#define STUCK_RELAY_DELAY 1000 // delay after charging opened to test, ms

#ifdef GFI
#define GFI_INTERRUPT 0 // interrupt number 0 = D2, 1 = D3

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
#define BLUE 0x6

#ifdef RGBLCD //Adafruit RGB LCD
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>

#endif //Adafruit RGB LCD

#ifdef I2CLCD
// N.B. must use Adafruit's LiquidCrystal library, not Arduino's.
// https://github.com/adafruit/LiquidCrystal
#include <LiquidCrystal.h>
#define LCD_I2C_ADDR 0 // for adafruit LCD backpack
#endif // I2CLCD


#define BTN_PIN A3 // button sensing pin
#define BTN_PRESS_SHORT 100  // ms
#define BTN_PRESS_LONG 500 // ms

//-- end configuration

//-- begin class definitions

#ifdef SERIALCLI
#define CLI_BUFLEN 13
class CLI {
  int m_CLIinByte; // CLI byte being read in
  char m_CLIinstr[CLI_BUFLEN]; // CLI byte being read in
  int m_CLIstrCount; //CLI string counter

public:
  CLI();
  void Init();
  void println(char *s) { 
    Serial.println(s); 
  }
  void print(char *s) { 
    Serial.print(s); 
  }
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
#ifdef RGBLCD //Adafruit RGB LCD
Adafruit_RGBLCDShield m_Lcd;
#endif //Adafruit RGB LCD
#ifdef I2CLCD
LiquidCrystal m_Lcd; 
#endif // I2CLCD


public:
  OnboardDisplay()
#ifdef I2CLCD
  : m_Lcd(LCD_I2C_ADDR)
#endif // I2CLCD
  {} 
  void Init();
  void SetGreenLed(uint8_t state);
  void SetRedLed(uint8_t state);
#ifdef LCD16X2
  void LcdBegin(int x,int y) { 
    m_Lcd.begin(x,y); 
  }
  void LcdPrint(const char *s) { 
    m_Lcd.print(s); 
  }
  void LcdPrint(int y,const char *s);
  void LcdPrint(int x,int y,const char *s) { 
    m_Lcd.setCursor(x,y);
    m_Lcd.print(s); 
  }
  void LcdPrint(int i) { 
    m_Lcd.print(i); 
  }
  void LcdSetCursor(int x,int y) { 
    m_Lcd.setCursor(x,y); 
  }
  void LcdClearLine(int y) {
    m_Lcd.setCursor(0,y);
    m_Lcd.print(g_BlankLine);
  }
  void LcdClear() { 
    m_Lcd.clear();
  }

  void LcdMsg(const char *l1,const char *l2);
  void LcdSetBacklightColor(uint8_t c) {
#ifdef RGBLCD
    m_Lcd.setBacklight(c);
#endif // RGBLCD
  }
#endif // LCD16X2

  void Update();
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
#define ECF_SERIAL_DBG         0x80 // enable debugging messages via serial
#define ECF_DEFAULT            0x00

// J1772EVSEController volatile m_bVFlags bits - not saved to EEPROM
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
  uint8_t m_bFlags; // ECF_xxx
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

public:
  J1772EVSEController();
  void Init();
  void Update(); // read sensors
  void Enable();
  void Disable();
  void LoadThresholds();

  uint8_t GetFlags() { return m_bFlags; }
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
#ifdef GFI
  void SetGfiTripped();
  uint8_t GfiTripped() { return m_bVFlags & ECVF_GFI_TRIPPED; }
#endif // GFI
  uint8_t SerDbgEnabled() { 
    return (m_bFlags & ECF_SERIAL_DBG) ? 1 : 0;
  }
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

  void read();
  uint8_t shortPress();
  uint8_t longPress();
};


typedef enum {MS_NONE,MS_SETUP,MS_SVC_LEVEL,MS_MAX_CURRENT,MS_RESET} MENU_STATE;
class Menu {
public:
  char *m_Title;
  uint8_t m_CurIdx;
  
  void init(const char *firstitem);

  Menu();

  virtual void Init() = 0;
  virtual void ShortPress() = 0;
  virtual Menu *LongPress() = 0;
};

class SetupMenu : public Menu {
public:
  SetupMenu();
  void Init();
  void ShortPress();
  Menu *LongPress();
};

class SvcLevelMenu : public Menu {
public:
  SvcLevelMenu();
  void Init();
  void ShortPress();
  Menu *LongPress();
};

class MaxCurrentMenu  : public Menu {
  uint8_t m_MaxCurrent;
  uint8_t m_MaxIdx;
  uint8_t *m_MaxAmpsList;
public:
  MaxCurrentMenu();
  void Init();
  void ShortPress();
  Menu *LongPress();
};

class DiodeChkMenu : public Menu {
public:
  DiodeChkMenu();
  void Init();
  void ShortPress();
  Menu *LongPress();
};


class VentReqMenu : public Menu {
public:
  VentReqMenu();
  void Init();
  void ShortPress();
  Menu *LongPress();
};


class ResetMenu : public Menu {
public:
  ResetMenu();
  void Init();
  void ShortPress();
  Menu *LongPress();
};


class BtnHandler {
  Btn m_Btn;
  Menu *m_CurMenu;
public:
  BtnHandler();
  void ChkBtn();
};

char g_sSetup[] = "Setup";
char g_sSvcLevel[] = "Service Level";
char g_sMaxCurrent[] = "Max Current";
char g_sDiodeCheck[] = "Diode Check";
char g_sVentReqChk[] = "Vent Req'd Check";
char g_sReset[] = "Reset";
char g_sExit[] = "Exit";

SetupMenu g_SetupMenu;
SvcLevelMenu g_SvcLevelMenu;
MaxCurrentMenu g_MaxCurrentMenu;
DiodeChkMenu g_DiodeChkMenu;
VentReqMenu g_VentReqMenu;
ResetMenu g_ResetMenu;

BtnHandler g_BtnHandler;
#endif // BTN_MENU

// -- end class definitions
//-- begin global variables

char g_sTmp[25];

THRESH_DATA g_DefaultThreshData = {875,780,690,0,260};
J1772EVSEController g_EvseController;
OnboardDisplay g_OBD;

#ifdef SERIALCLI
CLI g_CLI;
#endif // SERIALCLI

#ifdef LCD16X2
char g_sEvseError[] =  "EVSE ERROR";
#ifdef ADVPWR
char g_sPwrOn[] = "Power On";
char g_sSelfTest[] = "Self Test";
char g_sLevel1[] = "       L1";
char g_sLevel2[] = "       L2";
char g_sStuckRelay[] = "--Stuck Relay--";
char g_sEarthGround[] = "--Earth Ground--";
char g_sTestPassed[] = "Test Passed";
char g_sTestFailed[] = "TEST FAILED";
char g_sEvseSvc[] = "--ServiceLevel--";
#endif // ADVPWR
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
}

void CLI::Init()
{
  Serial.println("Open EVSE"); // CLI print prompt when serial is ready
  Serial.println("Hardware - Atmel ATMEGA328P-AU"); //CLI Info
  Serial.print("Software - Open EVSE "); //CLI info
  Serial.println(VERSTR);
  Serial.println("");

  g_CLI.println("type ? or help for command list");
  g_CLI.print("Open_EVSE>"); // CLI Prompt
  g_CLI.flush();

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

void CLI::getInput()
{
  int currentreading;
  uint8_t amp;
  if(Serial.available()) { // if byte(s) are available to be read
    m_CLIinByte = Serial.read(); // read the byte
    Serial.print(char(m_CLIinByte));
    if(m_CLIinByte != 13) {
      m_CLIinstr[m_CLIstrCount] = char(m_CLIinByte);
      m_CLIstrCount++;
    }

    if(m_CLIinByte == 13) { // if enter was pressed or max chars reached
      Serial.println(""); // print a newline
      if (strcmp(m_CLIinstr, "show") == 0){ //if match SHOW 

        Serial.println("Open EVSE"); // CLI print prompt when serial is ready
        Serial.println("Hardware - Atmel ATMEGA328P-AU"); //CLI Info
        Serial.print("Software - Open EVSE ");
	Serial.println(VERSTR);
        Serial.println("EVSE Settings");
        Serial.print("EVSE current capacity (Amps) = ");
        Serial.println((int)g_EvseController.GetCurrentCapacity()); 
        Serial.print("Min EVSE Current Capacity = ");
        Serial.println(MIN_CURRENT_CAPACITY);
        Serial.print("Max EVSE Current Capacity = ");
        Serial.println(MAX_CURRENT_CAPACITY_L2);
        char s[80];
        int i;
        sscanf(s,"%d",&i);
           
      } 
      else if ((strcmp(m_CLIinstr, "help") == 0) || (strcmp(m_CLIinstr, "?") == 0)){ // string compare
        Serial.println("Help Commands");
        Serial.println("");
        Serial.println("help  --   Display commands"); // print to the terminal
        Serial.println("set   --   Change Settings");
        Serial.println("show  --   Display settings and values");
        Serial.println("save  --   Write settings to EEPROM");
      } 
      else if (strcmp(m_CLIinstr, "set") == 0){ // string compare
        Serial.println("Set Commands - Usage: set amp");
        Serial.println("");
        Serial.println("amp  --  Set the EVSE Current Capacity"); // print to the terminal
	Serial.println("sdbg on/off - turn serial debugging on/off");
       } 
      else if (strcmp(m_CLIinstr, "set sdbg on") == 0){
	g_EvseController.EnableSerDbg(1);
	Serial.println("Serial Debugging Enabled");
      }
      else if (strcmp(m_CLIinstr, "set sdbg off") == 0){
	g_EvseController.EnableSerDbg(0);
	Serial.println("Serial Debugging Disabled");
      }
      else if (strcmp(m_CLIinstr, "set amp") == 0){ // string compare
        Serial.println("WARNING - DO NOT SET CURRENT HIGHER THAN 80%");
	Serial.println("OF YOUR CIRCUIT BREAKER OR"); 
        Serial.println("GREATER THAN THE RATED VALUE OF THE EVSE");
        Serial.println("");
        Serial.print("Enter amps (");
        Serial.print(MIN_CURRENT_CAPACITY);
        Serial.print("-");
        Serial.print((g_EvseController.GetCurSvcLevel()  == 1) ? MAX_CURRENT_CAPACITY_L1 : MAX_CURRENT_CAPACITY_L2);
	Serial.print("): ");
	amp = getInt();
	Serial.println((int)amp);
        if(g_EvseController.SetCurrentCapacity(amp,1)) {
	  Serial.println("Invalid Current Capacity");
	}

        Serial.print("\nEVSE Current Capacity now: "); // print to the terminal
        Serial.print((int)g_EvseController.GetCurrentCapacity());
        Serial.print(" Amps");
      } 
      else if (strcmp(m_CLIinstr, "save") == 0){ // string compare
        Serial.println("Saving Settings to EEPROM"); // print to the terminal
        SaveSettings();
      } 
      else { // if the input text doesn't match any defined above
        Serial.println("Unknown Command -- type ? or help for command list"); // echo back to the terminal
      } 
      Serial.println("");
      Serial.println("");
      Serial.print("Open_EVSE>");
      g_CLI.flush();
      m_CLIstrCount = 0; // get ready for new input... reset strCount
      m_CLIinByte = 0; // reset the inByte variable
      for(int i = 0; m_CLIinstr[i] != '\0'; i++) { // while the string does not have null
        m_CLIinstr[i] = '\0'; // fill it with null to erase it
      }
    }
  }
}

#endif // SERIALCLI



void OnboardDisplay::Init()
{
  pinMode (GREEN_LED_PIN, OUTPUT);
  pinMode (RED_LED_PIN, OUTPUT);

  SetGreenLed(LOW);
  SetRedLed(LOW);
  
#ifdef LCD16X2 //Adafruit RGB LCD  
  LcdBegin(16, 2);
 
  LcdPrint(0,0,"Open EVSE       ");
  delay(500);
  LcdPrint(0,1,"Version ");
  LcdPrint(VERSTR);
  LcdPrint("   ");
  delay(1500);
#endif // LCD16X2
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
// print at (0,y), filling out the line with trailing spaces
void OnboardDisplay::LcdPrint(int y,const char *s)
{
  m_Lcd.setCursor(0,y);
  char ss[25];
  sprintf(ss,"%-16s",s);
  ss[16] = '\0';
  m_Lcd.print(ss);
}

void OnboardDisplay::LcdMsg(const char *l1,const char *l2)
{
  LcdPrint(0,l1);
  LcdPrint(1,l2);
}
#endif // LCD16X2


void OnboardDisplay::Update()
{
  uint8_t curstate = g_EvseController.GetState();
  uint8_t svclvl = g_EvseController.GetCurSvcLevel();
  int i;

  if (g_EvseController.StateTransition()) {
    switch(curstate) {
    case EVSE_STATE_A: // not connected
      SetGreenLed(HIGH);
      SetRedLed(LOW);
      #ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(GREEN);
      sprintf(g_sTmp,"Ready     L%d:%dA",(int)svclvl,(int)g_EvseController.GetCurrentCapacity());
      LcdMsg(g_sTmp,"EV Not Connected");
      #endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_B: // connected/not charging
      SetGreenLed(HIGH);
      SetRedLed(HIGH);
      #ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(YELLOW);
      sprintf(g_sTmp,"Ready     L%d:%dA",(int)svclvl,(int)g_EvseController.GetCurrentCapacity());
      LcdMsg(g_sTmp,"EV Connected");
      #endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_C: // charging
      SetGreenLed(LOW);
      SetRedLed(LOW);
      #ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(BLUE);
      sprintf(g_sTmp,"Charging  L%d:%dA",(int)svclvl,(int)g_EvseController.GetCurrentCapacity());
      LcdPrint(0,g_sTmp);
      #endif //Adafruit RGB LCD
      // n.b. blue LED is on
      break;
    case EVSE_STATE_D: // vent required
      SetGreenLed(LOW);
      SetRedLed(HIGH);
      #ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      LcdMsg(g_sEvseError,"VENT REQUIRED");
      #endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_DIODE_CHK_FAILED:
      SetGreenLed(LOW);
      SetRedLed(HIGH);
      #ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      LcdMsg(g_sEvseError,"DIODE CHK FAILED");
      #endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_GFCI_FAULT:
      SetGreenLed(LOW);
      SetRedLed(HIGH);
      #ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      LcdMsg(g_sEvseError,"GFCI FAULT");
      #endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
     case EVSE_STATE_NO_GROUND:
      SetGreenLed(LOW);
      SetRedLed(HIGH);
      #ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      LcdMsg(g_sEvseError,"NO GROUND");
      #endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
     case EVSE_STATE_STUCK_RELAY:
      SetGreenLed(LOW);
      SetRedLed(HIGH);
      #ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(RED);
      LcdMsg(g_sEvseError,"STUCK RELAY");
      #endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_DISABLED:
      SetGreenLed(LOW);
      SetRedLed(HIGH);
#ifdef LCD16X2
      LcdPrint(0,"Stopped");
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
      sprintf(g_sTmp,"%02d:%02d:%02d",h,m,s);
      LcdPrint(1,g_sTmp);
    }
  }
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
  m_GfiFault = 0;

  Reset();
}

//RESET GFI LOGIC
void Gfi::Reset()
{
#ifdef WATCHDOG
  wdt_reset(); // pat the dog
#endif // WATCHDOG

  m_GfiFault = 0;
}

#endif // GFI
//-- begin J1772Pilot

void J1772Pilot::Init()
{
  pinMode(PILOT_PIN,OUTPUT);
  m_bit = digitalPinToBitMask(PILOT_PIN);
  m_port = digitalPinToPort(PILOT_PIN);

  SetState(PILOT_STATE_P12); // turns the pilot on 12V steady state
}


// no PWM pilot signal - steady state
// PILOT_STATE_P12 = steady +12V (EVSE_STATE_A - VEHICLE NOT CONNECTED)
// PILOT_STATE_N12 = steady -12V (EVSE_STATE_F - FAULT) 
void J1772Pilot::SetState(PILOT_STATE state)
{
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

  m_State = state;
}


// set EVSE current capacity in Amperes
// duty cycle 
// outputting a 1KHz square wave to digital pin 11 via Timer 1
//
int J1772Pilot::SetPWM(int amps)
{
  float duty = 0.0;
  float famps = (float) amps;
  if ((amps >= 6) && (amps < 51)) {
    // duty cycle in %
    duty = famps / 0.6;
  }
  else if ((amps > 51) && (amps < 80)) {
    duty = (famps / 2.5) + 64;
  }
  else if (amps == 80) {
    duty = 96;
  }

  if (duty) {
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
    OCR1B = (int)((2.5 * duty) - 1.0);

    SREG = oldSREG;

    m_State = PILOT_STATE_PWM;
    return 0;
  }
  else { // !duty
    // invalid amps
    return 1;
  }
}

//-- end J1772Pilot

//-- begin J1772EVSEController

J1772EVSEController::J1772EVSEController()
{
}

void J1772EVSEController::chargingOn()
{  // turn on charging current
  digitalWrite(CHARGING_PIN,HIGH); 
  m_bVFlags |= ECVF_CHARGING_ON;
  
  m_ChargeStartTime = now();
  m_ChargeStartTimeMS = millis();
}

void J1772EVSEController::chargingOff()
{ // turn off charging current
  digitalWrite(CHARGING_PIN,LOW); 
  m_bVFlags &= ~ECVF_CHARGING_ON;

  m_ChargeOffTime = now();
  m_ChargeOffTimeMS = millis();
} 

#ifdef GFI
inline void J1772EVSEController::SetGfiTripped()
{
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

void J1772EVSEController::EnableVentReq(uint8_t tf)
{
  if (tf) {
    m_bFlags &= ~ECF_VENT_REQ_DISABLED;
  }
  else {
    m_bFlags |= ECF_VENT_REQ_DISABLED;
  }
}

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
  if (SerDbgEnabled()) {
    Serial.print("SetSvcLevel: ");Serial.println((int)svclvl);
  }

  if (svclvl == 2) {
    m_bFlags |= ECF_L2; // set to Level 2
  }
  else {
    svclvl = 1;
    m_bFlags &= ~ECF_L2; // set to Level 1
  }

  int ampacity =  EEPROM.read((svclvl == 1) ? EOFS_CURRENT_CAPACITY_L1 : EOFS_CURRENT_CAPACITY_L2);

  if ((ampacity == 255) || (ampacity == 0)) {
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
#ifdef BTN_MENU_KLUDGE
uint8_t J1772EVSEController::doPost()
{return 1;
  int PS1state,PS2state;
  uint8_t svclvl = 0;

  m_Pilot.SetState(PILOT_STATE_P12); //check to see if EV is plugged in - write early so it will stabilize before reading.
  g_OBD.SetRedLed(HIGH); 

  PS1state = digitalRead(ACLINE1_PIN); //STUCK RELAY test read AC voltage with Relay Open 
  PS2state = digitalRead(ACLINE2_PIN); //STUCK RELAY test read AC voltage with Relay Open

  if ((PS1state == LOW) || (PS2state == LOW)) {   // If AC voltage is present (LOW) than the relay is stuck
    m_Pilot.SetState(PILOT_STATE_N12);
#ifdef LCD16X2 //Adafruit RGB LCD
    g_OBD.LcdSetBacklightColor(RED);
    //    g_OBD.LcdMsg(g_sStuckRelay,g_sTestFailed);
#endif  //Adafruit RGB LCD
  } 
  else {
    int reading = analogRead(VOLT_PIN); //read pilot
    m_Pilot.SetState(PILOT_STATE_N12);
    if (reading > 0) {              // IF no EV is plugged in its Okay to open the relay the do the L1/L2 and ground Check
      digitalWrite(CHARGING_PIN, HIGH);
      delay(500);
      PS1state = digitalRead(ACLINE1_PIN);
      PS2state = digitalRead(ACLINE2_PIN);
      digitalWrite(CHARGING_PIN, LOW);
      if ((PS1state == HIGH) && (PS2state == HIGH)) {     
	// m_EvseState = EVSE_STATE_NO_GROUND;
#ifdef LCD16X2 //Adafruit RGB LCD
	g_OBD.LcdSetBacklightColor(RED); 
	//	g_OBD.LcdMsg(g_sEarthGround,g_sTestFailed);
#endif  //Adafruit RGB LCD
      } 
      else {
	if ((PS1state == LOW) && (PS2state == LOW)) {  //L2   
	  svclvl = 2; // L2
	}  
	else { // L1
	  svclvl = 1; // L1
	}
      }  
    } 
  }
  
  g_OBD.SetRedLed(LOW); // Red LED off for ADVPWR

  if (svclvl == 0) {
    while (1); // error, wait forever
  }

  return svclvl;
}
#else // !BTN_MENU_KLUDGE
uint8_t J1772EVSEController::doPost()
{
  int PS1state,PS2state;
  uint8_t svclvl = 0;

  m_Pilot.SetState(PILOT_STATE_P12); //check to see if EV is plugged in - write early so it will stabilize before reading.
  g_OBD.SetRedLed(HIGH); 
#ifdef LCD16X2 //Adafruit RGB LCD
  g_OBD.LcdMsg(g_sPwrOn,g_sSelfTest);
  delay(1000);
#endif //Adafruit RGB LCD 

  PS1state = digitalRead(ACLINE1_PIN); //STUCK RELAY test read AC voltage with Relay Open 
  PS2state = digitalRead(ACLINE2_PIN); //STUCK RELAY test read AC voltage with Relay Open

  if ((PS1state == LOW) || (PS2state == LOW)) {   // If AC voltage is present (LOW) than the relay is stuck
    m_Pilot.SetState(PILOT_STATE_N12);
#ifdef LCD16X2 //Adafruit RGB LCD
    g_OBD.LcdSetBacklightColor(RED);
    g_OBD.LcdMsg(g_sStuckRelay,g_sTestFailed);
#endif  //Adafruit RGB LCD
  } 
  else {
#ifdef LCD16X2 //Adafruit RGB LCD
    g_OBD.LcdMsg(g_sStuckRelay,g_sTestPassed);
    delay(1000);
#endif //Adafruit RGB LCD
 

    int reading = analogRead(VOLT_PIN); //read pilot
    m_Pilot.SetState(PILOT_STATE_N12);
    if (reading > 0) {              // IF no EV is plugged in its Okay to open the relay the do the L1/L2 and ground Check
      digitalWrite(CHARGING_PIN, HIGH);
      delay(500);
      PS1state = digitalRead(ACLINE1_PIN);
      PS2state = digitalRead(ACLINE2_PIN);
      digitalWrite(CHARGING_PIN, LOW);
      if ((PS1state == HIGH) && (PS2state == HIGH)) {     
	// m_EvseState = EVSE_STATE_NO_GROUND;
#ifdef LCD16X2 //Adafruit RGB LCD
	g_OBD.LcdSetBacklightColor(RED); 
	g_OBD.LcdMsg(g_sEarthGround,g_sTestFailed);
#endif  //Adafruit RGB LCD
      } 
      else {
#ifdef LCD16X2 //Adafruit RGB LCD
	g_OBD.LcdMsg(g_sEarthGround,g_sTestPassed);
	delay(1000);
#endif  //Adafruit RGB LCD

	if ((PS1state == LOW) && (PS2state == LOW)) {  //L2   
#ifdef LCD16X2 //Adafruit RGB LCD
	  g_OBD.LcdMsg(g_sEvseSvc,g_sLevel2);
	  delay(1000);
#endif //Adafruit RGB LCD

	  svclvl = 2; // L2
	}  
	else { // L1
#ifdef LCD16X2 //Adafruit RGB LCD
	 g_OBD.LcdMsg(g_sEvseSvc,g_sLevel1);
	 delay(1000);
#endif //Adafruit RGB LCD
	  svclvl = 1; // L1
	}
      }  
    } 
  }
  
  g_OBD.SetRedLed(LOW); // Red LED off for ADVPWR

  if (svclvl == 0) {
    while (1); // error, wait forever
  }
  else {
    m_Pilot.SetState(PILOT_STATE_P12);
  }

  return svclvl;
}
#endif // BTN_MENU_KLUDGE
#endif // ADVPWR

void J1772EVSEController::Init()
{
  pinMode(CHARGING_PIN,OUTPUT);

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

  m_bVFlags = ECVF_DEFAULT;

#ifdef GFI
  m_GfiRetryCnt = 0;
  m_GfiTripCnt = 0;
#endif // GFI

  m_EvseState = EVSE_STATE_UNKNOWN;
  m_PrevEvseState = EVSE_STATE_UNKNOWN;


#ifdef ADVPWR  // Power on Self Test for Advanced Power Supply
  svclvl = doPost(); // auto detect service level overrides any saved values
#endif // ADVPWR  

  SetSvcLevel(svclvl);

#ifdef GFI
  m_Gfi.Init();
#endif // GFI


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
    
    if (((millis()-m_ChargeStartTimeMS) > GROUND_CHK_DELAY) && (PS1state == HIGH) && (PS2state == HIGH)) {
      // bad ground
      
      tmpevsestate = EVSE_STATE_NO_GROUND;
      m_EvseState = EVSE_STATE_NO_GROUND;
      
      chargingOff(); // open the relay
      nofault = 0;
    }
  }
  else { // stuck relay check - can test only when relay open
    if (((millis()-m_ChargeOffTimeMS) > STUCK_RELAY_DELAY) &&
	((PS1state == LOW) || (PS2state == LOW))) {
      // stuck relay
      
      tmpevsestate = EVSE_STATE_STUCK_RELAY;
      m_EvseState = EVSE_STATE_STUCK_RELAY;
      
      nofault = 0;
    }
  }
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
	(prevevsestate == EVSE_STATE_NO_GROUND)) {
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

    if (SerDbgEnabled()) {
      Serial.print("state: ");
      Serial.print((int)prevevsestate);
      Serial.print("->");
      Serial.print((int)m_EvseState);
      Serial.print(" p ");
      Serial.print(plow);
      Serial.print(" ");
      Serial.println(phigh);
    }
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
  }

  if (updatepwm && (m_Pilot.GetState() == PILOT_STATE_PWM)) {
    m_Pilot.SetPWM(amps);
  }
  return rc;
}


//-- end J1772EVSEController

#ifdef RGBLCD
void doRgbLcdBtns()
{   
  //dummy code - not yet
#ifdef notyet
    uint8_t buttons = g_OBD.readButtons();

  if (buttons) {
    
    if (buttons & BUTTON_UP) {
      
    }
    if (buttons & BUTTON_DOWN) {
      
    }
    if (buttons & BUTTON_LEFT) {
      
    }
    if (buttons & BUTTON_RIGHT) {
      
    }
    if (buttons & BUTTON_SELECT) {
 
    }
  }
#endif // notyet
}
#endif //Adafruit RGB LCD

#ifdef BTN_MENU
Btn::Btn()
{
  buttonState = BTN_STATE_OFF;
  lastDebounceTime = 0;
}

void Btn::read()
{
  uint8_t sample = (analogRead(BTN_PIN) < 10) ? 1 : 0;
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
  g_OBD.LcdMsg(m_Title,firstitem);
}

SetupMenu::SetupMenu()
{
  m_Title = g_sSetup;
}

void SetupMenu::Init()
{
  init(g_SvcLevelMenu.m_Title);
}

void SetupMenu::ShortPress()
{
  if (++m_CurIdx >= 6) {
    m_CurIdx = 0;
  }
  const char *title;
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
    title = g_ResetMenu.m_Title;
    break;
  default:
    title = g_sExit;
    break;
  }
  g_OBD.LcdPrint(1,title);
}

Menu *SetupMenu::LongPress()
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
  else if (m_CurIdx == 4) {
    return &g_ResetMenu;
  }
  else {
    return NULL;
  }
}

char *g_SvcLevelMenuItems[] = {"Level 1","Level 2"};
SvcLevelMenu::SvcLevelMenu()
{
  m_Title = g_sSvcLevel;
}

void SvcLevelMenu::Init()
{
  m_CurIdx = (g_EvseController.GetCurSvcLevel() == 1) ? 0 : 1;
  sprintf(g_sTmp,"+%s",g_SvcLevelMenuItems[m_CurIdx]);
  g_OBD.LcdMsg(m_Title,g_sTmp);
}

void SvcLevelMenu::ShortPress()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0,1);
  if (g_EvseController.GetCurSvcLevel() == (m_CurIdx+1)) {
    g_OBD.LcdPrint("+");
  }
  g_OBD.LcdPrint(g_SvcLevelMenuItems[m_CurIdx]);
}

Menu *SvcLevelMenu::LongPress()
{
  g_EvseController.SetSvcLevel(m_CurIdx+1);
  g_OBD.LcdPrint(0,1,"+");
  g_OBD.LcdPrint(g_SvcLevelMenuItems[m_CurIdx]);

  EEPROM.write(EOFS_FLAGS,g_EvseController.GetFlags());

  delay(500);

  return &g_SetupMenu;
}

uint8_t g_L1MaxAmps[] = {6,10,12,15};
uint8_t g_L2MaxAmps[] = {10,16,20,25,30,35,40,45,50,55,60,65,70,75,80};
MaxCurrentMenu::MaxCurrentMenu()
{
  m_Title = g_sMaxCurrent;
}


void MaxCurrentMenu::Init()
{
  m_CurIdx = 0;
  uint8_t cursvclvl = g_EvseController.GetCurSvcLevel();
  m_MaxAmpsList = (cursvclvl == 1) ? g_L1MaxAmps : g_L2MaxAmps;
  m_MaxCurrent = 0;
  uint8_t currentlimit = (cursvclvl == 1) ? MAX_CURRENT_CAPACITY_L1 : MAX_CURRENT_CAPACITY_L2;

  m_MaxIdx = -1;
  do {
    m_MaxCurrent = m_MaxAmpsList[++m_MaxIdx];
    if (m_MaxCurrent == g_EvseController.GetCurrentCapacity()) {
      m_CurIdx = m_MaxIdx;
      break;
    }
  } while (m_MaxCurrent < currentlimit);
  
  sprintf(g_sTmp,"%s Max Current",(cursvclvl == 1) ? "L1" : "L2");
  g_OBD.LcdPrint(0,g_sTmp);
  sprintf(g_sTmp,"+%dA",m_MaxAmpsList[m_CurIdx]);
  g_OBD.LcdPrint(1,g_sTmp);
}

void MaxCurrentMenu::ShortPress()
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

Menu *MaxCurrentMenu::LongPress()
{
  g_OBD.LcdPrint(0,1,"+");
  g_OBD.LcdPrint(m_MaxAmpsList[m_CurIdx]);
  g_OBD.LcdPrint("A");
  delay(500);
  EEPROM.write((g_EvseController.GetCurSvcLevel() == 1) ? EOFS_CURRENT_CAPACITY_L1 : EOFS_CURRENT_CAPACITY_L2,m_MaxAmpsList[m_CurIdx]);  
  g_EvseController.SetCurrentCapacity(m_MaxAmpsList[m_CurIdx]);
  return &g_SetupMenu;
}

char *g_DiodeChkMenuItems[] = {"Yes","No"};
DiodeChkMenu::DiodeChkMenu()
{
  m_Title = g_sDiodeCheck;
}

void DiodeChkMenu::Init()
{
  m_CurIdx = g_EvseController.DiodeCheckEnabled() ? 0 : 1;
  sprintf(g_sTmp,"+%s",g_DiodeChkMenuItems[m_CurIdx]);
  g_OBD.LcdMsg(m_Title,g_sTmp);
}

void DiodeChkMenu::ShortPress()
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
  g_OBD.LcdPrint(g_DiodeChkMenuItems[m_CurIdx]);
}

Menu *DiodeChkMenu::LongPress()
{
  g_OBD.LcdPrint(0,1,"+");
  g_OBD.LcdPrint(g_DiodeChkMenuItems[m_CurIdx]);

  g_EvseController.EnableDiodeCheck((m_CurIdx == 0) ? 1 : 0);

  EEPROM.write(EOFS_FLAGS,g_EvseController.GetFlags());

  delay(500);

  return &g_SetupMenu;
}

char *g_VentReqMenuItems[] = {"Yes","No"};
VentReqMenu::VentReqMenu()
{
  m_Title = g_sVentReqChk;
}

void VentReqMenu::Init()
{
  m_CurIdx = g_EvseController.VentReqEnabled() ? 0 : 1;
  sprintf(g_sTmp,"+%s",g_VentReqMenuItems[m_CurIdx]);
  g_OBD.LcdMsg(m_Title,g_sTmp);
}

void VentReqMenu::ShortPress()
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
  g_OBD.LcdPrint(g_VentReqMenuItems[m_CurIdx]);
}

Menu *VentReqMenu::LongPress()
{
  g_OBD.LcdPrint(0,1,"+");
  g_OBD.LcdPrint(g_VentReqMenuItems[m_CurIdx]);

  g_EvseController.EnableVentReq((m_CurIdx == 0) ? 1 : 0);

  EEPROM.write(EOFS_FLAGS,g_EvseController.GetFlags());

  delay(500);

  return &g_SetupMenu;
}

char *g_ResetMenuItems[] = {"Yes","No"};
ResetMenu::ResetMenu()
{
  m_Title = g_sReset;
}

void ResetMenu::Init()
{
  m_CurIdx = 0;
  g_OBD.LcdMsg("Reset Now?",g_ResetMenuItems[0]);
}

void ResetMenu::ShortPress()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdPrint(0,1,g_ResetMenuItems[m_CurIdx]);
}

Menu *ResetMenu::LongPress()
{
  g_OBD.LcdPrint(0,1,"+");
  g_OBD.LcdPrint(g_ResetMenuItems[m_CurIdx]);
  delay(500);
  if (m_CurIdx == 0) {
    // hardware reset by forcing watchdog to timeout
    wdt_enable(WDTO_500MS);   // enable watchdog timer
    delay(2000);
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
      m_CurMenu->ShortPress();
    }
    else {
      if (g_EvseController.GetState() == EVSE_STATE_DISABLED) {
	g_EvseController.Enable();
      }
      else {
	g_EvseController.Disable();
      }
    }
  }
  else if (m_Btn.longPress()) {
    if (m_CurMenu) {
      m_CurMenu = m_CurMenu->LongPress();
      if (m_CurMenu) {
	uint8_t curidx;
	if (m_CurMenu == &g_SetupMenu) {
	  curidx = g_SetupMenu.m_CurIdx;
	}
	m_CurMenu->Init();
	if (m_CurMenu == &g_SetupMenu) {
	  // restore prev menu item
	  g_SetupMenu.m_CurIdx = curidx-1;
	  g_SetupMenu.ShortPress();
	}
      }
      else {
	g_EvseController.Enable();
      }
    }
    else {
      g_EvseController.Disable();
      g_SetupMenu.Init();
      m_CurMenu = &g_SetupMenu;
    }
  }
}
#endif // BTN_MENU


void EvseReset()
{
  g_OBD.Init();

  g_EvseController.Init();

#ifdef SERIALCLI
  g_CLI.Init();
#endif // SERIALCLI

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
  
#ifdef RGBLCD //Adafruit RGB LCD    
  doRgbLcdBtns();
#endif //Adafruit RGB LCD

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


}

