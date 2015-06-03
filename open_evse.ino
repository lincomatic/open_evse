// -*- C++ -*-
/*
 * Open EVSE Firmware
 *
 * Copyright (c) 2011-2015 Sam C. Lin <lincomatic@gmail.com>
 * Copyright (c) 2011-2014 Chris Howell <chris1howell@msn.com>
 * timer code Copyright (c) 2013 Kevin L <goldserve1@hotmail.com>
 * portions Copyright (c) 2014-2015 Nick Sayer <nsayer@kfu.com>
 * portions Copyright (c) 2015 Craig Kirkpatrick

 Revised  Ver	By		Reason
 6/21/13  20b3	Scott Rubin	fixed LCD display bugs with RTC enabled
 6/25/13  20b4	Scott Rubin	fixed LCD display bugs, CLI fixes, when RTC disabled
 6/30/13  20b5	Scott Rubin	added LcdDetected() function, prevents hang if LCD not installed
 7/06/13  20b5	Scott Rubin	rewrote power detection in POST function for 1 or 2 relays
 7/11/13  20b5	Scott Rubin	skips POST if EV is connected, won't charge if open ground or stuck relay
 8/12/13  20b5b Scott Rubin    fix GFI error - changed gfi.Reset() to check for constant GFI signal
 8/26/13  20b6 Scott Rubin     add Stuck Relay State delay, fix Stuck Relay state exit (for Active E)
 9/20/13  20b7 Chris Howell    updated/tweaked/shortened CLI messages   
 10/25/14      Craig K         add smoothing to the Amperage readout
 3/1/15        Craig K         add TEMPERATURE_MONITORING
 3/7/15        Craig K         add KWH_RECORDING
  
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
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <pins_arduino.h>
#include "./Wire.h"
#include "./RTClib.h"
#include "open_evse.h"
// if using I2CLCD_PCF8574 uncomment below line  and comment out LiquidTWI2.h above
//#include <LiquidCrystal_I2C.h>
#ifdef TEMPERATURE_MONITORING
  #ifdef MCP9808_IS_ON_I2C
  #include "./Adafruit_MCP9808.h"  //  adding the ambient temp sensor to I2C
  #endif 
  #ifdef TMP007_IS_ON_I2C
  #include "./Adafruit_TMP007.h"   //  adding the TMP007 IR I2C sensor
  #endif 
#endif // TEMPERATURE_MONITORING

const char VERSTR[] PROGMEM = VERSION;


#if defined(BTN_MENU) || defined(SHOW_DISABLED_TESTS)
const char g_psSettings[] PROGMEM = "Settings";
const char g_psSetup[] PROGMEM = "Setup";
const char g_psSvcLevel[] PROGMEM = "Service Level";
const char g_psMaxCurrent[] PROGMEM = "Max Current";
const char g_psDiodeCheck[] PROGMEM = "Diode Check";
const char g_psVentReqChk[] PROGMEM = "Vent Req'd Check";
#ifdef RGBLCD
const char g_psBklType[] PROGMEM = "Backlight Type";
#endif
#ifdef ADVPWR
const char g_psGndChk[] PROGMEM = "Ground Check";
const char g_psRlyChk[] PROGMEM = "Stuck Relay Chk";
#endif // ADVPWR
#ifdef GFI_SELFTEST
const char g_psGfiTest[] PROGMEM = "GFI Self Test";
#endif
#endif // BTN_MENU || SHOW_DISABLED_TEST
#ifdef BTN_MENU
const char g_psReset[] PROGMEM = "Restart";
const char g_psExit[] PROGMEM = "Exit";
// Add additional strings - GoldServe
#ifdef AUTOSTART_MENU
const char g_psAutoStart[] PROGMEM = "Auto Start";
#endif //#ifdef AUTOSTART_MENU
#ifdef DELAYTIMER_MENU
const char g_psRTC[] PROGMEM = "Date/Time";
const char g_psRTC_Month[] PROGMEM = "Month";
const char g_psRTC_Day[] PROGMEM = "Day";
const char g_psRTC_Year[] PROGMEM = "Year";
const char g_psRTC_Hour[] PROGMEM = "Hour";
const char g_psRTC_Minute[] PROGMEM = "Minute";
const char g_psDelayTimer[] PROGMEM = "Delay Timer";
const char g_psDelayTimerStartHour[] PROGMEM = "Start Hour";
const char g_psDelayTimerStartMin[] PROGMEM = "Start Min";
const char g_psDelayTimerStopHour[] PROGMEM = "Stop Hour";
const char g_psDelayTimerStopMin[] PROGMEM = "Stop Min";
#endif // DELAYTIMER_MENU
#ifdef CHARGE_LIMIT
const char g_psChargeLimit[] PROGMEM = "Charge Limit";
#endif // CHARGE_LIMIT
#ifdef TIME_LIMIT
const char g_psTimeLimit[] PROGMEM = "Time Limit";
#endif // TIME_LIMIT

SettingsMenu g_SettingsMenu;
SetupMenu g_SetupMenu;
SvcLevelMenu g_SvcLevelMenu;
MaxCurrentMenu g_MaxCurrentMenu;
DiodeChkMenu g_DiodeChkMenu;
#ifdef RGBLCD
BklTypeMenu g_BklTypeMenu;
#endif // RGBLCD
#ifdef GFI_SELFTEST
GfiTestMenu g_GfiTestMenu;
#endif
VentReqMenu g_VentReqMenu;
#ifdef ADVPWR
GndChkMenu g_GndChkMenu;
RlyChkMenu g_RlyChkMenu;
#endif // ADVPWR
ResetMenu g_ResetMenu;
// Instantiate additional Menus - GoldServe
#ifdef AUTOSTART_MENU
AutoStartMenu g_AutoStartMenu;
#endif //#ifdef AUTOSTART_MENU
#if defined(DELAYTIMER_MENU)
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
#endif // DELAYTIMER_MENU
#ifdef CHARGE_LIMIT
ChargeLimitMenu g_ChargeLimitMenu;
#endif // CHARGE_LIMIT
#ifdef TIME_LIMIT
TimeLimitMenu g_TimeLimitMenu;
#endif // TIME_LIMIT


Menu *g_SettingsMenuList[] = {
#ifdef TIME_LIMIT
  &g_TimeLimitMenu,
#endif // TIME_LIMIT
#ifdef CHARGE_LIMIT
  &g_ChargeLimitMenu,
#endif // CHARGE_LIMIT
#ifdef DELAYTIMER_MENU
  &g_DelayMenu,
#endif // DELAYTIMER_MENU
#ifdef AUTOSTART_MENU
  &g_AutoStartMenu,
#endif // AUTOSTART_MENU
  &g_SetupMenu,
  &g_ResetMenu,
  NULL
};

Menu *g_SetupMenuList[] = {
#ifdef DELAYTIMER_MENU
  &g_RTCMenu,
#endif // DELAYTIMER_MENU
#ifdef RGBLCD
  &g_BklTypeMenu,
#endif // RGBLCD
  &g_SvcLevelMenu,
  &g_MaxCurrentMenu,
  &g_DiodeChkMenu,
  &g_VentReqMenu,
#ifdef ADVPWR
  &g_GndChkMenu,
  &g_RlyChkMenu,
#endif // ADVPWR
#ifdef GFI_SELFTEST
  &g_GfiTestMenu,
#endif // GFI_SELFTEST
  NULL
};

BtnHandler g_BtnHandler;
#endif // BTN_MENU
#ifdef DELAYTIMER
#define g_sHHMMfmt "%02d:%02d"
#endif// DELAYTIMER

//-- begin global variables

char g_sTmp[TMP_BUF_SIZE];

OnboardDisplay g_OBD;

// Instantiate RTC and Delay Timer - GoldServe
#ifdef RTC
RTC_DS1307 g_RTC;
DateTime g_CurrTime;

#if defined(RAPI)
void SetRTC(uint8_t y,uint8_t m,uint8_t d,uint8_t h,uint8_t mn,uint8_t s) {
  g_RTC.adjust(DateTime(y,m,d,h,mn,s));
}
void GetRTC(char *buf) {
  g_CurrTime = g_RTC.now();
  sprintf(buf,"%d %d %d %d %d %d",g_CurrTime.year()-2000,g_CurrTime.month(),g_CurrTime.day(),g_CurrTime.hour(),g_CurrTime.minute(),g_CurrTime.second());
}
#endif // RAPI
#endif // RTC
#ifdef DELAYTIMER
DelayTimer g_DelayTimer;
#ifdef DELAYTIMER_MENU
// Start variables to support RTC and Delay Timer - GoldServe
uint16_t g_year;
uint8_t g_month;
uint8_t g_day;
uint8_t g_hour;
uint8_t g_min;
uint8_t sec = 0;
#endif // DELAYTIMER_MENU
#endif // DELAYTIMER

#ifdef LCD16X2
#ifdef ADVPWR
const char g_psPwrOn[] PROGMEM = "Power On";
const char g_psSelfTest[] PROGMEM = "Self Test";
const char g_psAutoDetect[] PROGMEM = "Auto Detect";
const char g_psLevel1[] PROGMEM = "Svc Level: L1";
const char g_psLevel2[] PROGMEM = "Svc Level: L2";
const char g_psTestFailed[] PROGMEM = "TEST FAILED";
#endif // ADVPWR
const char g_psEvseError[] PROGMEM =  "EVSE ERROR";
const char g_psSvcReq[] PROGMEM =  "SERVICE REQUIRED";
const char g_psVentReq[] PROGMEM = "VENT REQUIRED";
const char g_psDiodeChkFailed[] PROGMEM = "DIODE CHECK";
const char g_psGfciFault[] PROGMEM = "GFCI FAULT";
const char g_psGfci[] PROGMEM = "GFCI";
#ifdef TEMPERATURE_MONITORING
const char g_psTemperatureFault[] PROGMEM = "OVER TEMPERATURE";
#endif
const char g_psNoGround[] PROGMEM = "NO GROUND";
const char g_psStuckRelay[] PROGMEM = "STUCK RELAY";
const char g_psDisabled[] PROGMEM =  "Disabled";
const char g_psWaiting[] PROGMEM =  "Waiting";
const char g_psSleeping[] PROGMEM = "Sleeping";
const char g_psEvConnected[] PROGMEM = "Connected";
#ifdef SHOW_DISABLED_TESTS
const char g_psDisabledTests[] PROGMEM = "TEST DISABLED";
#endif
#endif // LCD16X2

#ifdef TEMPERATURE_MONITORING
TempMonitor g_TempMonitor;
#endif // TEMPERATURE_MONITORING

#ifdef KWH_RECORDING
unsigned long g_WattHours_accumulated;
unsigned long g_WattSeconds;
#endif // KWH_RECORDING

#ifdef FT_ENDURANCE
int g_CycleCnt = -1;
long g_CycleHalfStart;
uint8_t g_CycleState;
#endif 

//-- end global variables

static inline void wiresend(uint8_t x) {
#if ARDUINO >= 100
  Wire.write((uint8_t)x);
#else
  Wire.send(x);
#endif
}

static inline uint8_t wirerecv(void) {
#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}

// if digits > 0, zero pad w/ to # digits
// WARNING: This function uses the *end* of g_sTmp as its buffer
char *u2a(unsigned long x,int8_t digits)
{
  int8_t d = digits;
  char *s = g_sTmp + sizeof(g_sTmp);
  *--s = 0;
  if (!x) {
    *--s = '0';
    d--;
  }
  else {
    for (; x; x/=10) {
      *--s = '0'+ x%10;
      d--;
    }
  }
  for (;d > 0;d--) {
    *--s = '0';
  }
  
  return s;
}

void EvseReset();

// wdt_init turns off the watchdog timer after we use it
// to reboot
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));
void wdt_init(void)
{
  MCUSR = 0;
  wdt_disable();

  return;
}


#ifdef TEMPERATURE_MONITORING
void TempMonitor::Init()
{
  m_Flags = 0;
  m_MCP9808_temperature = 230;  // 230 means 23.0C  Using an integer to save on floating point library use
  m_DS3231_temperature = 230;   // the DS3231 RTC has a built in temperature sensor
  m_TMP007_temperature = 230;

#ifdef MCP9808_IS_ON_I2C
  m_tempSensor.begin();
#endif // MCP9808_IS_ON_I2C

#ifdef TMP007_IS_ON_I2C
  m_tmp007.begin();
#endif // TMP007_IS_ON_I2C

  m_ampacity = g_EvseController.GetCurrentCapacity();  // Need to keep track of the user's original ampacity setting since temperature monitoring will throttle it back and need to later restore it
}

void TempMonitor::Read()
{
#ifdef TMP007_IS_ON_I2C
  m_TMP007_temperature = m_tmp007.readObjTempC10();   //  using the TI TMP007 IR sensor
#endif
#ifdef MCP9808_IS_ON_I2C
  m_MCP9808_temperature = m_tempSensor.readTempC10();  // for the MCP9808
  if (m_MCP9808_temperature == 2303) {
    m_MCP9808_temperature = 0; }  // return 0 if the sensor is not present on the I2C bus
#endif

       
#ifdef RTC
  // This code chunck below reads the DS3231 RTC's internal temperature sensor            
  Wire.beginTransmission(0x68);
  wiresend(uint8_t(0x0e));
  wiresend( 0x20 );               // write bit 5 to initiate conversion of temperature
  Wire.endTransmission();            
             
  Wire.beginTransmission(0x68);
  wiresend(uint8_t(0x11));
  Wire.endTransmission();
      
  Wire.requestFrom(0x68, 2);
  m_DS3231_temperature = 10 * wirerecv();	// Here's the MSB
  m_DS3231_temperature = m_DS3231_temperature + (5*(wirerecv()>>6))/2;  // keep the reading like 235 meaning 23.5C
  if (m_DS3231_temperature == 0x09FD) m_DS3231_temperature = 0;  // If the DS3231 is not present then return 0
  #ifdef OPENEVSE_2
    m_DS3231_temperature = 0;  // If the DS3231 is not present then return 0, OpenEVSE II does not use the DS3231
  #endif
#endif // RTC
}
#endif // TEMPERATURE_MONITORING


OnboardDisplay::OnboardDisplay()
#if defined(I2CLCD) || defined(RGBLCD)
#ifdef I2CLCD_PCF8574
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
  : m_Lcd(LCD_I2C_ADDR, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE)
#else
  : m_Lcd(LCD_I2C_ADDR,1)
#endif // I2CLCD_PCF8574
#endif // defined(I2CLCD) || defined(RGBLCD)
{
}


#if defined(DELAYTIMER)||defined(TIME_LIMIT)
const char CustomChar_0[8] PROGMEM = {0x0,0xe,0x15,0x17,0x11,0xe,0x0,0x0}; // clock
#endif
#ifdef DELAYTIMER
const char CustomChar_1[8] PROGMEM = {0x0,0x0,0xe,0xe,0xe,0x0,0x0,0x0}; // stop (cube)
const char CustomChar_2[8] PROGMEM = {0x0,0x8,0xc,0xe,0xc,0x8,0x0,0x0}; // play 
#endif // DELAYTIMER
#if defined(DELAYTIMER)||defined(CHARGE_LIMIT)
const char CustomChar_3[8] PROGMEM = {0x0,0xe,0xc,0x1f,0x3,0x6,0xc,0x8}; // lightning
#endif

void OnboardDisplay::Init()
{
  WDT_RESET();

#ifdef RGBLCD
  m_bFlags = 0;
#else
  m_bFlags = OBDF_MONO_BACKLIGHT;
#endif // RGBLCD

#ifdef GREEN_LED_REG
  pinGreenLed.init(GREEN_LED_REG,GREEN_LED_IDX,DigitalPin::OUT);
  SetGreenLed(0);
#endif
#ifdef RED_LED_REG
  pinRedLed.init(RED_LED_REG,RED_LED_IDX,DigitalPin::OUT);
  SetRedLed(0);
#endif

#ifdef LCD16X2
  LcdBegin(LCD_MAX_CHARS_PER_LINE, 2);
  LcdSetBacklightColor(WHITE);

#if defined(DELAYTIMER)||defined(TIME_LIMIT)
  memcpy_P(g_sTmp,CustomChar_0,8);
  m_Lcd.createChar(0, (uint8_t*)g_sTmp);
#endif
#ifdef DELAYTIMER
  memcpy_P(g_sTmp,CustomChar_1,8);
  m_Lcd.createChar(1, (uint8_t*)g_sTmp);
  memcpy_P(g_sTmp,CustomChar_2,8);
  m_Lcd.createChar(2, (uint8_t*)g_sTmp);
#endif //#ifdef DELAYTIMER
#if defined(DELAYTIMER)||defined(CHARGE_LIMIT)
  memcpy_P(g_sTmp,CustomChar_3,8);
  m_Lcd.createChar(3, (uint8_t*)g_sTmp);
#endif
  m_Lcd.clear();

#ifdef OPENEVSE_2
  LcdPrint_P(0,PSTR("Open EVSE II"));
#else
  LcdPrint_P(0,PSTR("Open EVSE"));
#endif
  LcdPrint_P(0,1,PSTR("Version "));
  LcdPrint_P(VERSTR);
  delay(1500);
  WDT_RESET();
#endif //#ifdef LCD16X2
}

#ifdef LCD16X2
void OnboardDisplay::LcdPrint(int x,int y,const char *s)
{ 
  m_Lcd.setCursor(x,y);
  m_Lcd.print(s); 
}

void OnboardDisplay::LcdPrint_P(const char PROGMEM *s)
{
  strncpy_P(m_strBuf,s,LCD_MAX_CHARS_PER_LINE);
  m_strBuf[LCD_MAX_CHARS_PER_LINE] = 0;
  m_Lcd.print(m_strBuf);
}

void OnboardDisplay::LcdPrint_P(int y,const char PROGMEM *s)
{
  strncpy_P(m_strBuf,s,LCD_MAX_CHARS_PER_LINE);
  m_strBuf[LCD_MAX_CHARS_PER_LINE] = 0;
  LcdPrint(y,m_strBuf);
}

void OnboardDisplay::LcdPrint_P(int x,int y,const char PROGMEM *s)
{
  strncpy_P(m_strBuf,s,LCD_MAX_CHARS_PER_LINE);
  m_strBuf[LCD_MAX_CHARS_PER_LINE] = 0;
  m_Lcd.setCursor(x,y);
  m_Lcd.print(m_strBuf);
}

void OnboardDisplay::LcdMsg_P(const char PROGMEM *l1,const char PROGMEM *l2)
{
  LcdPrint_P(0,l1);
  LcdPrint_P(1,l2);
}


// print at (0,y), filling out the line with trailing spaces
void OnboardDisplay::LcdPrint(int y,const char *s)
{
  m_Lcd.setCursor(0,y);
  uint8_t i,len = strlen(s);
  if (len > LCD_MAX_CHARS_PER_LINE)
    len = LCD_MAX_CHARS_PER_LINE;
  for (i=0;i < len;i++) {
    m_Lcd.write(s[i]);
  }
  for (i=len;i < LCD_MAX_CHARS_PER_LINE;i++) {
    m_Lcd.write(' ');
  }
}

void OnboardDisplay::LcdMsg(const char *l1,const char *l2)
{
  LcdPrint(0,l1);
  LcdPrint(1,l2);
}
#endif // LCD16X2


char g_sRdyLAstr[] = "L%d:%dA";
const char g_psReady[] PROGMEM = "Ready";
const char g_psCharging[] PROGMEM = "Charging";
void OnboardDisplay::Update(int8_t updmode)
{
  if (updateDisabled()) return;

  int i;
  uint8_t curstate = g_EvseController.GetState();
  uint8_t svclvl = g_EvseController.GetCurSvcLevel();

  if (g_EvseController.StateTransition() || (updmode != OBD_UPD_NORMAL)) {
    // Optimize function call - GoldServe
    sprintf(g_sTmp,g_sRdyLAstr,(int)svclvl,(int)g_EvseController.GetCurrentCapacity());
    
    switch(curstate) {
    case EVSE_STATE_A: // not connected
      SetGreenLed(1);
      SetRedLed(0);
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
      
 #ifdef KWH_RECORDING 
      sprintf(g_sTmp,"%5luWh",(g_WattSeconds / 3600) );
      LcdPrint(0,1,g_sTmp);

      sprintf(g_sTmp,"%6lukWh",(g_WattHours_accumulated / 1000));  // display accumulated kWh
      LcdPrint(7,1,g_sTmp);
 #endif // KWH_RECORDING
        
#endif //Adafruit RGB LCD
      // n.b. blue LED is off
      break;
    case EVSE_STATE_B: // connected/not charging
      SetGreenLed(1);
      SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(YELLOW);
      LcdClear();
      LcdSetCursor(0,0);
#ifdef CHARGE_LIMIT
      if (g_EvseController.GetChargeLimit()) {
	LcdWrite(3); // lightning
      }
#endif
#ifdef TIME_LIMIT
      if (g_EvseController.GetTimeLimit()) {
	LcdWrite(0); // clock
      }
#endif
      // Display Timer and Stop Icon - GoldServe
#ifdef DELAYTIMER
      g_DelayTimer.PrintTimerIcon();
#endif //#ifdef DELAYTIMER
      LcdPrint_P(g_psEvConnected);
      LcdPrint(10,0,g_sTmp);

      #ifdef KWH_RECORDING
      sprintf(g_sTmp,"%5luWh",(g_WattSeconds / 3600) );
      LcdPrint(0,1,g_sTmp);

      sprintf(g_sTmp,"%6lukWh",(g_WattHours_accumulated / 1000));  // Display accumulated kWh
      LcdPrint(7,1,g_sTmp);
      #endif // KWH_RECORDING
 
#endif //Adafruit RGB LCD
    // n.b. blue LED is off
      break;
    case EVSE_STATE_C: // charging
      SetGreenLed(0);
      SetRedLed(0);
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(TEAL);
      LcdClear();
      LcdSetCursor(0,0);
      // Display Timer and Stop Icon - GoldServe
#ifdef CHARGE_LIMIT
      if (g_EvseController.GetChargeLimit()) {
	LcdWrite(3); // lightning
      }
#endif
#ifdef TIME_LIMIT
      if (g_EvseController.GetTimeLimit()) {
	LcdWrite(0); // clock
      }
#endif
#ifdef DELAYTIMER
      g_DelayTimer.PrintTimerIcon();
#endif //#ifdef DELAYTIMER
      LcdPrint_P(g_psCharging);
#endif //Adafruit RGB LCD
      // n.b. blue LED is on
      break;
  case EVSE_STATE_D: // vent required
    SetGreenLed(0);
    SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
    LcdSetBacklightColor(RED);
    LcdMsg_P(g_psEvseError,g_psVentReq);
#endif //Adafruit RGB LCD
    // n.b. blue LED is off
    break;
  case EVSE_STATE_DIODE_CHK_FAILED:
    SetGreenLed(0);
    SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
    LcdSetBacklightColor(RED);
    LcdMsg_P(g_psEvseError,g_psDiodeChkFailed);
#endif //Adafruit RGB LCD
    // n.b. blue LED is off
    break;
  case EVSE_STATE_GFCI_FAULT:
    SetGreenLed(0);
    SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
    LcdSetBacklightColor(RED);
    LcdMsg_P(updmode == OBD_UPD_HARDFAULT ? g_psSvcReq : g_psEvseError,g_psGfciFault);
#endif //Adafruit RGB LCD
    // n.b. blue LED is off
    break;
#ifdef TEMPERATURE_MONITORING      
  case EVSE_STATE_OVER_TEMPERATURE:    // overtemp message in Red on the RGB LCD
    SetGreenLed(0);
    SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
    LcdSetBacklightColor(RED);
    LcdMsg_P(g_psSvcReq,g_psTemperatureFault);  //  SERVICE REQUIRED     OVER TEMPERATURE 
#endif
    break;
#endif //TEMPERATURE_MONITORING        
  case EVSE_STATE_NO_GROUND:
    SetGreenLed(0);
    SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
    LcdSetBacklightColor(RED);
    LcdMsg_P(updmode == OBD_UPD_HARDFAULT ? g_psSvcReq : g_psEvseError,g_psNoGround);
#endif //Adafruit RGB LCD
    // n.b. blue LED is off
    break;
  case EVSE_STATE_STUCK_RELAY:
    SetGreenLed(0);
    SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
    LcdSetBacklightColor(RED);
    LcdMsg_P(updmode == OBD_UPD_HARDFAULT ? g_psSvcReq : g_psEvseError,g_psStuckRelay);
#endif //Adafruit RGB LCD
    // n.b. blue LED is off
    break;
  case EVSE_STATE_DISABLED:
    SetGreenLed(0);
    SetRedLed(1);
#ifdef LCD16X2
    LcdSetBacklightColor(VIOLET);
    LcdClear();
    LcdSetCursor(0,0);
    LcdPrint_P(g_psDisabled);
    LcdPrint(10,0,g_sTmp);
#endif // LCD16X2
    break;
 case EVSE_STATE_GFI_TEST_FAILED:
    SetGreenLed(0);
    SetRedLed(1);
    LcdSetBacklightColor(RED);
    LcdMsg_P(g_psTestFailed,g_psGfci);
    break;
  case EVSE_STATE_SLEEPING:
    SetGreenLed(1);
    SetRedLed(1);
#ifdef LCD16X2
    LcdSetBacklightColor(VIOLET);
    LcdClear();
    LcdSetCursor(0,0);
    LcdPrint_P(g_psSleeping);
    LcdPrint(10,0,g_sTmp);
#endif // LCD16X2
    break;
  default:
    SetGreenLed(0);
    SetRedLed(1);
    // n.b. blue LED is off
  }
  }

#if defined(AMMETER) && defined(LCD16X2)
    if (((curstate == EVSE_STATE_C) || g_EvseController.AmmeterCalEnabled()) && AmmeterIsDirty()) {
  SetAmmeterDirty(0);

  uint32_t current = g_EvseController.GetChargingCurrent();

  if (current >= 1000) { // display only if > 1000
    int a = current / 1000;
    int ma = (current % 1000) / 100;
    if (ma > 9) {
      ma = 0;
      a++;
    }
    sprintf(g_sTmp,"%3d.%dA",a,ma);
  }
  else {
    strcpy_P(g_sTmp,PSTR("    0A"));
  }
  LcdPrint(10,0,g_sTmp);
 }
#endif // AMMETER

#if defined(DELAYTIMER) && defined(LCD16X2)
  DateTime curtime = g_RTC.now();
#endif //#ifdef DELAYTIMER

#ifdef LCD16X2
  if (curstate == EVSE_STATE_C) {
  time_t elapsedTime = g_EvseController.GetElapsedChargeTime();
  if (elapsedTime != g_EvseController.GetElapsedChargeTimePrev()) {   
   
    #ifdef KWH_RECORDING
      uint32_t current = g_EvseController.GetChargingCurrent();
    #ifdef VOLTMETER
      g_WattSeconds = g_WattSeconds + (g_EvseController.GetVoltage() / 1000 * current / 1000);
    #else
    if (g_EvseController.GetCurSvcLevel() == 2) {                        //  first verify L2 or L1
      g_WattSeconds =  g_WattSeconds + (VOLTS_FOR_L2 * current / 1000);  // accumulate Watt Seconds for Level2 charging
    }
    else {
      g_WattSeconds =  g_WattSeconds + (VOLTS_FOR_L1 * current / 1000);  // accumulate Watt Seconds for Level1 charging
    }
    #endif // VOLTMETER
    sprintf(g_sTmp,"%5luWh",(g_WattSeconds / 3600) );
    LcdPrint(0,1,g_sTmp);

    #ifdef VOLTMETER
      sprintf(g_sTmp," %3luV",(g_EvseController.GetVoltage() / 1000));  // Display voltage from OpenEVSE II
      LcdPrint(11,1,g_sTmp);
    #else
      sprintf(g_sTmp,"%6lukWh",(g_WattHours_accumulated / 1000));  // display accumulated kWh
      LcdPrint(7,1,g_sTmp);
    #endif // VOLTMETER
    #else // ! KWH_RECORDING
      LcdClearLine(0);
    #endif // KWH_RECORDING

#ifdef TEMPERATURE_MONITORING
    if ((g_TempMonitor.OverTemperature()) || TEMPERATURE_DISPLAY_ALWAYS)  {
      g_OBD.LcdClearLine(1);
      static const char *tempfmt = "%2d.%1dC";
#ifdef MCP9808_IS_ON_I2C
      if ( g_TempMonitor.m_MCP9808_temperature != 0 ) {   // it returns 0 if it is not present
	sprintf(g_sTmp,tempfmt,g_TempMonitor.m_MCP9808_temperature/10, g_TempMonitor.m_MCP9808_temperature % 10);  //  Ambient sensor near or on the LCD
	LcdPrint(0,1,g_sTmp);
      }
#endif

#ifdef RTC	
      if ( g_TempMonitor.m_DS3231_temperature != 0) {   // it returns 0 if it is not present
        sprintf(g_sTmp,tempfmt,g_TempMonitor.m_DS3231_temperature/10, g_TempMonitor.m_DS3231_temperature % 10);      //  sensor built into the DS3231 RTC Chip
	LcdPrint(5,1,g_sTmp);
      }
#endif
	
#ifdef TMP007_IS_ON_I2C
      if ( g_TempMonitor.m_TMP007_temperature != 0 ) {    // it returns 0 if it is not present
	sprintf(g_sTmp,tempfmt,g_TempMonitor.m_TMP007_temperature/10, g_TempMonitor.m_TMP007_temperature % 10);  //  Infrared sensor probably looking at 30A fuses
	LcdPrint(11,1,g_sTmp);
      }
#endif

      if (g_TempMonitor.BlinkAlarm() && g_TempMonitor.OverTemperatureShutdown()) { // Blink Red off-and-on while over the temperature shutdown limit, zero current should flow to the EV
	g_TempMonitor.SetBlinkAlarm(0);                                            // toggle the alarm flag so we can blink
	SetRedLed(1);
#ifdef LCD16X2 //Adafruit RGB LCD
	LcdSetBacklightColor(RED);
#endif //Adafruit RGB LCD            
      }
      else  {
	g_TempMonitor.SetBlinkAlarm(1);        // toggle the alarm flag so we can blink
	SetRedLed(0);
#ifdef LCD16X2 //Adafruit RGB LCD
	LcdSetBacklightColor(TEAL);
#endif
      }           
    }  // (g_TempMonitor.OverTemperature()) || TEMPERATURE_DISPLAY_ALWAYS) 
    else  {
      SetRedLed(0);          // restore the normal TEAL backlight in case it was left RED while last blinking
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(TEAL);
#endif
    }          
#else // !TEMPERATURE_MONITORING
 #ifndef KWH_RECORDING
        int h = hour(elapsedTime);          // display the elapsed charge time
        int m = minute(elapsedTime);
        int s = second(elapsedTime);
    #ifdef DELAYTIMER
      sprintf(g_sTmp,"%02d:%02d:%02d   %02d:%02d",h,m,s,(int)((curtime.hour() <= 23) ? curtime.hour() : 0),(int)((curtime.hour() <= 23) ? curtime.minute() : 0));
    #else
        sprintf(g_sTmp,"%02d:%02d:%02d",h,m,s);
    #endif //#ifdef DELAYTIMER
          
        LcdPrint(1,g_sTmp);
    #endif // KWH_RECORDING
#endif // TEMPERATURE_MONITORING
  }
 }

// Display a new stopped LCD screen with Delay Timers enabled - GoldServe
#ifdef DELAYTIMER
 else if (curstate == EVSE_STATE_SLEEPING) {
   if (memcmp(&curtime,&g_CurrTime,sizeof(curtime))) {
     LcdSetCursor(0,0);
     g_DelayTimer.PrintTimerIcon();
     LcdPrint_P(g_DelayTimer.IsTimerEnabled() ? g_psWaiting : g_psSleeping);
     //    sprintf(g_sTmp,"%02d:%02d \0\1",curtime.hour(),curtime.minute());
     sprintf(g_sTmp,"%02d:%02d:%02d",curtime.hour(),curtime.minute(),curtime.second());
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
       sprintf(g_sTmp,g_sRdyLAstr,(int)svclvl,(int)g_EvseController.GetCurrentCapacity());
       LcdPrint(10,0,g_sTmp);
     }
   }
 }
#endif // DELAYTIMER
#endif // LCD16X2

#if defined(DELAYTIMER) && defined(LCD16X2)
  g_CurrTime = curtime;
#endif //#ifdef DELAYTIMER

#ifdef FT_ENDURANCE
  LcdSetCursor(0,0);
  sprintf(g_sTmp,"%d %d",g_CycleCnt,(int)g_EvseController.ReadACPins());
  LcdPrint(g_sTmp);
#endif // FT_ENDURANCE
}




#ifdef BTN_MENU
Btn::Btn()
{
  buttonState = BTN_STATE_OFF;
  lastDebounceTime = 0;
  vlongDebounceTime = 0;
}

void Btn::init()
{
#ifdef BTN_REG
  pinBtn.init(BTN_REG,BTN_IDX,DigitalPin::INP_PU);
#endif
}

void Btn::read()
{
  uint8_t sample;
  unsigned long delta;
#ifdef ADAFRUIT_BTN
  sample = (g_OBD.readButtons() & BUTTON_SELECT) ? 1 : 0;
#else //!ADAFRUIT_BTN
  sample = btnPin.read() ? 0 : 1;
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
      delta = millis() - lastDebounceTime;

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
#ifdef RAPI
  else if (sample && vlongDebounceTime && (buttonState == BTN_STATE_LONG)) {
    if ((millis() - vlongDebounceTime) >= BTN_PRESS_VERYLONG) {
      vlongDebounceTime = 0;
      g_ERP.setWifiMode(WIFI_MODE_AP);
    }
  }
#endif // RAPI
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
    vlongDebounceTime = lastDebounceTime;
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

SettingsMenu::SettingsMenu()
{
  m_Title = g_psSettings;

  m_menuCnt = 0;
  while (g_SettingsMenuList[m_menuCnt]) {
    m_menuCnt++;
  }
}

#if defined(CHARGE_LIMIT)||defined(TIME_LIMIT)
void SettingsMenu::CheckSkipLimits()
{
  // only allow Charge Limit menu item if car connected and no error
  m_skipLimits = ((g_EvseController.GetState() == EVSE_STATE_B) || (g_EvseController.GetState() == EVSE_STATE_C)) ? 0 : 1;
}
#endif // CHARGE_LIMIT

void SettingsMenu::Init()
{
  m_CurIdx = 0;

#if defined(CHARGE_LIMIT)||defined(TIME_LIMIT)
  while (m_skipLimits && (
#if defined(CHARGE_LIMIT)
	 (g_SettingsMenuList[m_CurIdx] == &g_ChargeLimitMenu) ||
#endif
#if defined(TIME_LIMIT)
	 (g_SettingsMenuList[m_CurIdx] == &g_TimeLimitMenu)
#else
	 0
#endif
			  )) {
    m_CurIdx++;
  }
#endif // CHARGE_LIMIT || TIME_LIMIT

  g_OBD.LcdPrint_P(0,m_Title);
  g_OBD.LcdPrint_P(1,g_SettingsMenuList[m_CurIdx]->m_Title);
}

void SettingsMenu::Next()
{
  if (++m_CurIdx > m_menuCnt) {
    m_CurIdx = 0;
  }

#if defined(CHARGE_LIMIT)||defined(TIME_LIMIT)
  while (m_skipLimits && (
#if defined(CHARGE_LIMIT)
	 (g_SettingsMenuList[m_CurIdx] == &g_ChargeLimitMenu) ||
#endif
#if defined(TIME_LIMIT)
	 (g_SettingsMenuList[m_CurIdx] == &g_TimeLimitMenu)
#else
	 0
#endif
			  )) {
    m_CurIdx++;
  }
#endif // CHARGE_LIMIT || TIME_LIMIT

  const char PROGMEM *title;
  if (m_CurIdx < m_menuCnt) {
    title = g_SettingsMenuList[m_CurIdx]->m_Title;
  }
  else {
    title = g_psExit;
  }

  g_OBD.LcdPrint_P(1,title);
}

Menu *SettingsMenu::Select()
{
  if (m_CurIdx < m_menuCnt) {
    return g_SettingsMenuList[m_CurIdx];
  }
  else {
    g_OBD.LcdClear();
    return NULL;
  }
}

SetupMenu::SetupMenu()
{
  m_Title = g_psSetup;

  m_menuCnt = 0;
  while (g_SetupMenuList[m_menuCnt]) {
    m_menuCnt++;
  }
}

void SetupMenu::Init()
{
  m_CurIdx = 0;
  g_OBD.LcdPrint_P(0,m_Title);
  g_OBD.LcdPrint_P(1,g_SetupMenuList[0]->m_Title);
}

void SetupMenu::Next()
{
  if (++m_CurIdx > m_menuCnt) {
    m_CurIdx = 0;
  }

  const char PROGMEM *title;
  if (m_CurIdx < m_menuCnt) {
    title = g_SetupMenuList[m_CurIdx]->m_Title;
  }
  else {
    title = g_psExit;
  }

  g_OBD.LcdPrint_P(1,title);
}

Menu *SetupMenu::Select()
{
  if (m_CurIdx < m_menuCnt) {
    return g_SetupMenuList[m_CurIdx];
  }
  else {
    m_CurIdx = 0;
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

#ifdef RGBLCD
char *g_BklMenuItems[] = {"RGB","Monochrome"};
BklTypeMenu::BklTypeMenu()
{
  m_Title = g_psBklType;
}
void BklTypeMenu::Init()
{
  g_OBD.LcdPrint_P(0,m_Title);
  m_CurIdx = (g_BtnHandler.GetSavedLcdMode() == BKL_TYPE_RGB) ? 0 : 1;
  sprintf(g_sTmp,"+%s",g_BklMenuItems[m_CurIdx]);
  g_OBD.LcdPrint(1,g_sTmp);
}

void BklTypeMenu::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0,1);
  g_OBD.LcdPrint(g_sPlus);
  g_OBD.LcdPrint(g_BklMenuItems[m_CurIdx]);
}

Menu *BklTypeMenu::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_BklMenuItems[m_CurIdx]);

  g_EvseController.SetBacklightType((m_CurIdx == 0) ? BKL_TYPE_RGB : BKL_TYPE_MONO);
  g_BtnHandler.SetSavedLcdMode((m_CurIdx == 0) ? BKL_TYPE_RGB : BKL_TYPE_MONO);

  return &g_SetupMenu;

}
#endif // RGBLCD


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
  g_OBD.LcdPrint(0,1,g_sPlus);
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
    g_OBD.LcdPrint(g_sPlus);
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
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_SvcLevelMenuItems[m_CurIdx]);

  g_EvseController.SaveEvseFlags();

  delay(500);

  return &g_SetupMenu;
}

uint8_t g_L1MaxAmps[] = {6,10,12,14,15,16,0};
uint8_t g_L2MaxAmps[] = {10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,0};
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
    g_OBD.LcdPrint(g_sPlus);
  }
  g_OBD.LcdPrint(m_MaxAmpsList[m_CurIdx]);
  g_OBD.LcdPrint("A");
}

Menu *MaxCurrentMenu::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(m_MaxAmpsList[m_CurIdx]);
  g_OBD.LcdPrint("A");
  delay(500);
  eeprom_write_byte((uint8_t*)((g_EvseController.GetCurSvcLevel() == 1) ? EOFS_CURRENT_CAPACITY_L1 : EOFS_CURRENT_CAPACITY_L2),m_MaxAmpsList[m_CurIdx]);  
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
    g_OBD.LcdPrint(g_sPlus);
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
}

Menu *DiodeChkMenu::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);

  g_EvseController.EnableDiodeCheck((m_CurIdx == 0) ? 1 : 0);

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
    g_OBD.LcdPrint(g_sPlus);
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
}

Menu *GfiTestMenu::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);

  g_EvseController.EnableGfiSelfTest((m_CurIdx == 0) ? 1 : 0);

  delay(500);

  return &g_SetupMenu;
}
#endif // GFI_SELFTEST

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
    g_OBD.LcdPrint(g_sPlus);
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
}

Menu *VentReqMenu::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);

  g_EvseController.EnableVentReq((m_CurIdx == 0) ? 1 : 0);

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
    g_OBD.LcdPrint(g_sPlus);
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
}

Menu *GndChkMenu::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);

  g_EvseController.EnableGndChk((m_CurIdx == 0) ? 1 : 0);

  delay(500);

  return &g_SetupMenu;
}
RlyChkMenu::RlyChkMenu()
{
  m_Title = g_psRlyChk;
}

void RlyChkMenu::Init()
{
  g_OBD.LcdPrint_P(0,m_Title);
  m_CurIdx = g_EvseController.StuckRelayChkEnabled() ? 0 : 1;
  sprintf(g_sTmp,"+%s",g_YesNoMenuItems[m_CurIdx]);
  g_OBD.LcdPrint(1,g_sTmp);
}

void RlyChkMenu::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0,1);
  uint8_t dce = g_EvseController.StuckRelayChkEnabled();
  if ((dce && !m_CurIdx) || (!dce && m_CurIdx)) {
    g_OBD.LcdPrint(g_sPlus);
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
}

Menu *RlyChkMenu::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);

  g_EvseController.EnableStuckRelayChk((m_CurIdx == 0) ? 1 : 0);

  delay(500);

  return &g_SetupMenu;
}
#endif // ADVPWR

ResetMenu::ResetMenu()
{
  m_Title = g_psReset;
}

const char g_psResetNow[] PROGMEM = "Restart Now?";
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

#ifdef DELAYTIMER_MENU
// pluspos = -1 = suppress "+"
void DtsStrPrint1(uint16_t y,uint8_t mo,uint8_t d,uint8_t h,uint8_t mn,int8_t pluspos)
{
  *g_sTmp = 0;
  if (pluspos == 0) strcat(g_sTmp,g_sPlus);
  strcat(g_sTmp,u2a(mo,2));
  strcat(g_sTmp,g_sSlash);
  if (pluspos == 1) strcat(g_sTmp,g_sPlus);
  strcat(g_sTmp,u2a(d,2));
  strcat(g_sTmp,g_sSlash);
  if (pluspos == 2) strcat(g_sTmp,g_sPlus);
  strcat(g_sTmp,u2a(y,2));
  strcat(g_sTmp,g_sSpace);
  if (pluspos == 3) strcat(g_sTmp,g_sPlus);
  strcat(g_sTmp,u2a(h,2));
  strcat(g_sTmp,g_sColon);
  if (pluspos == 4) strcat(g_sTmp,g_sPlus);
  strcat(g_sTmp,u2a(mn,2));

  g_OBD.LcdPrint(1,g_sTmp);
}


RTCMenu::RTCMenu()
{
  m_Title = g_psRTC;
}
const char g_psSetDateTime[] PROGMEM = "Set Date/Time?";
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
  g_OBD.LcdPrint(0,1,g_sPlus);
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

  DtsStrPrint1(g_year,g_month,g_day,g_hour,g_min,0);
}
void RTCMenuMonth::Next()
{
  if (++m_CurIdx >= 13) {
    m_CurIdx = 1;
  }
  g_OBD.LcdClearLine(1);
  DtsStrPrint1(g_year,m_CurIdx,g_day,g_hour,g_min,0);
}
Menu *RTCMenuMonth::Select()
{
  g_month = m_CurIdx;
  DtsStrPrint1(g_year,m_CurIdx,g_day,g_hour,g_min,0);
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
  DtsStrPrint1(g_year,g_month,m_CurIdx,g_hour,g_min,1);
}
void RTCMenuDay::Next()
{
  if (++m_CurIdx >= 32) {
    m_CurIdx = 1;
  }
  g_OBD.LcdClearLine(1);
  DtsStrPrint1(g_year,g_month,m_CurIdx,g_hour,g_min,1);
}
Menu *RTCMenuDay::Select()
{
  g_day = m_CurIdx;
  DtsStrPrint1(g_year,g_month,m_CurIdx,g_hour,g_min,1);
  delay(500);
  return &g_RTCMenuYear;
}
RTCMenuYear::RTCMenuYear()
{
}
void RTCMenuYear::Init()
{
  g_OBD.LcdPrint_P(0,g_psRTC_Year);
  m_CurIdx = g_year;
  if (m_CurIdx < 15 || m_CurIdx > 25){
    m_CurIdx = 15;
    g_year = 15;
  }
  DtsStrPrint1(m_CurIdx,g_month,g_day,g_hour,g_min,2);
}
void RTCMenuYear::Next()
{
  if (++m_CurIdx > 25) {
    m_CurIdx = 15;
  }
  DtsStrPrint1(m_CurIdx,g_month,g_day,g_hour,g_min,2);
}
Menu *RTCMenuYear::Select()
{
  g_year = m_CurIdx;
  DtsStrPrint1(m_CurIdx,g_month,g_day,g_hour,g_min,2);
  delay(500);
  return &g_RTCMenuHour;
}
RTCMenuHour::RTCMenuHour()
{
}
void RTCMenuHour::Init()
{
  g_OBD.LcdPrint_P(0,g_psRTC_Hour);
  m_CurIdx = g_hour;
  DtsStrPrint1(g_year,g_month,g_day,m_CurIdx,g_min,3);
}
void RTCMenuHour::Next()
{
  if (++m_CurIdx >= 24) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  DtsStrPrint1(g_year,g_month,g_day,m_CurIdx,g_min,3);
}
Menu *RTCMenuHour::Select()
{
  g_hour = m_CurIdx;
  DtsStrPrint1(g_year,g_month,g_day,m_CurIdx,g_min,3);
  delay(500);
  return &g_RTCMenuMinute;
}
RTCMenuMinute::RTCMenuMinute()
{
}
void RTCMenuMinute::Init()
{
  g_OBD.LcdPrint_P(0,g_psRTC_Minute);
  m_CurIdx = g_min;
  DtsStrPrint1(g_year,g_month,g_day,g_hour,m_CurIdx,4);
}
void RTCMenuMinute::Next()
{
  if (++m_CurIdx >= 60) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  DtsStrPrint1(g_year,g_month,g_day,g_hour,m_CurIdx,4);
}
Menu *RTCMenuMinute::Select()
{
  g_min = m_CurIdx;
  DtsStrPrint1(g_year,g_month,g_day,g_hour,m_CurIdx,4);
  g_RTC.adjust(DateTime(g_year, g_month, g_day, g_hour, g_min, 0));
  delay(500);
  return &g_SetupMenu;
}
char *g_DelayMenuItems[] = {"Yes/No","Set Start","Set Stop"};

void HsStrPrint1(uint8_t h,uint8_t m,int8_t pluspos=-1)
{
  *g_sTmp = 0;
  if (pluspos == 0) strcat(g_sTmp,g_sPlus);
  strcat(g_sTmp,u2a(h,2));
  strcat(g_sTmp,g_sColon);
  if (pluspos == 1) strcat(g_sTmp,g_sPlus);
  strcat(g_sTmp,u2a(m,2));
  g_OBD.LcdPrint(1,g_sTmp);
}


DelayMenu::DelayMenu()
{
  m_Title = g_psDelayTimer;
}
void DelayMenu::Init()
{
  m_CurIdx = 0;
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
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
}
void DelayMenuEnableDisable::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);
  if (m_CurIdx == !g_DelayTimer.IsTimerEnabled()){
    g_OBD.LcdPrint(0,1,g_sPlus);
    g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
  } else {
    g_OBD.LcdPrint(0,1,g_YesNoMenuItems[m_CurIdx]);
  }
}
Menu *DelayMenuEnableDisable::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
  delay(500);
  if (m_CurIdx == 0) {
    g_DelayTimer.Enable();
  } else {
    g_DelayTimer.Disable();
  }
  return &g_SettingsMenu;
}
DelayMenuStartHour::DelayMenuStartHour()
{
}
void DelayMenuStartHour::Init()
{
  g_OBD.LcdPrint_P(0,g_psDelayTimerStartHour);
  g_hour = g_DelayTimer.GetStartTimerHour();
  g_min = g_DelayTimer.GetStartTimerMin();
  m_CurIdx = g_hour;
  HsStrPrint1(m_CurIdx,g_min,0);
}

void DelayMenuStartHour::Next()
{
  if (++m_CurIdx >= 24) {
    m_CurIdx = 0;
  }
  HsStrPrint1(m_CurIdx,g_min,0);
}
Menu *DelayMenuStartHour::Select()
{
  g_hour = m_CurIdx;
  HsStrPrint1(m_CurIdx,g_min,0);
  delay(500);
  return &g_DelayMenuStartMin;
}
DelayMenuStopHour::DelayMenuStopHour()
{
}
void DelayMenuStopHour::Init()
{
  g_OBD.LcdPrint_P(0,g_psDelayTimerStopHour);
  g_hour = g_DelayTimer.GetStopTimerHour();
  g_min = g_DelayTimer.GetStopTimerMin();
  m_CurIdx = g_hour;
  HsStrPrint1(m_CurIdx,g_min,0);
}
void DelayMenuStopHour::Next()
{
  if (++m_CurIdx >= 24) {
    m_CurIdx = 0;
  }
  HsStrPrint1(m_CurIdx,g_min,0);
}
Menu *DelayMenuStopHour::Select()
{
  g_hour = m_CurIdx;
  HsStrPrint1(m_CurIdx,g_min,0);
  delay(500);
  return &g_DelayMenuStopMin;
}
DelayMenuStartMin::DelayMenuStartMin()
{
}
void DelayMenuStartMin::Init()
{
  g_OBD.LcdPrint_P(0,g_psDelayTimerStartMin);
  m_CurIdx = g_min;
  HsStrPrint1(g_hour,m_CurIdx,1);
}

void DelayMenuStartMin::Next()
{
  if (++m_CurIdx >= 60) {
    m_CurIdx = 0;
  }
  HsStrPrint1(g_hour,m_CurIdx,1);
}
Menu *DelayMenuStartMin::Select()
{
  g_min = m_CurIdx;
  HsStrPrint1(g_hour,m_CurIdx,1);
  g_DelayTimer.SetStartTimer(g_hour, g_min);
  delay(500);
  return &g_SettingsMenu;
}
DelayMenuStopMin::DelayMenuStopMin()
{
}
void DelayMenuStopMin::Init()
{
  g_OBD.LcdPrint_P(0,g_psDelayTimerStopMin);
  m_CurIdx = g_min;
  HsStrPrint1(g_hour,m_CurIdx,1);
}

void DelayMenuStopMin::Next()
{
  if (++m_CurIdx >= 60) {
    m_CurIdx = 0;
  }
  HsStrPrint1(g_hour,m_CurIdx,1);
}
Menu *DelayMenuStopMin::Select()
{
  g_min = m_CurIdx;
  HsStrPrint1(g_hour,m_CurIdx,1);
  g_DelayTimer.SetStopTimer(g_hour, g_min);  // Set Timers
  delay(500);
  return &g_SettingsMenu;
}
#endif // DELAYTIMER_MENU
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
    g_OBD.LcdPrint(g_sPlus);
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
  
}
Menu *AutoStartMenu::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
  g_EvseController.EnableAutoStart((m_CurIdx == 0) ? 1 : 0);
  g_EvseController.SaveEvseFlags();
  delay(500);
  return &g_SettingsMenu;
}
#endif //#ifdef AUTOSTART_MENU

#ifdef CHARGE_LIMIT
uint8_t g_ChargeLimitList[] = {0,1,2,3,4,5,10,15,20,30,0};

ChargeLimitMenu::ChargeLimitMenu()
{
  m_Title = g_psChargeLimit;
}

void ChargeLimitMenu::showCurSel(uint8_t plus)
{
  *g_sTmp = 0;
  if (plus) strcpy(g_sTmp,g_sPlus);
  if (g_ChargeLimitList[m_CurIdx] == 0) {
    strcat(g_sTmp,"off");
  }
  else {
    strcat(g_sTmp,u2a(g_ChargeLimitList[m_CurIdx]));
    strcat(g_sTmp," kWh");
  }
  g_OBD.LcdPrint(1,g_sTmp);
}


void ChargeLimitMenu::Init()
{
  m_CurIdx = 0;
  m_kwhLimit = g_EvseController.GetChargeLimit();

  for (m_MaxIdx=1;g_ChargeLimitList[m_MaxIdx] != 0;m_MaxIdx++);

  if (m_kwhLimit) {
    for (uint8_t i=0;i <= m_MaxIdx;i++) {
      if (g_ChargeLimitList[i] == m_kwhLimit) {
	m_CurIdx = i;
	break;
      }
    }
  }
  else {
    m_CurIdx = 0;
    m_kwhLimit = 0;
  }
  
  g_OBD.LcdPrint_P(0,g_psChargeLimit);
  showCurSel(1);
}

void ChargeLimitMenu::Next()
{
  if (++m_CurIdx > m_MaxIdx) {
    m_CurIdx = 0;
  }
  showCurSel((m_kwhLimit == g_ChargeLimitList[m_CurIdx]) ? 1 : 0);
}

Menu *ChargeLimitMenu::Select()
{
  uint8_t limit = g_ChargeLimitList[m_CurIdx];
  showCurSel(1);
  g_EvseController.SetChargeLimit(limit);
#ifdef TIME_LIMIT
  if (limit) {
    g_EvseController.SetTimeLimit(0);
  }
#endif // TIME_LIMIT
  delay(500);
  return limit ? NULL : &g_SettingsMenu;
}

#endif // CHARGE_LIMIT


#ifdef TIME_LIMIT
// above 60min must be in half hour increments < 256min
// uint8_t g_TimeLimitList[] = {0,15,30,60,90,120,180,240,300,360,420,480,0}; // minutes
uint8_t g_TimeLimitList[] = {0,1,2,4,6,8,10,12,16,20,24,28,32,0}; // 15 min increments

TimeLimitMenu::TimeLimitMenu()
{
  m_Title = g_psTimeLimit;
}

void TimeLimitMenu::showCurSel(uint8_t plus)
{
  uint16_t limit = g_TimeLimitList[m_CurIdx] * 15;
  *g_sTmp = 0;
  if (plus) strcpy(g_sTmp,g_sPlus);
  if (limit == 0) {
    strcat(g_sTmp,"off");
  }
  else {
    if (limit < 60) {
      strcat(g_sTmp,u2a(limit));
      strcat(g_sTmp," min");
    }
    else {
      strcat(g_sTmp,u2a(limit / 60));
      if (limit % 60) { // assume == 30
	strcat(g_sTmp,".5");
      }
      strcat(g_sTmp," hr");
    }
  }
  g_OBD.LcdPrint(1,g_sTmp);
}


void TimeLimitMenu::Init()
{
  m_CurIdx = 0;
  m_timeLimit = g_EvseController.GetTimeLimit();

  for (m_MaxIdx=1;g_TimeLimitList[m_MaxIdx] != 0;m_MaxIdx++);

  if (m_timeLimit) {
    for (uint8_t i=0;i <= m_MaxIdx;i++) {
      if (g_TimeLimitList[i] == m_timeLimit) {
	m_CurIdx = i;
	break;
      }
    }
  }
  else {
    m_CurIdx = 0;
    m_timeLimit = 0;
  }
  
  g_OBD.LcdPrint_P(0,g_psTimeLimit);
  showCurSel(1);
}

void TimeLimitMenu::Next()
{
  if (++m_CurIdx > m_MaxIdx) {
    m_CurIdx = 0;
  }
  showCurSel((m_timeLimit == g_TimeLimitList[m_CurIdx]) ? 1 : 0);
}

Menu *TimeLimitMenu::Select()
{
  uint8_t limit = g_TimeLimitList[m_CurIdx];
  showCurSel(1);
  g_EvseController.SetTimeLimit(limit);
#ifdef CHARGE_LIMIT
  if (limit) {
    g_EvseController.SetChargeLimit(0);
  }
#endif // CHARGE_LIMIT
  delay(500);
  return limit ? NULL : &g_SettingsMenu;
}

#endif // TIME_LIMIT

Menu *ResetMenu::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
  delay(500);
  if (m_CurIdx == 0) {
    g_EvseController.Reboot();
  }
  return NULL;
}

BtnHandler::BtnHandler()
{
  m_CurMenu = NULL;
}

void BtnHandler::ChkBtn(int8_t nosleeptoggle)
{
  WDT_RESET();

  m_Btn.read();
  if (m_Btn.shortPress()) {
    if (m_CurMenu) {
      m_CurMenu->Next();
    }
    else {
      if (!nosleeptoggle) {
	if ((g_EvseController.GetState() == EVSE_STATE_DISABLED) ||
	    (g_EvseController.GetState() == EVSE_STATE_SLEEPING)) {
	  g_EvseController.Enable();
	}
	else {
	  g_EvseController.Sleep();
	}
      }
    }
  }
  else if (m_Btn.longPress()) {
    if (m_CurMenu) {
      m_CurMenu = m_CurMenu->Select();
      if (m_CurMenu) {
	uint8_t curidx;
	if ((m_CurMenu == &g_SettingsMenu)||(m_CurMenu == &g_SetupMenu)) {
	  curidx = m_CurMenu->m_CurIdx;
	}
	m_CurMenu->Init();
	if ((m_CurMenu == &g_SettingsMenu)||(m_CurMenu == &g_SetupMenu)) {
	  if (curidx > 0) {
	    // restore prev menu item
	    m_CurMenu->m_CurIdx = curidx-1;
	    m_CurMenu->Next();
	  }
	}

      }
      else { // exit
	if (nosleeptoggle) {
	  g_EvseController.Reboot();
	}
	else {
#if defined(DELAYTIMER)
	  if (!g_DelayTimer.IsTimerEnabled()){
	    g_EvseController.Enable();
	  }
#else  
	  g_EvseController.Enable();
#endif        
	  g_OBD.DisableUpdate(0);
	  g_OBD.LcdSetBacklightType(m_SavedLcdMode); // exiting menus - restore LCD mode
	  g_OBD.Update(OBD_UPD_FORCE);
	}
      }
    }
    else {
      if (nosleeptoggle) {
	g_SetupMenu.Init();
	m_CurMenu = &g_SetupMenu;
      }
      else {
#if defined(CHARGE_LIMIT) || defined(TIME_LIMIT)
	g_SettingsMenu.CheckSkipLimits();
#endif // CHARGE_LIMIT
	g_EvseController.Sleep();
	g_OBD.DisableUpdate(1);
	m_SavedLcdMode = g_OBD.IsLcdBacklightMono() ? BKL_TYPE_MONO : BKL_TYPE_RGB;
	g_OBD.LcdSetBacklightColor(WHITE);
	g_SettingsMenu.Init();
	m_CurMenu = &g_SettingsMenu;
      }
    }
  }
}
#endif // BTN_MENU

// Start Delay Timer Functions - GoldServe
#ifdef DELAYTIMER
void DelayTimer::Init() {
  //Read EEPROM settings
  uint8_t rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_FLAGS);
  if (rtmp == 0xff) { // uninitialized EEPROM
    m_DelayTimerEnabled = 0x00;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_FLAGS, m_DelayTimerEnabled);
  }
  else {
    m_DelayTimerEnabled = rtmp;
  }
  rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_START_HOUR);
  if (rtmp == 0xff) { // uninitialized EEPROM
    m_StartTimerHour = DEFAULT_START_HOUR;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_START_HOUR, m_StartTimerHour);
  }
  else {
    m_StartTimerHour = rtmp;
  }
  rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_START_MIN);
  if (rtmp == 0xff) { // uninitialized EEPROM
    m_StartTimerMin = DEFAULT_START_MIN;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_START_MIN, m_StartTimerMin);
  }
  else {
    m_StartTimerMin = rtmp;
  }
  rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_STOP_HOUR);
  if (rtmp == 0xff) { // uninitialized EEPROM
    m_StopTimerHour = DEFAULT_STOP_HOUR;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_STOP_HOUR, m_StopTimerHour);
  }
  else {
    m_StopTimerHour = rtmp;
  }
  rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_STOP_MIN);
  if (rtmp == 0xff) { // uninitialized EEPROM
    m_StopTimerMin = DEFAULT_STOP_MIN;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_STOP_MIN, m_StopTimerMin);
  }
  else {
    m_StopTimerMin = rtmp;
  }
}

void DelayTimer::CheckTime()
{
  unsigned long curms = millis();
  if ((curms - m_LastCheck) > 1000ul) {
    if (IsTimerEnabled() && IsTimerValid()){
      g_CurrTime = g_RTC.now();
      m_CurrHour = g_CurrTime.hour();
      m_CurrMin = g_CurrTime.minute();

      uint16_t startTimerMinutes = m_StartTimerHour * 60 + m_StartTimerMin; 
      uint16_t stopTimerMinutes = m_StopTimerHour * 60 + m_StopTimerMin;
      uint16_t currTimeMinutes = m_CurrHour * 60 + m_CurrMin;
      
      if (stopTimerMinutes < startTimerMinutes) {
	//End time is for next day 
	if ( ( (currTimeMinutes >= startTimerMinutes) && (currTimeMinutes > stopTimerMinutes) ) || 
             ( (currTimeMinutes <= startTimerMinutes) && (currTimeMinutes < stopTimerMinutes) ) ){
	  // Within time interval
          if (g_EvseController.GetState() == EVSE_STATE_SLEEPING) {
	    g_EvseController.Enable();
	  }           
	}
	else {
	  // S.Low 3/12/14 Added check at T+1 minute in case interrupt is late
	  if ((currTimeMinutes >= stopTimerMinutes)&&(currTimeMinutes <= stopTimerMinutes+1)) { 
	    // Not in time interval
	    g_EvseController.Sleep();         
	  }
	}
      }
      else { // not crossing midnite
	if ((currTimeMinutes >= startTimerMinutes) && (currTimeMinutes < stopTimerMinutes)) { 
	  // Within time interval
	  if (g_EvseController.GetState() == EVSE_STATE_SLEEPING) {
	    g_EvseController.Enable();
	  }          
	}
	else {
	  // S.Low 3/12/14 Added check at T+1 minute in case interrupt is late
	  if ((currTimeMinutes >= stopTimerMinutes)&&(currTimeMinutes <= stopTimerMinutes+1)) { 
	    // Not in time interval
	    g_EvseController.Sleep();          
	  }
	}
      }
    }
    m_LastCheck = curms;
  }
}
void DelayTimer::Enable(){
  m_DelayTimerEnabled = 0x01;
  eeprom_write_byte((uint8_t*)EOFS_TIMER_FLAGS, m_DelayTimerEnabled);
  g_EvseController.EnableAutoStart(0);
  g_EvseController.SaveSettings();
  CheckTime();
  g_OBD.Update(OBD_UPD_FORCE);
}
void DelayTimer::Disable(){
  m_DelayTimerEnabled = 0x00;
  eeprom_write_byte((uint8_t*)EOFS_TIMER_FLAGS, m_DelayTimerEnabled);
  g_EvseController.EnableAutoStart(1);
  g_EvseController.SaveSettings();
  g_OBD.Update(OBD_UPD_FORCE);
}
void DelayTimer::PrintTimerIcon(){
  //g_OBD.LcdClear();
  //g_OBD.LcdSetCursor(0,0);
  if (IsTimerEnabled() && IsTimerValid()){
#ifdef BTN_MENU
    g_OBD.LcdWrite(0x0);
#endif //#ifdef BTN_MENU
  }
}
// End Delay Timer Functions - GoldServe
#endif //#ifdef DELAYTIMER

void EvseReset()
{
  Wire.begin();

#ifdef RTC
  g_RTC.begin();
#ifdef DELAYTIMER
  g_DelayTimer.Init();
#endif  // DELAYTIMER
#endif // RTC

#ifdef SERIALCLI
  g_CLI.Init();
#endif // SERIALCLI

  g_OBD.Init();

  g_EvseController.Init();
}

void setup()
{
  wdt_disable();
  
  delay(200);  // give I2C devices time to be ready before running code that wants to initialize I2C devices.  Otherwise a hang can occur upon powerup.
  
  Serial.begin(SERIAL_BAUD);

#ifdef BTN_MENU
  g_BtnHandler.init();
#endif // BTN_MENU

  EvseReset();

#ifdef TEMPERATURE_MONITORING
  g_TempMonitor.Init();
#endif

  WDT_ENABLE();

#ifdef KWH_RECORDING
      if (eeprom_read_dword((uint32_t*)EOFS_KWH_ACCUMULATED) == 0xffffffff) { // Check for unitialized eeprom condition so it can begin at 0kWh
        eeprom_write_dword((uint32_t*)EOFS_KWH_ACCUMULATED,0); //  Set the four bytes to zero just once in the case of unitialized eeprom
      }

      g_WattHours_accumulated = eeprom_read_dword((uint32_t*)EOFS_KWH_ACCUMULATED);        // get the stored value for the kWh from eeprom
#endif // KWH_RECORDING
}  // setup()


void loop()
{
  WDT_RESET();

  g_EvseController.Update();

  g_OBD.Update();

  g_EvseController.ProcessInputs(0);
  
  // Delay Timer Handler - GoldServe
#ifdef DELAYTIMER
  g_DelayTimer.CheckTime();
#endif //#ifdef DELAYTIMER
}
