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
#include <Wire.h>
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
void GetVerStr(char *buf) { strcpy_P(buf,VERSTR); }

#ifdef LCD16X2
char *g_BlankLine = "                ";
#endif // LCD16X2

char g_sPlus[] = "+";
char g_sSlash[] = "/";
char g_sColon[] = ":";
char g_sSpace[] = " ";

#if defined(BTN_MENU) || defined(SHOW_DISABLED_TESTS)
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
const char g_psRlyChk[] PROGMEM = "Relay Check";
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
const char g_psRTC_Month[] PROGMEM = "Set Month";
const char g_psRTC_Day[] PROGMEM = "Set Day";
const char g_psRTC_Year[] PROGMEM = "Set Year";
const char g_psRTC_Hour[] PROGMEM = "Set Hour";
const char g_psRTC_Minute[] PROGMEM = "Set Minute";
const char g_psDelayTimer[] PROGMEM = "Delay Timer";
//const char g_psDelayTimerEnableDisable[] PROGMEM = "Enable/Disable Timer?";
const char g_psDelayTimerStartHour[] PROGMEM = "Set Start Hour";
const char g_psDelayTimerStartMin[] PROGMEM = "Set Start Min";
const char g_psDelayTimerStopHour[] PROGMEM = "Set Stop Hour";
const char g_psDelayTimerStopMin[] PROGMEM = "Set Stop Min";
#endif // DELAYTIMER_MENU

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

Menu *g_MenuList[] = {
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
#ifdef DELAYTIMER_MENU
  &g_RTCMenu,
  &g_DelayMenu,
#endif // DELAYTIMER_MENU
#ifdef AUTOSTART_MENU
  &g_AutoStartMenu,
#endif // AUTOSTART_MENU
  &g_ResetMenu,
  NULL
};

BtnHandler g_BtnHandler;
#endif // BTN_MENU
#ifdef DELAYTIMER
char *g_sHHMMfmt = "%02d:%02d";
#endif// DELAYTIMER

//-- begin global variables

char g_sTmp[64];

THRESH_DATA g_DefaultThreshData = {875,780,690,0,260};
J1772EVSEController g_EvseController;
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

#ifdef SERIALCLI
CLI g_CLI;

const char g_psEnabled[] PROGMEM = "enabled";
const char g_psDisabled[] PROGMEM = "disabled";
#endif // SERIALCLI

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
const char g_psStopped[] PROGMEM =  "Stopped";
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
unsigned long g_WattSeconds = 0;
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
char *u2a(unsigned x,int digits=0)
{
  int d = digits;
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
  for (;d;d--) {
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

void SaveEvseFlags()
{
  uint16_t flags = g_EvseController.GetFlags();
  eeprom_write_word((uint16_t *)EOFS_FLAGS,flags);
}

void SaveSettings()
{
  // n.b. should we add dirty bits so we only write the changed values? or should we just write them on the fly when necessary?
  eeprom_write_byte((uint8_t *)((g_EvseController.GetCurSvcLevel() == 1) ? EOFS_CURRENT_CAPACITY_L1 : EOFS_CURRENT_CAPACITY_L2),(byte)g_EvseController.GetCurrentCapacity());
  SaveEvseFlags();
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
  if (m_DS3231_temperature == 0x09FD) m_DS3231_temperature = 0xffff;  // If the DS3231 is not present then return this negative integer
  #ifdef OPENEVSE_2
    m_DS3231_temperature = 0xffff;  // If the DS3231 is not present then return this negative integer, OpenEVSE II does not use the DS3231
  #endif
#endif // RTC
}
#endif // TEMPERATURE_MONITORING

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
  print_P(PSTR("Firmware - Open EVSE V")); //CLI info
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

const char g_pson[] PROGMEM = "on";
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
   println_P(PSTR("amp - set current capacity"));
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

void CLI::println_P(const char PROGMEM *s)
{
  strncpy_P(m_strBuf,s,m_strBufLen);
  m_strBuf[m_strBufLen-1] = 0;
  println(m_strBuf);
}

void CLI::print_P(const char PROGMEM *s)
{
  strncpy_P(m_strBuf,s,m_strBufLen);
  m_strBuf[m_strBufLen-1] = 0;
  print(m_strBuf);
}

#endif // SERIALCLI

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
  m_strBufLen = LCD_MAX_CHARS_PER_LINE+1;
}


#ifdef DELAYTIMER
// custom icons - GoldServe
const char CustomChar_0[8] PROGMEM = {0x0,0xe,0x15,0x17,0x11,0xe,0x0,0x0};
const char CustomChar_1[8] PROGMEM = {0x0,0x0,0xe,0xe,0xe,0x0,0x0,0x0};
const char CustomChar_2[8] PROGMEM = {0x0,0x8,0xc,0xe,0xc,0x8,0x0,0x0};
#endif // DELAYTIMER
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

#ifdef DELAYTIMER
  memcpy_P(g_sTmp,CustomChar_0,8);
  m_Lcd.createChar(0, (uint8_t*)g_sTmp);
  memcpy_P(g_sTmp,CustomChar_1,8);
  m_Lcd.createChar(1, (uint8_t*)g_sTmp);
  memcpy_P(g_sTmp,CustomChar_2,8);
  m_Lcd.createChar(2, (uint8_t*)g_sTmp);
#endif //#ifdef DELAYTIMER

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


void OnboardDisplay::SetGreenLed(uint8_t state)
{
#ifdef GREEN_LED_REG
  pinGreenLed.write(state);
#endif
}

void OnboardDisplay::SetRedLed(uint8_t state)
{
#ifdef RED_LED_REG
  pinRedLed.write(state);
#endif
}

#ifdef LCD16X2
void OnboardDisplay::LcdPrint(int x,int y,const char *s)
{ 
  if (LcdDetected()) {
    m_Lcd.setCursor(x,y);
    m_Lcd.print(s); 
  }
}

void OnboardDisplay::LcdPrint_P(const char PROGMEM *s)
{
  if (LcdDetected()) {
    strncpy_P(m_strBuf,s,m_strBufLen);
    m_strBuf[m_strBufLen-1] = 0;
    m_Lcd.print(m_strBuf);
  }
}

void OnboardDisplay::LcdPrint_P(int y,const char PROGMEM *s)
{
  strncpy_P(m_strBuf,s,m_strBufLen);
  m_strBuf[m_strBufLen-1] = 0;
  LcdPrint(y,m_strBuf);
}

void OnboardDisplay::LcdPrint_P(int x,int y,const char PROGMEM *s)
{
  strncpy_P(m_strBuf,s,m_strBufLen);
  m_strBuf[m_strBufLen-1] = 0;
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
  if (LcdDetected()) {
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
    LcdPrint_P(g_psStopped);
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

  /*      int ma;
	  if (current < 1000) {
	  // nnnmA
	  ma = (int)current;
	  sprintf(g_sTmp,"%4dmA",ma);
	  }
	  else {
	  // nn.nA
	  int a = current / 1000;
	  ma = ((current % 1000) + 50) / 100;
	  sprintf(g_sTmp,"%3d.%dA",a,ma);
	  }
	  LcdPrint(10,0,g_sTmp);
  */
  //      sprintf(g_sTmp,"%lu ",current);
  //      Serial.print(g_sTmp);
  if (current >= 1000) { // display only if > 1000
    // nnA
    //int a = (current + 500) / 1000;
	
    // nn.nA
    int a = current / 1000;
    int ma = (current % 1000) / 100;
    if (ma > 9) {
      ma = 0;
      a++;
    }
    sprintf(g_sTmp,"%3d.%dA",a,ma);
  }
  else {
    strcpy(g_sTmp,"    0A");
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
    #ifdef OPENEVSE_2
      g_WattSeconds = g_WattSeconds + (g_EvseController.readVoltmeter() / 1000 * current / 1000);
    #else
    if (g_EvseController.GetCurSvcLevel() == 2) {                        //  first verify L2 or L1
      g_WattSeconds =  g_WattSeconds + (VOLTS_FOR_L2 * current / 1000);  // accumulate Watt Seconds for Level2 charging
    }
    else {
      g_WattSeconds =  g_WattSeconds + (VOLTS_FOR_L1 * current / 1000);  // accumulate Watt Seconds for Level1 charging
    }
    #endif // OPENEVSE_2   
    sprintf(g_sTmp,"%5luWh",(g_WattSeconds / 3600) );
    LcdPrint(0,1,g_sTmp);

    #ifdef OPENEVSE_2
      sprintf(g_sTmp," %3luV",(g_EvseController.readVoltmeter() / 1000));  // Display voltage from OpenEVSE II
      LcdPrint(11,1,g_sTmp);
    #else
      sprintf(g_sTmp,"%6lukWh",(g_WattHours_accumulated / 1000));  // display accumulated kWh
      LcdPrint(7,1,g_sTmp);
    #endif // OPENEVSE_2
    #else // ! KWH_RECORDING
      LcdPrint(0,1,g_BlankLine);
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
      if ( g_TempMonitor.m_DS3231_temperature != 0xffff) {   // it returns 0xffff if it is not present
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


#ifdef GFI
// interrupt service routing
void gfi_isr()
{
  g_EvseController.SetGfiTripped();
}


void Gfi::Init()
{
  pin.init(GFI_REG,GFI_IDX,DigitalPin::INP);
  // GFI triggers on rising edge
  attachInterrupt(GFI_INTERRUPT,gfi_isr,RISING);

#ifdef GFI_SELFTEST
  pinTest.init(GFITEST_REG,GFITEST_IDX,DigitalPin::OUT);
#endif

  Reset();
}


//RESET GFI LOGIC
void Gfi::Reset()
{
  WDT_RESET();

#ifdef GFI_SELFTEST
  testInProgress = 0;
  testSuccess = 0;
#endif // GFI_SELFTEST

  if (pin.read()) m_GfiFault = 1; // if interrupt pin is high, set fault
  else m_GfiFault = 0;
}

#ifdef GFI_SELFTEST

uint8_t Gfi::SelfTest()
{
  testInProgress = 1;
  testSuccess = 0;
  for(int i=0; i < GFI_TEST_CYCLES; i++) {
    pinTest.write(1);
    delayMicroseconds(GFI_PULSE_DURATION_US);
    pinTest.write(0);
    delayMicroseconds(GFI_PULSE_DURATION_US);
    if (testSuccess) break; // no need to keep trying.
  }

  // wait for GFI pin to clear
  do {
    delay(50);
    WDT_RESET();
  }
  while(pin.read());

#ifndef OPENEVSE_2
  // sometimes getting spurious GFI faults when testing just before closing
  // relay.
  // wait a little more for everything to settle down
  // this delay is needed only if 10uF cap is in the circuit, which makes the circuit
  // temporarily overly sensitive to trips until it discharges
  delay(1000);
#endif // OPENEVSE_2

  m_GfiFault = 0;
  testInProgress = 0;

  return !testSuccess;
}
#endif // GFI_SELFTEST
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
 
#if (PILOT_IDX == 1) // PB1
  DDRB |= _BV(PORTB1);
  TCCR1A |= _BV(COM1A1);
#else // PB2
  DDRB |= _BV(PORTB2);
  TCCR1A |= _BV(COM1B1);
#endif // PILOT_IDX
#else // fast PWM
  pin.init(PILOT_REG,PILOT_IDX,DigitalPin::OUT);
#endif

  SetState(PILOT_STATE_P12); // turns the pilot on 12V steady state
}


// no PWM pilot signal - steady state
// PILOT_STATE_P12 = steady +12V (EVSE_STATE_A - VEHICLE NOT CONNECTED)
// PILOT_STATE_N12 = steady -12V (EVSE_STATE_F - FAULT) 
void J1772Pilot::SetState(PILOT_STATE state)
{
  AutoCriticalSection asc;

#ifdef PAFC_PWM
  if (state == PILOT_STATE_P12) {
#if (PILOT_IDX == 1)
    OCR1A = TOP;
#else
    OCR1B = TOP;
#endif
  }
  else {
#if (PILOT_IDX == 1) // PB1
    OCR1A = 0;
#else // PB2
    OCR1B = 0;
#endif
  }
#else // fast PWM
  TCCR1A = 0; //disable pwm by turning off COM1A1,COM1A0,COM1B1,COM1B0
  pin.write((state == PILOT_STATE_P12) ? 1 : 0);
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


#if (PILOT_IDX == 1) // PB1
  OCR1A = cnt;
#else // PB2
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
    AutoCriticalSection asc;
    // Timer1 initialization:
    // 16MHz / 64 / (OCR1A+1) / 2 on digital 9
    // 16MHz / 64 / (OCR1A+1) on digital 10
    // 1KHz variable duty cycle on digital 10, 500Hz fixed 50% on digital 9
    // pin 10 duty cycle = (OCR1B+1)/(OCR1A+1)

    TCCR1A = _BV(COM1A0) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
    OCR1A = 249;

    // 10% = 24 , 96% = 239
    OCR1B = ocr1b;

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

J1772EVSEController::J1772EVSEController() :
  adcPilot(VOLT_PIN)
#ifdef CURRENT_PIN
  , adcCurrent(CURRENT_PIN)
#endif
#ifdef VOLTMETER_PIN
  , adcVoltMeter(VOLTMETER_PIN)
#endif
{
}

// use watchdog to perform a reset
void J1772EVSEController::Reboot()
{
  m_Pilot.SetState(PILOT_STATE_P12);

#ifdef LCD16X2
  g_OBD.LcdPrint_P(1,PSTR("Resetting..."));
#endif

  if (chargingIsOn()) {
   // give the EV some time to open its contactor in response to P12
    delay(3000);
  }

  // hardware reset by forcing watchdog to timeout
  wdt_enable(WDTO_1S);   // enable watchdog timer
  delay(1500);
}



#ifdef SHOW_DISABLED_TESTS
void J1772EVSEController::ShowDisabledTests()
{
  if (m_wFlags & (ECF_DIODE_CHK_DISABLED|
		  ECF_VENT_REQ_DISABLED|
		  ECF_GND_CHK_DISABLED|
		  ECF_STUCK_RELAY_CHK_DISABLED|
		  ECF_GFI_TEST_DISABLED)) {
    g_OBD.LcdSetBacklightColor(YELLOW);

    if (!DiodeCheckEnabled()) {
      g_OBD.LcdMsg_P(g_psDisabledTests,g_psDiodeCheck);
      delay(SHOW_DISABLED_DELAY);
    }
    if (!VentReqEnabled()) {
      g_OBD.LcdMsg_P(g_psDisabledTests,g_psVentReqChk);
      delay(SHOW_DISABLED_DELAY);
    }
    if (!GndChkEnabled()) {
      g_OBD.LcdMsg_P(g_psDisabledTests,g_psGndChk);
      delay(SHOW_DISABLED_DELAY);
    }
    if (!StuckRelayChkEnabled()) {
      g_OBD.LcdMsg_P(g_psDisabledTests,g_psRlyChk);
      delay(SHOW_DISABLED_DELAY);
    }
    if (!GfiSelfTestEnabled()) {
      g_OBD.LcdMsg_P(g_psDisabledTests,g_psGfiTest);
      delay(SHOW_DISABLED_DELAY);
    }

    g_OBD.LcdSetBacklightColor(WHITE);
  }
}
#endif //SHOW_DISABLED_TESTS

void J1772EVSEController::chargingOn()
{  // turn on charging current
  pinCharging.write(1);
#ifdef CHARGING2_REG
  pinCharging2.write(1);
#endif
#ifdef CHARGINGAC_REG
  pinChargingAC.write(1);
#endif
  m_bVFlags |= ECVF_CHARGING_ON;
  
  m_ChargeOnTime = now();
  m_ChargeOnTimeMS = millis();
}

void J1772EVSEController::chargingOff()
{ // turn off charging current
  pinCharging.write(0);
#ifdef CHARGING2_REG
  pinCharging2.write(0);
#endif
#ifdef CHARGINGAC_REG
  pinChargingAC.write(0);
#endif
  m_bVFlags &= ~ECVF_CHARGING_ON;

  m_ChargeOffTime = now();
  m_ChargeOffTimeMS = millis();

#ifdef AMMETER
  m_ChargingCurrent = 0;
#endif
} 

void J1772EVSEController::HardFault()
{
  g_SetupMenu.EnableExitItem(0);
  g_OBD.Update(OBD_UPD_HARDFAULT);
  while (1) ProcessInputs(1); // spin forever or until user resets via menu
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
    m_wFlags &= ~ECF_DIODE_CHK_DISABLED;
  }
  else {
    m_wFlags |= ECF_DIODE_CHK_DISABLED;
  }
  SaveEvseFlags();
}

#ifdef GFI_SELFTEST
void J1772EVSEController::EnableGfiSelfTest(uint8_t tf)
{
  if (tf) {
    m_wFlags &= ~ECF_GFI_TEST_DISABLED;
  }
  else {
    m_wFlags |= ECF_GFI_TEST_DISABLED;
  }
  SaveEvseFlags();
}
#endif

void J1772EVSEController::EnableVentReq(uint8_t tf)
{
  if (tf) {
    m_wFlags &= ~ECF_VENT_REQ_DISABLED;
  }
  else {
    m_wFlags |= ECF_VENT_REQ_DISABLED;
  }
  SaveEvseFlags();
}

#ifdef ADVPWR
void J1772EVSEController::EnableGndChk(uint8_t tf)
{
  if (tf) {
    m_wFlags &= ~ECF_GND_CHK_DISABLED;
  }
  else {
    m_NoGndRetryCnt = 0;
    m_NoGndStart = 0;
    m_wFlags |= ECF_GND_CHK_DISABLED;
  }
  SaveEvseFlags();
}

void J1772EVSEController::EnableStuckRelayChk(uint8_t tf)
{
  if (tf) {
    m_wFlags &= ~ECF_STUCK_RELAY_CHK_DISABLED;
  }
  else {
    m_wFlags |= ECF_STUCK_RELAY_CHK_DISABLED;
  }
  SaveEvseFlags();
}

void J1772EVSEController::EnableAutoSvcLevel(uint8_t tf)
{
  if (tf) {
    m_wFlags &= ~ECF_AUTO_SVC_LEVEL_DISABLED;
  }
  else {
    m_wFlags |= ECF_AUTO_SVC_LEVEL_DISABLED;
  }
  SaveEvseFlags();
}


#endif // ADVPWR

// Functions to support Auto Start feature - GoldServe
#ifdef MANUALSTART 
void J1772EVSEController::EnableAutoStart(uint8_t tf)
{
  if (tf) {
    m_wFlags &= ~ECF_AUTO_START_DISABLED;
  }
  else {
    m_wFlags |= ECF_AUTO_START_DISABLED;
  }
  SaveEvseFlags();
}
#endif //#ifdef MANUALSTART
void J1772EVSEController::EnableSerDbg(uint8_t tf)
{
  if (tf) {
    m_wFlags |= ECF_SERIAL_DBG;
  }
  else {
    m_wFlags &= ~ECF_SERIAL_DBG;
  }
  SaveEvseFlags();
}

#ifdef RGBLCD
int J1772EVSEController::SetBacklightType(uint8_t t,uint8_t update)
{
#ifdef RGBLCD
  g_OBD.LcdSetBacklightType(t,update);
  if (t == BKL_TYPE_MONO) m_wFlags |= ECF_MONO_LCD;
  else m_wFlags &= ~ECF_MONO_LCD;
  SaveEvseFlags();
  Serial.print("wflags");Serial.print(m_wFlags,HEX);
#endif // RGBLCD
  return 0;
}
#endif // RGBLCD
void J1772EVSEController::Enable()
{
#ifdef SLEEP_STATUS_REG
  if (m_EvseState == EVSE_STATE_SLEEPING) {
    pinSleepStatus.write(0);
  }
#endif // SLEEP_STATUS_REG

  m_PrevEvseState = EVSE_STATE_DISABLED;
  m_EvseState = EVSE_STATE_UNKNOWN;
  m_Pilot.SetState(PILOT_STATE_P12);
}

void J1772EVSEController::Disable()
{
  if (m_EvseState != EVSE_STATE_DISABLED) { 
    m_Pilot.SetState(PILOT_STATE_N12);
    m_EvseState = EVSE_STATE_DISABLED;
    // panic stop so we won't wait for EV to open its contacts first
    chargingOff();
    g_OBD.Update(OBD_UPD_FORCE);
#ifdef RAPI
    if (StateTransition()) {
      g_ERP.sendEvseState();
    }
#endif // RAPI
    // cancel state transition so g_OBD doesn't keep updating
    m_PrevEvseState = EVSE_STATE_DISABLED;
  }
}


void J1772EVSEController::Sleep()
{
  if (m_EvseState != EVSE_STATE_SLEEPING) {
    m_Pilot.SetState(PILOT_STATE_P12);
    m_EvseState = EVSE_STATE_SLEEPING;
#ifdef SLEEP_STATUS_REG
    pinSleepStatus.write(1);
#endif // SLEEP_STATUS_REG

    g_OBD.Update(OBD_UPD_FORCE);
#ifdef RAPI
    if (StateTransition()) {
      g_ERP.sendEvseState();
    }
#endif // RAPI
    // cancel state transition so g_OBD doesn't keep updating
    m_PrevEvseState = EVSE_STATE_SLEEPING;
    // try to prevent arcing of our relay by waiting for EV to open its contacts first
    // use the charge end time variable temporarily to count down
    // when to open the contacts in Update()
    m_ChargeOffTimeMS = millis();
  }
}

void J1772EVSEController::LoadThresholds()
{
  memcpy(&m_ThreshData,&g_DefaultThreshData,sizeof(m_ThreshData));
}

void J1772EVSEController::SetSvcLevel(uint8_t svclvl,uint8_t updatelcd)
{
#ifdef SERIALCLI
  if (SerDbgEnabled()) {
    g_CLI.printlnn();
    g_CLI.print_P(PSTR("SetSvcLevel: "));Serial.println((int)svclvl);
  }
#endif //#ifdef SERIALCLI
  if (svclvl == 2) {
    m_wFlags |= ECF_L2; // set to Level 2
  }
  else {
    svclvl = 1;
    m_wFlags &= ~ECF_L2; // set to Level 1
  }

  SaveEvseFlags();

  uint8_t ampacity =  eeprom_read_byte((uint8_t*)((svclvl == 1) ? EOFS_CURRENT_CAPACITY_L1 : EOFS_CURRENT_CAPACITY_L2));

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

  if (updatelcd) {
    g_OBD.Update(OBD_UPD_FORCE);
  }
}

#ifdef ADVPWR

// acpinstate : bit 1 = AC pin 1, bit0 = AC pin 2
uint8_t J1772EVSEController::ReadACPins()
{
#ifndef OPENEVSE_2
#ifdef SAMPLE_ACPINS
  //
  // AC pins are active low, so we set them high
  // and then if voltage is detected on a pin, it will go low
  //
  uint8_t ac1 = 2;
  uint8_t ac2 = 1;
  unsigned long startms = millis();
  
  do {
    if (ac1 && !pinAC1.read()) {
      ac1 = 0;
    }
    if (ac2 && !pinAC2.read()) {
      ac2 = 0;
    }
  } while ((ac1 || ac2) && ((millis() - startms) < AC_SAMPLE_MS));
  return ac1 | ac2;
#else // !SAMPLE_ACPINS
  return (pinAC1.read() ? 2 : 0) | (pinAC2.read() ? 1 : 0);
#endif // SAMPLE_ACPINS
#else
  // For OpenEVSE II, there is only ACLINE1_PIN, and it is
  // active *high*. '3' is the value for "both AC lines dead"
  // and '0' is the value for "both AC lines live". There is
  // no need to sample, as the hardware does a peak-hold.
  return (pinAC1.read() ? 0 : 3);
#endif // OPENEVSE_2
}



uint8_t J1772EVSEController::doPost()
{
  WDT_RESET();

  uint8_t RelayOff, Relay1, Relay2; //Relay Power status
  uint8_t svcState = UD;	// service state = undefined

#ifdef SERIALCLI
  if (SerDbgEnabled()) {
    g_CLI.print_P(PSTR("POST start..."));
  }
#endif //#ifdef SERIALCLI


  m_Pilot.SetState(PILOT_STATE_P12); //check to see if EV is plugged in

  g_OBD.SetRedLed(1); 
#ifdef LCD16X2 //Adafruit RGB LCD
  g_OBD.LcdMsg_P(g_psPwrOn,g_psSelfTest);
#endif //Adafruit RGB LCD 

  if (AutoSvcLevelEnabled()) {
#ifdef OPENEVSE_2
    // For OpenEVSE II, there is a voltmeter for auto L1/L2.
    unsigned long ac_volts = readVoltmeter();
    if (ac_volts > L2_VOLTAGE_THRESHOLD) {
      svcState = L2;
    } else {
      svcState = L1;
    }
#ifdef SERIALCLI
    if (SerDbgEnabled()) {
      g_CLI.print_P(PSTR("AC millivolts: "));Serial.println(ac_volts);
      g_CLI.print_P(PSTR("SvcState: "));Serial.println((int)svcState);
    }  
#endif //#ifdef SERIALCLI
#ifdef LCD16X2
    g_OBD.LcdMsg_P(g_psAutoDetect,(svcState == L2) ? g_psLevel2 : g_psLevel1);
#endif //LCD16x2

#else //!OPENEVSE_2

    delay(150); // delay reading for stable pilot before reading
    int reading = adcPilot.read(); //read pilot
#ifdef SERIALCLI
    if (SerDbgEnabled()) {
      g_CLI.printlnn();
      g_CLI.print_P(PSTR("Pilot: "));Serial.println((int)reading);
    }
#endif //#ifdef SERIALCLI

    m_Pilot.SetState(PILOT_STATE_N12);
    if (reading > 900) {  // IF EV is not connected its Okay to open the relay the do the L1/L2 and ground Check

      // save state with both relays off - for stuck relay state
      RelayOff = ReadACPins();
          
      // save state with Relay 1 on 
      pinCharging.write(1);
#ifdef CHARGINGAC_REG
      pinChargingAC.write(1);
#endif
      delay(RelaySettlingTime);
      Relay1 = ReadACPins();
      pinCharging.write(0);
#ifdef CHARGINGAC_REG
      pinChargingAC.write(0);
#endif
      delay(RelaySettlingTime); //allow relay to fully open before running other tests
          
      // save state for Relay 2 on
#ifdef CHARGING2_REG
      pinCharging2.write(1); 
#endif
      delay(RelaySettlingTime);
      Relay2 = ReadACPins();
#ifdef CHARGING2_REG
      pinCharging2.write(0); 
#endif
      delay(RelaySettlingTime); //allow relay to fully open before running other tests
        
      // decide input power state based on the status read  on L1 and L2
      // either 2 SPST or 1 DPST relays can be configured 
      // valid svcState is L1 - one hot, L2 both hot, OG - open ground both off, SR - stuck relay when shld be off 
      //  
      if (RelayOff == none) { // relay not stuck on when off
	switch ( Relay1 ) {
	case ( both ): //
	  if ( Relay2 == none ) svcState = L2;
	  if (StuckRelayChkEnabled()) {
	    if ( Relay2 != none ) svcState = SR;
	  }
	  break;
	case ( none ): //
	  if (GndChkEnabled()) {
	    if ( Relay2 == none ) svcState = OG;
	  }
	  if ( Relay2 == both ) svcState = L2;
	  if ( Relay2 == L1 || Relay2 == L2 ) svcState = L1;
	  break;
	case ( L1on ): // L1 or L2
	case ( L2on ):
	  if (StuckRelayChkEnabled()) {
	    if ( Relay2 != none ) svcState = SR;
	  }
	if ( Relay2 == none ) svcState = L1;
	if ( (Relay1 == L1on) && (Relay2 == L2on)) svcState = L2;
	if ( (Relay1 == L2on) && (Relay2 == L1on)) svcState = L2;
	break;
	} // end switch
      }
      else { // Relay stuck on
	if (StuckRelayChkEnabled()) {
	  svcState = SR;
	}
      }
#ifdef SERIALCLI
      if (SerDbgEnabled()) {
	g_CLI.print_P(PSTR("RelayOff: "));Serial.println((int)RelayOff);
	g_CLI.print_P(PSTR("Relay1: "));Serial.println((int)Relay1);
	g_CLI.print_P(PSTR("Relay2: "));Serial.println((int)Relay2);
	g_CLI.print_P(PSTR("SvcState: "));Serial.println((int)svcState);
      }  
#endif //#ifdef SERIALCLI

      // update LCD
#ifdef LCD16X2
      if (svcState == L1) g_OBD.LcdMsg_P(g_psAutoDetect,g_psLevel1);
      if (svcState == L2) g_OBD.LcdMsg_P(g_psAutoDetect,g_psLevel2);
      if ((svcState == OG) || (svcState == SR))  {
	g_OBD.LcdSetBacklightColor(RED);
      }
      if (svcState == OG) g_OBD.LcdMsg_P(g_psTestFailed,g_psNoGround);
      if (svcState == SR) g_OBD.LcdMsg_P(g_psTestFailed,g_psStuckRelay);
#endif // LCD16X2
    } // endif test, no EV is plugged in
    else {
      // since we can't auto detect, for safety's sake, we must set to L1
      svcState = L1;
      SetAutoSvcLvlSkipped(1);
      // EV connected.. do stuck relay check
      goto stuckrelaychk;
    }
#endif //#else OPENEVSE_2
  }
  else { // ! AutoSvcLevelEnabled
  stuckrelaychk:
    if (StuckRelayChkEnabled()) {
      RelayOff = ReadACPins();
      if ((RelayOff & 3) != 3) {
	svcState = SR;
#ifdef LCD16X2
	g_OBD.LcdMsg_P(g_psTestFailed,g_psStuckRelay);
#endif // LCD16X2
      }
    }
  } // endif AutoSvcLevelEnabled
  
#ifdef GFI_SELFTEST
  // only run GFI test if no fault detected above
  if (((svcState == UD)||(svcState == L1)||(svcState == L2)) &&
      GfiSelfTestEnabled()) {
    if (m_Gfi.SelfTest()) {
#ifdef LCD16X2
      g_OBD.LcdMsg_P(g_psTestFailed,g_psGfci);
#endif // LCD16X2
      svcState = FG;
    }
  }
#endif

  if ((svcState == OG)||(svcState == SR)||(svcState == FG)) {
    g_OBD.LcdSetBacklightColor(RED);
    g_OBD.SetGreenLed(0);
    g_OBD.SetRedLed(1);
  }
  else {
    g_OBD.SetRedLed(0);
  }
  m_Pilot.SetState(PILOT_STATE_P12);

#ifdef SERIALCLI
  if (SerDbgEnabled()) {
    g_CLI.print_P(PSTR("POST result: "));
    Serial.println((int)svcState);
  }
#endif //#ifdef SERIALCLI

  WDT_RESET();

  return svcState;
}
#endif // ADVPWR

void J1772EVSEController::ProcessInputs(uint8_t nosleeptoggle)
{
#ifdef RAPI
  g_ERP.doCmd(0);
#endif
#ifdef SERIALCLI
  g_CLI.getInput();
#endif // SERIALCLI
#ifdef BTN_MENU
  g_BtnHandler.ChkBtn(nosleeptoggle);
#endif
}

void J1772EVSEController::Init()
{
  // read settings from EEPROM
  uint16_t rflgs = eeprom_read_word((uint16_t*)EOFS_FLAGS);

#ifdef RGBLCD
  if ((rflgs != 0xffff) && (rflgs & ECF_MONO_LCD)) {
    g_OBD.LcdSetBacklightType(BKL_TYPE_MONO);
  }
#endif // RGBLCD

  pinCharging.init(CHARGING_REG,CHARGING_IDX,DigitalPin::OUT);
#ifdef CHARGING2_REG
  pinCharging2.init(CHARGING2_REG,CHARGING2_IDX,DigitalPin::OUT);
#endif
#ifdef CHARGINGAC_REG
  pinChargingAC.init(CHARGINGAC_REG,CHARGINGAC_IDX,DigitalPin::OUT);
#endif
#ifdef ACLINE1_REG
  pinAC1.init(ACLINE1_REG,ACLINE1_IDX,DigitalPin::INP_PU);
#endif
#ifdef ACLINE2_REG
  pinAC2.init(ACLINE2_REG,ACLINE2_IDX,DigitalPin::INP_PU);
#endif
#ifdef SLEEP_STATUS_REG
  pinSleepStatus.init(SLEEP_STATUS_REG,SLEEP_STATUS_IDX,DigitalPin::OUT);
#endif

#ifdef GFI
  m_Gfi.Init();
#endif // GFI

  chargingOff();

  m_Pilot.Init(); // init the pilot

  uint8_t svclvl = (uint8_t)DEFAULT_SERVICE_LEVEL;

  if (rflgs == 0xffff) { // uninitialized EEPROM
    m_wFlags = ECF_DEFAULT;
#ifdef RGBLCD
    if (DEFAULT_LCD_BKL_TYPE == BKL_TYPE_MONO) {
      m_wFlags |= ECF_MONO_LCD;
    }
#endif // RGBLCD
  }
  else {
    m_wFlags = rflgs;
    svclvl = GetCurSvcLevel();

  }

#ifdef NOCHECKS
  m_wFlags |= ECF_DIODE_CHK_DISABLED|ECF_VENT_REQ_DISABLED|ECF_GND_CHK_DISABLED|ECF_STUCK_RELAY_CHK_DISABLED|ECF_GFI_TEST_DISABLED;
#endif

#ifdef AMMETER
  m_AmmeterCurrentOffset = eeprom_read_word((uint16_t*)EOFS_AMMETER_CURR_OFFSET);
  m_CurrentScaleFactor = eeprom_read_word((uint16_t*)EOFS_CURRENT_SCALE_FACTOR);
  
  if (m_AmmeterCurrentOffset == 0x0000ffff) {
    m_AmmeterCurrentOffset = DEFAULT_AMMETER_CURRENT_OFFSET;
  }
  if (m_CurrentScaleFactor == 0x0000ffff) {
    m_CurrentScaleFactor = DEFAULT_CURRENT_SCALE_FACTOR;
  }
  
  m_AmmeterReading = 0;
  m_ChargingCurrent = 0;
  //  m_LastAmmeterReadMs = 0;
#endif // AMMETER

#ifndef RGBLCD
  m_wFlags |= ECF_MONO_LCD;
#endif

  m_bVFlags = ECVF_DEFAULT;
#ifdef GFI
  m_GfiRetryCnt = 0;
  m_GfiTripCnt = eeprom_read_byte((uint8_t*)EOFS_GFI_TRIP_CNT);
  if (m_GfiTripCnt == 255) {
    m_GfiTripCnt = 0;
  }
#endif // GFI
#ifdef ADVPWR
  m_NoGndRetryCnt = 0;
  m_NoGndTripCnt = eeprom_read_byte((uint8_t*)EOFS_NOGND_TRIP_CNT);
  if (m_NoGndTripCnt != 255) {
    m_NoGndTripCnt = 0;
  }

  m_StuckRelayStartTimeMS = 0;
  m_StuckRelayTripCnt = eeprom_read_byte((uint8_t*)EOFS_STUCK_RELAY_TRIP_CNT);
  if (m_StuckRelayTripCnt != 255) {
    m_StuckRelayTripCnt = 0;
  }

  m_NoGndRetryCnt = 0;
  m_NoGndStart = 0;
#endif // ADVPWR

  m_EvseState = EVSE_STATE_UNKNOWN;
  m_PrevEvseState = EVSE_STATE_UNKNOWN;


#ifdef ADVPWR

#ifdef FT_READ_AC_PINS
  while (1) {
    WDT_RESET();
    sprintf(g_sTmp,"%d",(int)ReadACPins());
    g_OBD.LcdMsg("AC Pins",g_sTmp);
  }
#endif // FT_READ_AC_PINS
 
#ifdef SHOW_DISABLED_TESTS
  ShowDisabledTests();
#endif
 
  g_SetupMenu.EnableExitItem(0);
  uint8_t fault; 
  do {
    fault = 0; // reset post fault
    uint8_t psvclvl = doPost(); // auto detect service level overrides any saved values
    
    if ((AutoSvcLevelEnabled()) && ((psvclvl == L1) || (psvclvl == L2)))  svclvl = psvclvl; //set service level
    if ((GndChkEnabled()) && (psvclvl == OG))  { m_EvseState = EVSE_STATE_NO_GROUND; fault = 1;} // set No Ground error
    if ((StuckRelayChkEnabled()) && (psvclvl == SR)) { m_EvseState = EVSE_STATE_STUCK_RELAY; fault = 1; } // set Stuck Relay error
#ifdef GFI_SELFTEST
    if ((GfiSelfTestEnabled()) && (psvclvl == FG)) { m_EvseState = EVSE_STATE_GFI_TEST_FAILED; fault = 1; } // set GFI test fail error
#endif
    if (fault) {
#ifdef UL_COMPLIANT
      // UL wants EVSE to hard fault until power cycle if POST fails
      while (1) { // spin forever
#endif
	unsigned long faultms = millis();
	// wait for GFI_TIMEOUT before retrying POST
	while ((millis() - faultms) < GFI_TIMEOUT) {
	  ProcessInputs(1);
	}
#ifdef UL_COMPLIANT
      }
#endif
    }
  } while ( fault && ( m_EvseState == EVSE_STATE_GFI_TEST_FAILED || m_EvseState == EVSE_STATE_NO_GROUND ||  m_EvseState == EVSE_STATE_STUCK_RELAY ));
#endif // ADVPWR  

  g_SetupMenu.EnableExitItem(1);

  SetSvcLevel(svclvl);

  // Start Manual Start Feature - GoldServe
#ifdef MANUALSTART
  if (AutoStartEnabled()){
    Enable();
  } else {
    Sleep();
  }
#endif //#ifdef MANUALSTART
  // End Manual Start Feature - GoldServe

  g_OBD.SetGreenLed(0);
}

void J1772EVSEController::ReadPilot(int *plow,int *phigh,int loopcnt)
{
  int pl = 1023;
  int ph = 0;

  // 1x = 114us 20x = 2.3ms 100x = 11.3ms
  for (int i=0;i < 100;i++) {
    int reading = adcPilot.read();  // measures pilot voltage
    
    if (reading > ph) {
      ph = reading;
    }
    else if (reading < pl) {
      pl = reading;
    }
  }

  *plow = pl;
  *phigh = ph;
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
  int plow;
  int phigh;

  unsigned long curms = millis();

  if (m_EvseState == EVSE_STATE_DISABLED) {
    // n.b. don't know why, but if we don't have the delay below
    // then doEnableEvse will always have packetBuf[2] == 0
    // so we can't re-enable the EVSE
    delay(5);
    return;
  }
  else if (m_EvseState == EVSE_STATE_SLEEPING) {
    if (chargingIsOn()) {
      ReadPilot(&plow,&phigh);
      // wait for pilot voltage to go > STATE C. This will happen if
      // a) EV reacts and goes back to state B (opens its contacts)
      // b) user pulls out the charge connector
      // if it doesn't happen within 3 sec, we'll just open our relay anyway
      if ((phigh >= m_ThreshData.m_ThreshBC)
	  || ((curms - m_ChargeOffTimeMS) >= 3000)) {
	chargingOff();
#ifdef FT_SLEEP_DELAY
	sprintf(g_sTmp,"SLEEP OPEN %d",(int)phigh);
	g_OBD.LcdMsg(g_sTmp,(phigh >= m_ThreshData.m_ThreshBC) ? "THRESH" : "TIMEOUT");
	delay(2000);
#endif
      }
    }
    return;
  }

  uint8_t prevevsestate = m_EvseState;
  uint8_t tmpevsestate = EVSE_STATE_UNKNOWN;
  uint8_t nofault = 1;

#ifdef ADVPWR
  uint8_t acpinstate = ReadACPins();
  
  if (chargingIsOn()) { // relay closed
    if ((curms - m_ChargeOnTimeMS) > GROUND_CHK_DELAY) {
      // ground check - can only test when relay closed
      if (GndChkEnabled() && ((acpinstate & 3) == 3)) {
	// bad ground
	tmpevsestate = EVSE_STATE_NO_GROUND;
	m_EvseState = EVSE_STATE_NO_GROUND;
	
	chargingOff(); // open the relay
	
	if ((prevevsestate != EVSE_STATE_NO_GROUND) && (m_NoGndTripCnt < 254)) {
	  m_NoGndTripCnt++;
	  eeprom_write_byte((uint8_t*)EOFS_NOGND_TRIP_CNT,m_NoGndTripCnt);
	}
	m_NoGndStart = curms;
	
	nofault = 0;
      }

      // if EV was plugged in during POST, we couldn't do AutoSvcLevel detection,
      // so we had to hardcode L1. During first charge session, we can probe and set to L2 if necessary
      if (AutoSvcLvlSkipped() && (m_EvseState == EVSE_STATE_C)) {
	if (!acpinstate) {
	  // set to L2
	  SetSvcLevel(2,1);
	}
	SetAutoSvcLvlSkipped(0);
      }
    }
  }
  else { // !chargingIsOn() - relay open
    if (prevevsestate == EVSE_STATE_NO_GROUND) {
      if (((m_NoGndRetryCnt < GFI_RETRY_COUNT) || (GFI_RETRY_COUNT == 255)) &&
	  ((curms - m_NoGndStart) > GFI_TIMEOUT)) {
	m_NoGndRetryCnt++;
      }
      else {
	tmpevsestate = EVSE_STATE_NO_GROUND;
	m_EvseState = EVSE_STATE_NO_GROUND;
	
	nofault = 0;
      }
    }
    else if (StuckRelayChkEnabled()) {    // stuck relay check - can test only when relay open
      if (((acpinstate & 3) != 3)) { // Stuck Relay reading
	if ((prevevsestate != EVSE_STATE_STUCK_RELAY) && !m_StuckRelayStartTimeMS) { //check for first occurence
	  m_StuckRelayStartTimeMS = curms; // mark start state
	}
        if ( ( ((curms - m_ChargeOffTimeMS) > STUCK_RELAY_DELAY) && //  charge off de-bounce
               ((curms - m_StuckRelayStartTimeMS) > STUCK_RELAY_DELAY) ) ||  // start delay de-bounce
	     (prevevsestate == EVSE_STATE_STUCK_RELAY) ) { // already in error state
	  // stuck relay
	  if ((prevevsestate != EVSE_STATE_STUCK_RELAY) && (m_StuckRelayTripCnt < 254)) {
	    m_StuckRelayTripCnt++;
	    eeprom_write_byte((uint8_t*)EOFS_STUCK_RELAY_TRIP_CNT,m_StuckRelayTripCnt);
	  }   
	  tmpevsestate = EVSE_STATE_STUCK_RELAY;
	  m_EvseState = EVSE_STATE_STUCK_RELAY;
	  nofault = 0;
	}
      } // end of stuck relay reading
      else m_StuckRelayStartTimeMS = 0; // not stuck - reset
    } // end of StuckRelayChkEnabled
  } // end of !chargingIsOn() - relay open
#endif // ADVPWR
   
#ifdef GFI
  if (m_Gfi.Fault()) {
    tmpevsestate = EVSE_STATE_GFCI_FAULT;
    m_EvseState = EVSE_STATE_GFCI_FAULT;

    if (prevevsestate != EVSE_STATE_GFCI_FAULT) {
      if (m_GfiTripCnt < 254) {
	m_GfiTripCnt++;
	eeprom_write_byte((uint8_t*)EOFS_GFI_TRIP_CNT,m_GfiTripCnt);
      }
      m_GfiRetryCnt = 0;
      m_GfiTimeout = curms + GFI_TIMEOUT;
    }
    else if (curms >= m_GfiTimeout) {
#ifdef FT_GFI_RETRY
      g_OBD.LcdMsg("Reset","GFI");
      delay(250);
#endif // FT_GFI_RETRY
      m_GfiRetryCnt++;

      if ((GFI_RETRY_COUNT != 255) && (m_GfiRetryCnt > GFI_RETRY_COUNT)) {
	HardFault();
      }
      else {
	m_Gfi.Reset();
	m_GfiTimeout = curms + GFI_TIMEOUT;
      }
    }

    nofault = 0;
  }
#endif // GFI


#ifdef TEMPERATURE_MONITORING                 //  A state for OverTemp fault
  if ((g_TempMonitor.m_TMP007_temperature >= TEMPERATURE_INFRARED_PANIC)  || 
      (g_TempMonitor.m_MCP9808_temperature >= TEMPERATURE_AMBIENT_PANIC)  ||
      (g_TempMonitor.m_DS3231_temperature >= TEMPERATURE_AMBIENT_PANIC))  { 
    tmpevsestate = EVSE_STATE_OVER_TEMPERATURE;
    m_EvseState = EVSE_STATE_OVER_TEMPERATURE;
    nofault = 0;
  }
#endif // TEMPERATURE_MONITORING

  if (nofault) {
    if ((prevevsestate == EVSE_STATE_GFCI_FAULT) ||
        (prevevsestate == EVSE_STATE_OVER_TEMPERATURE) ||
        (prevevsestate == EVSE_STATE_NO_GROUND) ||
	(prevevsestate == EVSE_STATE_STUCK_RELAY)) {
      // just got out of fault state - pilot back on
      m_Pilot.SetState(PILOT_STATE_P12);
      prevevsestate = EVSE_STATE_UNKNOWN;
      m_EvseState = EVSE_STATE_UNKNOWN;
    }

    ReadPilot(&plow,&phigh);

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

#ifdef FT_ENDURANCE
    if (nofault) {
      if (((tmpevsestate == EVSE_STATE_A)||(tmpevsestate == EVSE_STATE_B)) && (g_CycleCnt < 0)) {
        g_CycleCnt = 0;
        g_CycleHalfStart = curms;
        g_CycleState = EVSE_STATE_B;
      } 

      if (g_CycleCnt >= 0) {
        if (g_CycleState == EVSE_STATE_B) {
	  if ((curms - g_CycleHalfStart) >= 9000) {
	    g_CycleCnt++;
	    g_CycleHalfStart = curms;
	    tmpevsestate = EVSE_STATE_C;
	    g_CycleState = EVSE_STATE_C;
	  }
	  else tmpevsestate = EVSE_STATE_B;
        }
        else if (g_CycleState == EVSE_STATE_C) {
	  if ((curms - g_CycleHalfStart) >= 1000) {
	    g_CycleHalfStart = curms;
	    tmpevsestate = EVSE_STATE_B;
	    g_CycleState = EVSE_STATE_B;
	  }
	  else tmpevsestate = EVSE_STATE_C;
        }
      }
    }
#endif // FT_ENDURANCE

    // debounce state transitions
    if (tmpevsestate != prevevsestate) {
      if (tmpevsestate != m_TmpEvseState) {
        m_TmpEvseStateStart = curms;
      }
      else if ((curms - m_TmpEvseStateStart) >= ((tmpevsestate == EVSE_STATE_A) ? DELAY_STATE_TRANSITION_A : DELAY_STATE_TRANSITION)) {
        m_EvseState = tmpevsestate;
      }
    }
  } // nofault

  m_TmpEvseState = tmpevsestate;

#ifdef FT_GFI_RETRY
  if (nofault && (prevevsestate == EVSE_STATE_C) && 
      ((curms - m_ChargeOnTimeMS) > 10000)) {
    g_OBD.LcdMsg("Induce","Fault");
    for(int i = 0; i < GFI_TEST_CYCLES; i++) {
      pinTest.write(1);
      delayMicroseconds(GFI_PULSE_DURATION_US);
      pinTest.write(0);
      delayMicroseconds(GFI_PULSE_DURATION_US);
    }
  }
#endif // FT_GFI_RETRY

  
  // state transition
  if (m_EvseState != prevevsestate) {
    if (m_EvseState == EVSE_STATE_A) { // connected, not ready to charge
      chargingOff(); // turn off charging current
      m_Pilot.SetState(PILOT_STATE_P12);
      #ifdef KWH_RECORDING
        g_WattHours_accumulated = g_WattHours_accumulated + (g_WattSeconds / 3600);
        eeprom_write_dword((uint32_t*)EOFS_KWH_ACCUMULATED,g_WattHours_accumulated); 
      #endif // KWH_RECORDING
    }
    else if (m_EvseState == EVSE_STATE_B) { // connected 
      chargingOff(); // turn off charging current
      m_Pilot.SetPWM(m_CurrentCapacity);
      #ifdef KWH_RECORDING
        if (prevevsestate == EVSE_STATE_A) {
          g_WattSeconds = 0;
        }
      #endif
    }
    else if (m_EvseState == EVSE_STATE_C) {
      m_Pilot.SetPWM(m_CurrentCapacity);
#ifdef FT_GFI_LOCKOUT
      for(int i = 0; i < GFI_TEST_CYCLES; i++) {
	pinTest.write(1);
	delayMicroseconds(GFI_PULSE_DURATION_US);
	pinTest.write(0);
	delayMicroseconds(GFI_PULSE_DURATION_US);
      }
      g_OBD.LcdMsg("Closing","Relay");
      delay(150);
#endif // FT_GFI_LOCKOUT

#ifdef UL_GFI_SELFTEST
      // test GFI before closing relay
      if (GfiSelfTestEnabled() && m_Gfi.SelfTest()) {
       // GFI test failed - hard fault
        m_EvseState = EVSE_STATE_GFI_TEST_FAILED;
	HardFault();
      }
#endif // UL_GFI_SELFTEST

      #ifdef KWH_RECORDING
        if (prevevsestate == EVSE_STATE_A) {
          g_WattSeconds = 0;
        }
      #endif
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
#ifdef TEMPERATURE_MONITORING
    else if (m_EvseState == EVSE_STATE_OVER_TEMPERATURE) {
      // vehicle state Over Teperature within the EVSE
      m_Pilot.SetState(PILOT_STATE_P12); // Signal the EV to pause, high current should cease within five seconds
      
      while ((millis()-curms) < 5000) {
	wdt_reset();
      }
      chargingOff(); // open the EVSE relays hopefully the EV has already disconnected by now by the J1772 specification
      m_Pilot.SetState(PILOT_STATE_N12);  // This will tell the EV that the EVSE has major problems requiring disconnecting from the EV
      HardFault();
    }
#endif //TEMPERATURE_MONITORING
    else if (m_EvseState == EVSE_STATE_DIODE_CHK_FAILED) {
      chargingOff(); // turn off charging current
      // must leave pilot on so we can keep checking
      // N.B. J1772 specifies to go to State F (-12V) but we can't do that
      // and keep checking
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
#ifdef UL_COMPLIANT
      // per discussion w/ UL Fred Reyes 20150217
      // always hard fault stuck relay
      HardFault();
#endif // UL_COMPLIANT
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

#ifdef UL_COMPLIANT
  if (!nofault && (prevevsestate == EVSE_STATE_C)) {
    // if fault happens immediately (within 2 sec) after charging starts, hard fault
    if ((curms - m_ChargeOnTimeMS) <= 2000) {
      HardFault();
    }
  }
#endif // UL_COMPLIANT

  m_PrevEvseState = prevevsestate;

#ifdef AMMETER
  if (((m_EvseState == EVSE_STATE_C) && (m_CurrentScaleFactor > 0)) || AmmeterCalEnabled()) {
    
    readAmmeter();
    uint32_t ma = MovingAverage(m_AmmeterReading);
    if (ma != 0xffffffff) {
      m_ChargingCurrent = ma * m_CurrentScaleFactor - m_AmmeterCurrentOffset;  // subtract it
      if (m_ChargingCurrent < 0) {
	m_ChargingCurrent = 0;
      }
      g_OBD.SetAmmeterDirty(1);
      //	sprintf(g_sTmp,"*%lu %lu %lu*",ma,m_CurrentScaleFactor,m_ChargingCurrent);
      //	sprintf(g_sTmp,"%lu ",m_ChargingCurrent);
      //	sprintf(g_sTmp,"ma %lu ",ma);
      //	Serial.print(g_sTmp);
    }
    //    }
  }
#endif // AMMETER
  if (m_EvseState == EVSE_STATE_C) {
    m_ElapsedChargeTimePrev = m_ElapsedChargeTime;
    m_ElapsedChargeTime = now() - m_ChargeOnTime;

#ifdef TEMPERATURE_MONITORING
    if (m_ElapsedChargeTime != m_ElapsedChargeTimePrev) {
      g_TempMonitor.Read();
      if (!g_TempMonitor.OverTemperature() && ((g_TempMonitor.m_TMP007_temperature   >= TEMPERATURE_INFRARED_THROTTLE_DOWN ) ||  // any sensor reaching threshold trips action
					       (g_TempMonitor.m_MCP9808_temperature  >= TEMPERATURE_AMBIENT_THROTTLE_DOWN ) ||
					       (g_TempMonitor.m_DS3231_temperature  >= TEMPERATURE_AMBIENT_THROTTLE_DOWN ))) {   // Throttle back the L2 current advice to the EV
	g_TempMonitor.SetOverTemperature(1);
	SetCurrentCapacity(g_TempMonitor.m_ampacity / 2,0,1);     // set L2 to the throttled back level
      }
      
      if (g_TempMonitor.OverTemperature() && ((g_TempMonitor.m_TMP007_temperature   <= TEMPERATURE_INFRARED_RESTORE_AMPERAGE ) &&  // all sensors need to show return to lower levels
					      (g_TempMonitor.m_MCP9808_temperature  <= TEMPERATURE_AMBIENT_RESTORE_AMPERAGE  ) &&
					      (g_TempMonitor.m_DS3231_temperature  <= TEMPERATURE_AMBIENT_RESTORE_AMPERAGE  ))) {  // restore the original L2 current advice to the EV
	g_TempMonitor.SetOverTemperature(0);
	SetCurrentCapacity(g_TempMonitor.m_ampacity,0,1);    // set L2 to the user's original setting for L2 current
      }           
      
      
      if (!g_TempMonitor.OverTemperatureShutdown() && ((g_TempMonitor.m_TMP007_temperature   >= TEMPERATURE_INFRARED_SHUTDOWN ) ||  // any sensor reaching threshold trips action
						       (g_TempMonitor.m_MCP9808_temperature  >= TEMPERATURE_AMBIENT_SHUTDOWN  )  ||
						       (g_TempMonitor.m_DS3231_temperature  >= TEMPERATURE_AMBIENT_SHUTDOWN  ))) {   // Throttle back the L2 current advice to the EV
	g_TempMonitor.SetOverTemperatureShutdown(1);
        SetCurrentCapacity(g_TempMonitor.m_ampacity / 4,0,1);
      }
      
      if (g_TempMonitor.OverTemperatureShutdown() && ((g_TempMonitor.m_TMP007_temperature   <= TEMPERATURE_INFRARED_THROTTLE_DOWN ) &&  // all sensors need to show return to lower levels
						      (g_TempMonitor.m_MCP9808_temperature  <= TEMPERATURE_AMBIENT_THROTTLE_DOWN )  &&
						      (g_TempMonitor.m_DS3231_temperature  <= TEMPERATURE_AMBIENT_THROTTLE_DOWN ))) {   //  restore the throttled down current advice to the EV since things have cooled down again
	g_TempMonitor.SetOverTemperatureShutdown(0);
	SetCurrentCapacity(g_TempMonitor.m_ampacity / 2,0,1);    // set L2 to the throttled back level
        m_Pilot.SetPWM(m_CurrentCapacity);
      }    
    }
#endif // TEMPERATURE_MONITORING
  }
}

#ifdef CALIBRATE
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
    int i;
    for (i=0;i < 1000;i++) {
      reading = adcPilot.read();  // measures pilot voltage

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
#endif // CALIBRATE

int J1772EVSEController::SetCurrentCapacity(uint8_t amps,uint8_t updatelcd,uint8_t nosave)
{
  int rc = 0;
  uint8_t maxcurrentcap = (GetCurSvcLevel() == 1) ? MAX_CURRENT_CAPACITY_L1 : MAX_CURRENT_CAPACITY_L2;

  if ((amps >= MIN_CURRENT_CAPACITY) && (amps <= maxcurrentcap)) {
    m_CurrentCapacity = amps;
  }
  else if (amps < MIN_CURRENT_CAPACITY) {
    m_CurrentCapacity = MIN_CURRENT_CAPACITY;
    rc = 1;
  }
  else {
    m_CurrentCapacity = maxcurrentcap;
    rc = 2;
  }

  if (!nosave) {
    eeprom_write_byte((uint8_t*)((GetCurSvcLevel() == 1) ? EOFS_CURRENT_CAPACITY_L1 : EOFS_CURRENT_CAPACITY_L2),(byte)m_CurrentCapacity);
  }

  if (m_Pilot.GetState() == PILOT_STATE_PWM) {
    m_Pilot.SetPWM(m_CurrentCapacity);
  }

  if (updatelcd) {
    g_OBD.Update(OBD_UPD_FORCE);
  }

  return rc;
}

#ifdef OPENEVSE_2
// 35 ms is just a bit longer than 1.5 cycles at 50 Hz
#define VOLTMETER_POLL_INTERVAL (35)
// This is just a wild guess
// #define VOLTMETER_SCALE_FACTOR (266)     // original guess
#define VOLTMETER_SCALE_FACTOR (262)        // calibrated for Craig K OpenEVSE II build
// #define VOLTMETER_OFFSET_FACTOR (40000)  // original guess
#define VOLTMETER_OFFSET_FACTOR (46800)     // calibrated for Craig K OpenEVSE II build
unsigned long J1772EVSEController::readVoltmeter()
{
  unsigned int peak = 0;
  for(unsigned long start_time = millis(); (millis() - start_time) < VOLTMETER_POLL_INTERVAL; ) {
    unsigned int val = adcVoltMeter.read();
    if (val > peak) peak = val;
  }
  return ((unsigned long)peak) * VOLTMETER_SCALE_FACTOR + VOLTMETER_OFFSET_FACTOR;
}
#endif

#ifdef AMMETER
static inline unsigned long ulong_sqrt(unsigned long in)
{
  unsigned long out;
  // find the last int whose square is not too big
  // Yes, it's wasteful, but we only theoretically ever have to go to 512.
  // Removing floating point saves us almost 1K of flash.
  for(out = 1; out*out <= in; out++) ;
  return out - 1;
}

void J1772EVSEController::readAmmeter()
{
  WDT_RESET();

  unsigned long sum = 0;
  unsigned int zero_crossings = 0;
  unsigned long last_zero_crossing_time = 0, now_ms;
  long last_sample = -1; // should be impossible - the A/d is 0 to 1023.
  unsigned int sample_count = 0;
  for(unsigned long start = millis(); ((now_ms = millis()) - start) < CURRENT_SAMPLE_INTERVAL; ) {
    long sample = (long) adcCurrent.read();
    // If this isn't the first sample, and if the sign of the value differs from the
    // sign of the previous value, then count that as a zero crossing.
    if (last_sample != -1 && ((last_sample > 512) != (sample > 512))) {
      // Once we've seen a zero crossing, don't look for one for a little bit.
      // It's possible that a little noise near zero could cause a two-sample
      // inversion.
      if ((now_ms - last_zero_crossing_time) > CURRENT_ZERO_DEBOUNCE_INTERVAL) {
        zero_crossings++;
        last_zero_crossing_time = now_ms;
      }
    }
    last_sample = sample;
    switch(zero_crossings) {
    case 0: 
      continue; // Still waiting to start sampling
    case 1:
    case 2:
      // Gather the sum-of-the-squares and count how many samples we've collected.
      sum += (unsigned long)((sample - 512) * (sample - 512));
      sample_count++;
      continue;
    case 3:
      // The answer is the square root of the mean of the squares.
      // But additionally, that value must be scaled to a real current value.
      // we will do that elsewhere
      m_AmmeterReading = ulong_sqrt(sum / sample_count);
      return;
    }
  }
  // ran out of time. Assume that it's simply not oscillating any. 
  m_AmmeterReading = 0;

  WDT_RESET();
}

#define MA_PTS 32 // # points in moving average MUST BE power of 2
#define MA_BITS 5 // log2(MA_PTS)
/*
uint32_t MovingAverage(uint32_t samp)
{
  static uint32_t samps[MA_PTS] = {0};
  uint32_t tot = samp;
  samps[0] = samp;

  for (int8_t c=MA_PTS-1;c > 0;c--) {
  samps[c] = samps[c-1];
  tot += samps[c];
  }
  
  return tot >> MA_BITS;
}
*/

// to save memory
// instead of doing a real moving average we just do a non-overlapping
// sliding window and output a value every MA_PTS
uint32_t MovingAverage(uint32_t samp)
{
  static uint32_t tot = 0;
  static int8_t curidx = 0;
  
  if (curidx == 0) {
    tot = 0;
  }

  tot += samp;

  if (++curidx == MA_PTS) {
    curidx = 0;
    return tot >> MA_BITS; // tot / MA_PTS
  }
  return 0xffffffff;
}

#endif // AMMETER


//-- end J1772EVSEController

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

SetupMenu::SetupMenu()
{
  m_Title = g_psSetup;

  m_menuCnt = 0;
  while (g_MenuList[m_menuCnt]) {
    m_menuCnt++;
  }
}

void SetupMenu::Init()
{
  m_CurIdx = 0;
  g_OBD.LcdPrint_P(0,m_Title);
  g_OBD.LcdPrint_P(1,g_MenuList[0]->m_Title);
}

void SetupMenu::Next()
{
  if ((++m_CurIdx > m_menuCnt) ||
      (m_noExit && (m_CurIdx == m_menuCnt))) { // skip exit item
    m_CurIdx = 0;
  }

  const char PROGMEM *title;
  if (m_CurIdx < m_menuCnt) {
    title = g_MenuList[m_CurIdx]->m_Title;
  }
  else {
    title = g_psExit;
  }

  g_OBD.LcdPrint_P(1,title);
}

Menu *SetupMenu::Select()
{
  if (m_CurIdx < m_menuCnt) {
    return g_MenuList[m_CurIdx];
  }
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

  SaveEvseFlags();

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
  return &g_SetupMenu;
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
  return &g_SetupMenu;
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
  return &g_SetupMenu;
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
  SaveEvseFlags();
  delay(500);
  return &g_SetupMenu;
}
#endif //#ifdef AUTOSTART_MENU

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
#ifdef BTN_ENABLE_TOGGLE
      if (!nosleeptoggle) {
	if ((g_EvseController.GetState() == EVSE_STATE_DISABLED) ||
	    (g_EvseController.GetState() == EVSE_STATE_SLEEPING)) {
	  g_EvseController.Enable();
	}
	else {
	  g_EvseController.Sleep();
	}
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
      else { // exit
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
    else {
#ifndef BTN_ENABLE_TOGGLE
      uint8_t curstate = g_EvseController.GetState();
      if ((curstate != EVSE_STATE_B) && (curstate != EVSE_STATE_C)) {
#endif // !BTN_ENABLE_TOGGLE
	g_EvseController.Sleep();
	g_OBD.DisableUpdate(1);
	m_SavedLcdMode = g_OBD.IsLcdBacklightMono() ? BKL_TYPE_MONO : BKL_TYPE_RGB;
	g_OBD.LcdSetBacklightColor(WHITE);
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
  uint8_t rflgs = eeprom_read_byte((uint8_t*)EOFS_TIMER_FLAGS);
  if (rflgs == 0xff) { // uninitialized EEPROM
    m_DelayTimerEnabled = 0x00;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_FLAGS, m_DelayTimerEnabled);
  }
  else {
    m_DelayTimerEnabled = rflgs;
  }
  uint8_t rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_START_HOUR);
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
      
      //Serial.println(m_StartTimerSeconds);
      //Serial.println(m_StopTimerSeconds);
      //Serial.println(m_CurrTimeSeconds);
      
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
  SaveSettings();
  CheckTime();
  g_OBD.Update(OBD_UPD_FORCE);
}
void DelayTimer::Disable(){
  m_DelayTimerEnabled = 0x00;
  eeprom_write_byte((uint8_t*)EOFS_TIMER_FLAGS, m_DelayTimerEnabled);
  g_EvseController.EnableAutoStart(1);
  SaveSettings();
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
#ifdef RTC
  Wire.begin();
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
