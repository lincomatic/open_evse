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


#define OPEN_EVSE

#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <pins_arduino.h>
#include <Wire.h>
#include "./Time.h"
#include "avrstuff.h"
#if defined(ARDUINO) && (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h" // shouldn't need this but arduino sometimes messes up and puts inside an #ifdef
#endif // ARDUINO

#define VERSION "D3.7.8"

//-- begin features

// show disabled tests before POST
#define SHOW_DISABLED_TESTS

// current measurement
#define AMMETER

// serial remote api
#define RAPI

// serial port command line
// For the RTC version, only CLI or LCD can be defined at one time. 
// There is a directive to take care of that if you forget.
//#define SERIALCLI

// enable watchdog timer
#define WATCHDOG


// Support for Nick Sayer's OpenEVSE II board, which has alternate hardware for ground check/stuck relay check and a voltmeter for L1/L2.
//#define OPENEVSE_2

#ifdef OPENEVSE_2
// If the AC voltage is > 150,000 mV, then it's L2. Else, L1.
#define L2_VOLTAGE_THRESHOLD (150000)
#define VOLTMETER
// 35 ms is just a bit longer than 1.5 cycles at 50 Hz
#define VOLTMETER_POLL_INTERVAL (35)
// This is just a wild guess
// #define VOLTMETER_SCALE_FACTOR (266)     // original guess
#define DEFAULT_VOLT_SCALE_FACTOR (262)        // calibrated for Craig K OpenEVSE II build
// #define VOLTMETER_OFFSET_FACTOR (40000)  // original guess
#define DEFAULT_VOLT_OFFSET (46800)     // calibrated for Craig K OpenEVSE II build
#endif // OPENEVSE_2

// GFI support
#define GFI

// If you loop a wire from the third GFI pin through the CT a few times and then to ground,
// enable this. ADVPWR must also be defined.
#define GFI_SELFTEST

// behavior specified by UL
// 1) if enabled, POST failure will cause a hard fault until power cycled.
//    disabled, will retry POST continuously until it passes
// 2) if enabled, any a fault occurs immediately after charge is initiated, 
//    hard fault until power cycled. Otherwise, do the standard delay/retry sequence
#define UL_COMPLIANT
// if enabled, do GFI self test before closing relay
#define UL_GFI_SELFTEST

#ifdef UL_GFI_SELFTEST
#define GFI_SELFTEST
#endif //UL_GFI_SELFTEST


// Temperature monitoring support    // comment out both TEMPERATURE_MONITORING and KWH_RECORDING to have the elapsed time and time of day displayed on the second line of the LCD
#define TEMPERATURE_MONITORING

#ifdef AMMETER
// kWh Recording feature depends upon #AMMETER support
#define KWH_RECORDING
#endif //AMMETER

//Adafruit RGBLCD (MCP23017) - can have RGB or monochrome backlight
#define RGBLCD

//select default LCD backlight mode. can be overridden w/CLI/RAPI
#define BKL_TYPE_MONO 0
#define BKL_TYPE_RGB  1
#define DEFAULT_LCD_BKL_TYPE BKL_TYPE_RGB
//#define DEFAULT_LCD_BKL_TYPE BKL_TYPE_MONO

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

// valid only if ADVPWR defined - for rectified MID400 chips which block
// half cycle (for ground check on both legs)
#define SAMPLE_ACPINS
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
// REQUIRES HARDWARE RTC: DS1307 or DS3231 connected via I2C
#define RTC // enable RTC & timer functions

#ifdef RTC
// Option for Delay Timer - GoldServe
#define DELAYTIMER

// Option for AutoStart Menu. If defined, ManualStart feature is also defined by default - GoldServe
//#define AUTOSTART_MENU

#if defined(DELAYTIMER) && defined(BTN_MENU)
#define DELAYTIMER_MENU
#endif

// AutoStart feature must be defined if Delay Timers are used - GoldServe
#if defined(DELAYTIMER)||defined(AUTOSTART_MENU)
// Option for AutoStart Enable/Disable - GoldServe
#define MANUALSTART
#endif

#endif // RTC

// if defined, this pin goes HIGH when the EVSE is sleeping, and LOW otherwise
//#define SLEEP_STATUS_REG &PINB
//#define SLEEP_STATUS_IDX 4


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
#error INVALID CONFIG - CANNOT enable SERIALCLI with RTC together - too much RAM USE
#endif
#endif

#if defined(RAPI) && defined(SERIALCLI)
#error INVALID CONFIG - CANNOT DEFINE SERIALCLI AND RAPI TOGETHER SINCE THEY BOTH USE THE SERIAL PORT
#endif

#if defined(OPENEVSE_2) && !defined(ADVPWR)
#error INVALID CONFIG - OPENEVSE_2 implies/requires ADVPWR
#endif

#if defined(UL_COMPLIANT) && !defined(GFI_SELFTEST)
#error INVALID CONFIG - GFI SELF TEST NEEDED FOR UL COMPLIANCE
#endif

//
// begin functional tests
//
//
// for debugging ONLY - turns off all safety checks
//#define NOCHECKS
// DO NOT USE FT_xxx. FOR FUNCTIONAL TESTING ONLY
//
// Test for GFI fault lockout
// immediately GFI fault when entering STATE C -> should hard fault
// there will be a delay of a few seconds before the fault because we
// need to loop to cause the GFI fault
// right after GFI fault generated, will flash Closing/Relay on LCD
// -> should hard GFCI fault instantly when relay closes
//#define FT_GFI_LOCKOUT

// Test for auto reclose after GFI fault. Attach EVSIM in STATE C
// 10 sec after charging starts will induce fault. 1 minute after fault
// induced, should clear the fault and resume normal operation
//#define FT_GFI_RETRY

//
// test delay between start of sleep mode and opening relay
// connect EV and put into charging state C
// relay should open after pilot goes back to state B or higher
// or after 3 sec, whichever happens first
// LCD will display SLEEP OPEN/THRESH if opened due to EV response
//              or  SLEEP OPEN/TIMEOUT if due to timeout
//
//#define FT_SLEEP_DELAY
//
// endurance test
// alternate state B 9 sec/state C 1 sec forever
// top line displays cycle count
// bottom line displays ac pin state
//#define FT_ENDURANCE

// just read AC pins and display on line 1
//#define FT_READ_AC_PINS

//
// end functional tests
//

//-- begin configuration

// WARNING: ALL DELAYS *MUST* BE SHORTER THAN THIS TIMER OR WE WILL GET INTO
// AN INFINITE RESET LOOP
#define WATCHDOG_TIMEOUT WDTO_2S

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
#define CURRENT_PIN 0 // analog current reading pin ADCx
#define VOLT_PIN 1 // analog pilot voltage reading pin ADCx
#ifdef OPENEVSE_2
#define VOLTMETER_PIN 2 // analog AC Line voltage voltemeter pin ADCx
// This pin must match the last write to CHARGING_PIN, modulo a delay. If
// it is low when CHARGING_PIN is high, that's a missing ground.
// If it's high when CHARGING_PIN is low, that's a stuck relay.
// Auto L1/L2 is done with the voltmeter.
#define ACLINE1_REG &PIND // OpenEVSE II has only one AC test pin.
#define ACLINE1_IDX 3 

#define CHARGING_REG &PIND // OpenEVSE II has just one relay pin.
#define CHARGING_IDX 7 // OpenEVSE II has just one relay pin.
#else // !OPENEVSE_2
 // TEST PIN 1 for L1/L2, ground and stuck relay
#define ACLINE1_REG &PIND
#define ACLINE1_IDX 3
 // TEST PIN 2 for L1/L2, ground and stuck relay
#define ACLINE2_REG &PIND
#define ACLINE2_IDX 4

// digital Relay trigger pin
#define CHARGING_REG &PINB
#define CHARGING_IDX 0
// digital Relay trigger pin for second relay
#define CHARGING2_REG &PIND
#define CHARGING2_IDX 7
//digital Charging pin for AC relay
#define CHARGINGAC_REG &PINB
#define CHARGINGAC_IDX 1

// obsolete LED pin
//#define RED_LED_REG &PIND
//#define RED_LED_IDX 5
// obsolete LED pin
//#define GREEN_LED_REG &PINB
//#define GREEN_LED_IDX 5
#endif // OPENEVSE_2

// N.B. if PAFC_PWM is enabled, then pilot pin can be PB1 or PB2
// if using fast PWM (PAFC_PWM disabled) pilot pin *MUST* be PB2
#define PILOT_REG &PINB
#define PILOT_IDX 2


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

// AMMETER stuff
#define EOFS_CURRENT_SCALE_FACTOR 9 // 2 bytes
#define EOFS_AMMETER_CURR_OFFSET  11 // 2 bytes
#define EOFS_KWH_ACCUMULATED 13 // 4 bytes

// fault counters
#define EOFS_GFI_TRIP_CNT      17 // 1 byte
#define EOFS_NOGND_TRIP_CNT    18 // 1 byte
#define EOFS_STUCK_RELAY_TRIP_CNT 19 // 1 byte

#define EOFS_VOLT_OFFSET 20 // 2 bytes
#define EOFS_VOLT_SCALE_FACTOR 22 // 2 bytes


// must stay within thresh for this time in ms before switching states
#define DELAY_STATE_TRANSITION 250
// must transition to state A from contacts closed in < 100ms according to spec
// but Leaf sometimes bounces from 3->1 so we will debounce it a little anyway
#define DELAY_STATE_TRANSITION_A 25

// for ADVPWR
#define GROUND_CHK_DELAY  1000 // delay after charging started to test, ms
#define STUCK_RELAY_DELAY 1000 // delay after charging opened to test, ms
#define RelaySettlingTime  250 // time for relay to settle in post, ms

// for SAMPLE_ACPINS - max number of ms to sample
#define AC_SAMPLE_MS 20 // 1 cycle @ 60Hz = 16.6667ms @ 50Hz = 20ms

#ifdef GFI
#define GFI_INTERRUPT 0 // interrupt number 0 = PD2, 1 = PD3
// interrupt number 0 = PD2, 1 = PD3
#define GFI_REG &PIND
#define GFI_IDX 2

#ifdef GFI_SELFTEST
// pin is supposed to be wrapped around the GFI CT 5+ times
#define GFITEST_REG &PIND
#define GFITEST_IDX 6

#define GFI_TEST_CYCLES 60
#define GFI_PULSE_DURATION_US 8333 // of roughly 60 Hz. - 8333us as a half-cycle
#endif


#ifdef GFI_TESTING
#define GFI_TIMEOUT ((unsigned long)(15*1000))
#define GFI_RETRY_COUNT  255
#else // !GFI_TESTING
#define GFI_TIMEOUT ((unsigned long)(5*60000)) // 15*60*1000 doesn't work. go figure
// number of times to retry tests before giving up. 255 = retry indefinitely
#define GFI_RETRY_COUNT  6
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
#ifdef RGBLCD
#define MCP23017 // Adafruit RGB LCD (PANELOLU2 is now supported without additional define)
#else
#define MCP23008 // Adafruit I2C Backpack
#endif
#include "./LiquidTWI2.h"
#define LCD_I2C_ADDR 0x20 // for adafruit shield or backpack
#endif // I2CLCD_PCF8574
#endif // RGBLCD || I2CLCD

// button sensing pin
#define BTN_REG &PINC
#define BTN_IDX 3
#define BTN_PRESS_SHORT 50  // ms
#define BTN_PRESS_LONG 500 // ms
#define BTN_PRESS_VERYLONG 10000


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

// First, you need to select the burden resistor for the CT. You choose the largest value possible such that 
// the maximum peak-to-peak voltage for the current range is 5 volts. To obtain this value, divide the maximum 
// outlet current by the Te. That value is the maximum CT current RMS. You must convert that to P-P, so multiply 
// by 2*sqrt(2). Divide 5 by that value and select the next lower standard resistor value. For the reference 
// design, Te is 1018 and the outlet maximum is 30. 5/((30/1018)*2*sqrt(2)) = 59.995, so a 56 ohm resistor 
// is called for. Call this value Rb (burden resistor).

// Next, one must use Te and Rb to determine the volts-per-amp value. Note that the readCurrent() 
// method calculates the RMS value before the scaling factor, so RMS need not be taken into account.
// (1 / Te) * Rb = Rb / Te = Volts per Amp. For the reference design, that's 55.009 mV.
// Each count of the A/d converter is 4.882 mV (5/1024). V/A divided by V/unit is unit/A. For the reference 
// design, that's 11.26. But we want milliamps per unit, so divide that into 1000 to get 88.7625558. Round near...
//#define DEFAULT_CURRENT_SCALE_FACTOR 106 // for RB = 47 - recommended for 30A max
//#define DEFAULT_CURRENT_SCALE_FACTOR 184 // for RB = 27 - recommended for 50A max 
// Craig K, I arrived at 213 by scaling my previous multiplier of 225 down by the ratio of my panel meter reading of 28 with the OpenEVSE uncalibrated reading of 29.6
// then upped the scale factor to 220 after fixing the zero offset by subtracing 900ma
//#define DEFAULT_CURRENT_SCALE_FACTOR 220 // for RB = 22 - measured by Craig on his new OpenEVSE V3 
// NOTE: setting DEFAULT_CURRENT_SCALE_FACTOR TO 0 will disable the ammeter
// until it is overridden via RAPI
//#define DEFAULT_CURRENT_SCALE_FACTOR 220   // Craig K, average of three OpenEVSE controller calibrations
#ifdef OPENEVSE_2
#define DEFAULT_CURRENT_SCALE_FACTOR 186   // OpenEVSE II with a 27 Ohm burden resistor, after a 2-point calibration at 12.5A and 50A
#else
#define DEFAULT_CURRENT_SCALE_FACTOR 220   // OpenEVSE v2.5 and v3 with a 22 Ohm burden resistor (note that the schematic may say 28 Ohms by mistake)
#endif

// subtract this from ammeter current reading to correct zero offset
#ifdef OPENEVSE_2
#define DEFAULT_AMMETER_CURRENT_OFFSET 230 // OpenEVSE II with a 27 Ohm burden resistor, after a 2-point calibration at 12.5A and 50A
#else
#define DEFAULT_AMMETER_CURRENT_OFFSET 0   // OpenEVSE v2.5 and v3 with a 22 Ohm burden resistor.  Could use a more thorough calibration exercise to nails this down.
#endif

#ifdef KWH_RECORDING
#define VOLTS_FOR_L1 120       // conventional for North America
//  #define VOLTS_FOR_L2 230   // conventional for most of the world
#define VOLTS_FOR_L2 240       // conventional for North America
#endif // KWH_RECORDING

// The maximum number of milliseconds to sample an ammeter pin in order to find three zero-crossings.
// one and a half cycles at 50 Hz is 30 ms.
#define CURRENT_SAMPLE_INTERVAL 35

// Once we detect a zero-crossing, we should not look for one for another quarter cycle or so. 1/4 // cycle at 50 Hz is 5 ms.
#define CURRENT_ZERO_DEBOUNCE_INTERVAL 5

#endif // AMMETER

#ifdef TEMPERATURE_MONITORING

#define MCP9808_IS_ON_I2C    // Use the MCP9808 connected to I2C          
#define TMP007_IS_ON_I2C     // Use the TMP007 IR sensor on I2C 
#define TEMPERATURE_DISPLAY_ALWAYS 0     // Set this flag to 1 to always show temperatures on the bottom line of the 16X2 LCD
                                         // Set to it 0 to only display when temperatures become elevated 
// #define TESTING_TEMPERATURE_OPERATION // Set this flag to play with very low sensor thresholds or to evaluate the code.
                                         // Leave it commented out instead to run with normal temperature thresholds.

// Temperature thresholds below are expressed as 520 meaning 52.0C to save from needing floating point library
// Keep any adjustments that you make at least 2C apart, giving things some hysterisis.  (example is 580 is 3C apart from 550)
// The RESTORE_AMPERAGE value must be lower than the THROTTLE_DOWN value
// The THROTTLE_DOWN value must be lower than the SHUTDOWN value
// The SHUTDOWN value must be lower than the PANIC value
#ifndef TESTING_TEMPERATURE_OPERATION
    // normal oerational thresholds just below
#define TEMPERATURE_AMBIENT_THROTTLE_DOWN 520     // This is the temperature in the enclosure where we tell the car to draw 1/2 amperage.
#define TEMPERATURE_AMBIENT_RESTORE_AMPERAGE 490  // If the OpenEVSE responds nicely to the lower current drawn and temperatures in the enclosure
                                                  // recover to this level we can kick the current back up to the user's original amperage setting.
#define TEMPERATURE_AMBIENT_SHUTDOWN 550          // This is the temperature in the enclosure where we tell the car to draw 1/4 amperage or 6A is minimum.
                                                  
#define TEMPERATURE_AMBIENT_PANIC 580             //  At this temperature gracefully tell the EV to quit drawing any current, and leave the EVSE in 
                                                  //  an over temperature error state.  The EVSE can be restart from the button or unplugged.
                                                  //  If temperatures get to this level it is advised to open the enclosure to look for trouble.

#define TEMPERATURE_INFRARED_THROTTLE_DOWN 650    // This is the temperature seen  by the IR sensor where we tell the car to draw 1/2 amperage.
#define TEMPERATURE_INFRARED_RESTORE_AMPERAGE 600 // If the OpenEVSE responds nicely to the lower current drawn and temperatures in the enclosure
                                                  // recover to this level we can kick the current back up to the user's original amperage setting.
#define TEMPERATURE_INFRARED_SHUTDOWN 700         // This is the temperature in the enclosure where we tell the car to draw 1/4 amperage or 6A is minimum.

#define TEMPERATURE_INFRARED_PANIC 750            //  At this temperature gracefully tell the EV to quit drawing any current, and leave the EVSE in 
                                                  //  an over temperature error state.  The EVSE can be restart from the button or unplugged.
                                                  //  If temperatures get to this level it is advised to open the enclosure to look for trouble.
#else  //TESTING_TEMPERATURE_OPERATION

// Below are good values for testing purposes at room temperature with an EV simulator and no actual high current flowing
#define TEMPERATURE_AMBIENT_THROTTLE_DOWN 290     // This is the temperature in the enclosure where we tell the car to draw 1/2 amperage.
#define TEMPERATURE_AMBIENT_RESTORE_AMPERAGE 270  // If the OpenEVSE responds nicely to the lower current drawn and temperatures in the enclosure
                                                  // recover to this level we can kick the current back up to the user's original amperage setting.
#define TEMPERATURE_AMBIENT_SHUTDOWN 310          // This is the temperature in the enclosure where we tell the car to draw 1/4 amperage or 6A is minimum.

#define TEMPERATURE_AMBIENT_PANIC 330             //  At this temperature gracefully tell the EV to quit drawing any current, and leave the EVSE in 
                                                  //  an over temperature error state.  The EVSE can be restart from the button or unplugged.
                                                  //  If temperatures get to this level it is advised to open the enclosure to look for trouble.                              

#define TEMPERATURE_INFRARED_THROTTLE_DOWN 330    // This is the temperature seen  by the IR sensor where we tell the car to draw 1/2 amperage.
#define TEMPERATURE_INFRARED_RESTORE_AMPERAGE 270 // If the OpenEVSE responds nicely to the lower current drawn and temperatures in the enclosure
                                                  // recover to this level we can kick the current back up to the user's original amperage setting.
#define TEMPERATURE_INFRARED_SHUTDOWN 360         // This is the temperature in the enclosure where we tell the car to draw 1/4 amperage or 6A is minimum.

#define TEMPERATURE_INFRARED_PANIC 400            // At this temperature gracefully tell the EV to quit drawing any current, and leave the EVSE in 
                                                  // an over temperature error state.  The EVSE can be restart from the button or unplugged.
                                                  // If temperatures get to this level it is advised to open the enclosure to look for trouble.

#endif // TESTING_TEMPERATURE_OPERATION 

#endif // TEMPERATURE_MONITORING

// how long to show each disabled test on LCD
#define SHOW_DISABLED_DELAY 1500

//-- end configuration

//-- begin class definitions

#ifdef WATCHDOG
#define WDT_RESET() wdt_reset() // pat the dog
#define WDT_ENABLE() wdt_enable(WATCHDOG_TIMEOUT)
#else
#define WDT_RESET()
#define WDT_ENABLE()
#endif // WATCHDOG

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
  void println_P(const char PROGMEM *s);
  void print(char *s) { 
    Serial.print(s); 
  }
  void print_P(const char PROGMEM *s);
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
#define OBDF_UPDATE_DISABLED 0x40

// OnboardDisplay::Update()
#define OBD_UPD_NORMAL    0
#define OBD_UPD_FORCE     1 // update even if no state transition
#define OBD_UPD_HARDFAULT 2 // update w/ hard fault
class OnboardDisplay 
{
#ifdef RED_LED_REG
  DigitalPin pinRedLed;
#endif
#ifdef GREEN_LED_REG
  DigitalPin pinGreenLed;
#endif
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
  uint8_t m_strBufLen;

  int8_t updateDisabled() { return  m_bFlags & OBDF_UPDATE_DISABLED; }

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
  void LcdPrint_P(const char PROGMEM *s);
  void LcdPrint(int y,const char *s);
  void LcdPrint_P(int y,const char PROGMEM *s);
  void LcdPrint(int x,int y,const char *s);
  void LcdPrint_P(int x,int y,const char PROGMEM *s);
  void LcdPrint(int i) { 
    if (LcdDetected()) m_Lcd.print(i); 
  }
  void LcdSetCursor(int x,int y) { 
    m_Lcd.setCursor(x,y); 
  }
  void LcdClearLine(int y) {
    m_Lcd.setCursor(0,y);
    if (LcdDetected()) m_Lcd.print(g_BlankLine);
    m_Lcd.setCursor(0,y);
  }
  void LcdClear() { 
    if (LcdDetected()) m_Lcd.clear();
  }
  void LcdWrite(uint8_t data) { 
    if (LcdDetected()) m_Lcd.write(data);
  }
  void LcdMsg(const char *l1,const char *l2);
  void LcdMsg_P(const char PROGMEM *l1,const char PROGMEM *l2);
  void LcdSetBacklightType(uint8_t t,uint8_t update=OBD_UPD_FORCE) { // BKL_TYPE_XXX
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

  void DisableUpdate(int8_t on) {
    if (on) m_bFlags |= OBDF_UPDATE_DISABLED;
    else m_bFlags &= ~OBDF_UPDATE_DISABLED;
  }
  void Update(int8_t updmode=OBD_UPD_NORMAL); // OBD_UPD_xxx
};

#ifdef GFI
class Gfi {
  DigitalPin pin;
  uint8_t m_GfiFault;
#ifdef GFI_SELFTEST
  DigitalPin pinTest;
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
  uint8_t SelfTest();
  void SetTestSuccess() { testSuccess = 1; }
  uint8_t SelfTestSuccess() { return testSuccess; }
  uint8_t SelfTestInProgress() { return testInProgress; }
#endif
};
#endif // GFI

#ifdef TEMPERATURE_MONITORING
#include "./Adafruit_MCP9808.h"  //  adding the ambient temp sensor to I2C
#include "./Adafruit_TMP007.h"   //  adding the TMP007 IR I2C sensor

// TempMonitor.m_Flags
#define TMF_OVERTEMPERATURE          0x01
#define TMF_OVERTEMPERATURE_SHUTDOWN 0x02
#define TMF_BLINK_ALARM              0x04
class TempMonitor {
  uint8_t m_Flags;
public:
#ifdef MCP9808_IS_ON_I2C
  Adafruit_MCP9808 m_tempSensor;
#endif  //MCP9808_IS_ON_I2C
#ifdef TMP007_IS_ON_I2C
  Adafruit_TMP007 m_tmp007;
#endif  //TMP007_IS_ON_I2C

  uint8_t m_ampacity;  // using this to keep track of the user's original ampacity to restore after temperature monitoring has throttled it to a lower value
  // these three temperatures need to be signed integers
  int16_t m_MCP9808_temperature;  // 230 means 23.0C  Using an integer to save on floating point library use
  int16_t m_DS3231_temperature;   // the DS3231 RTC has a built in temperature sensor
  int16_t m_TMP007_temperature;

  TempMonitor() {}
  void Init();
  void Read();

  void SetBlinkAlarm(int8_t tf) {
    if (tf) m_Flags |= TMF_BLINK_ALARM;
    else m_Flags &= ~TMF_BLINK_ALARM;
  }
  int8_t BlinkAlarm() { return (m_Flags & TMF_BLINK_ALARM) ? 1 : 0; }
  void SetOverTemperature(int8_t tf) {
    if (tf) m_Flags |= TMF_OVERTEMPERATURE;
    else m_Flags &= ~TMF_OVERTEMPERATURE;
  }
  int8_t OverTemperature() { return (m_Flags & TMF_OVERTEMPERATURE) ? 1 : 0; }
  void SetOverTemperatureShutdown(int8_t tf) {
    if (tf) m_Flags |= TMF_OVERTEMPERATURE_SHUTDOWN;
    else m_Flags &= ~TMF_OVERTEMPERATURE_SHUTDOWN;
  }
  int8_t OverTemperatureShutdown() { return (m_Flags & TMF_OVERTEMPERATURE_SHUTDOWN) ? 1 : 0; }
};
#endif // TEMPERATURE_MONITORING

typedef enum {
  PILOT_STATE_P12,PILOT_STATE_PWM,PILOT_STATE_N12} 
PILOT_STATE;
class J1772Pilot {
  PILOT_STATE m_State;
#ifndef PAFC_PWM
  DigitalPin pin;
#endif // !PAFC_PWM
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
#define EVSE_STATE_OVER_TEMPERATURE 0x0A // over temperature error shutdown                    
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
#define ECVF_AUTOSVCLVL_SKIPPED 0x01 // auto svc level test skipped during post
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
  AdcPin adcPilot;
#ifdef CURRENT_PIN
  AdcPin adcCurrent;
#endif
#ifdef VOLTMETER_PIN
  AdcPin adcVoltMeter;
#endif

  DigitalPin pinCharging;
#ifdef CHARGING2_REG
  DigitalPin pinCharging2;
#endif
#ifdef CHARGINGAC_REG
  DigitalPin pinChargingAC;
#endif
#ifdef ACLINE1_REG
  DigitalPin pinAC1;
#endif
#ifdef ACLINE2_REG
  DigitalPin pinAC2;
#endif
#ifdef SLEEP_STATUS_REG
  DigitalPin pinSleepStatus;
#endif
#ifdef ADVPWR
  unsigned long m_NoGndStart;
  unsigned long m_NoGndRetryCnt;
  uint8_t m_NoGndTripCnt;
  unsigned long m_StuckRelayStartTimeMS;
  uint8_t m_StuckRelayTripCnt;
#endif // ADVPWR
  uint16_t m_wFlags; // ECF_xxx
  uint8_t m_bVFlags; // ECVF_xxx
  THRESH_DATA m_ThreshData;
  uint8_t m_EvseState;
  uint8_t m_PrevEvseState;
  uint8_t m_TmpEvseState;
  unsigned long m_TmpEvseStateStart;
  uint8_t m_CurrentCapacity; // max amps we can output
  unsigned long m_ChargeOnTimeMS; // millis() when relay last closed
  unsigned long m_ChargeOffTimeMS; // millis() when relay last opened
  time_t m_ChargeOnTime; // unixtime when relay last closed
  time_t m_ChargeOffTime;   // unixtime when relay last opened
  time_t m_ElapsedChargeTime;
  time_t m_ElapsedChargeTimePrev;

#ifdef ADVPWR
// power states for doPost() (active low)
#define both 0
#define L1on 1
#define L2on 2
#define none 3
// service states for doPost()
#define UD 0 // undefined
#define L1 1 // L1
#define L2 2 // L2
#define OG 3 // open ground
#define SR 4 // stuck relay
#define FG 5 // GFI fault

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
  unsigned long m_AmmeterReading;
  int32_t m_ChargingCurrent;
  int16_t m_AmmeterCurrentOffset;
  int16_t m_CurrentScaleFactor;

  void readAmmeter();
#endif // AMMETER
#ifdef VOLTMETER
  uint16_t m_VoltOffset;
  uint16_t m_VoltScaleFactor;
  uint32_t readVoltmeter();
#endif // VOLTMETER

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
  int SetCurrentCapacity(uint8_t amps,uint8_t updatelcd=0,uint8_t nosave=0);
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

  void SetAutoSvcLvlSkipped(uint8_t tf) {
    if (tf) m_bVFlags |= ECVF_AUTOSVCLVL_SKIPPED;
    else m_bVFlags &= ~ECVF_AUTOSVCLVL_SKIPPED;
  }
  uint8_t AutoSvcLvlSkipped() { return m_bVFlags & ECVF_AUTOSVCLVL_SKIPPED; }


  uint8_t ReadACPins();
#endif // ADVPWR

  void HardFault();

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

#ifdef VOLTMETER
  void SetVoltmeter(uint16_t scale,uint16_t offset);
  uint32_t ReadVoltmeter();
#endif // VOLTMETER
#ifdef AMMETER
  int32_t GetChargingCurrent() { return m_ChargingCurrent; }
  int16_t GetAmmeterCurrentOffset() { return m_AmmeterCurrentOffset; }
  int16_t GetCurrentScaleFactor() { return m_CurrentScaleFactor; }
  void SetAmmeterCurrentOffset(int16_t offset) {
    m_AmmeterCurrentOffset = offset;
    eeprom_write_word((uint16_t*)EOFS_AMMETER_CURR_OFFSET,offset);
  }
  void SetCurrentScaleFactor(int16_t scale) {
    m_CurrentScaleFactor = scale;
    eeprom_write_word((uint16_t*)EOFS_CURRENT_SCALE_FACTOR,scale);
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
  void ProcessInputs(uint8_t nosleeptoggle);
  void Reboot();
#ifdef SHOW_DISABLED_TESTS
  void ShowDisabledTests();
#endif
};

#ifdef BTN_MENU
#define BTN_STATE_OFF   0
#define BTN_STATE_SHORT 1 // short press
#define BTN_STATE_LONG  2 // long press
class Btn {
#ifdef BTN_REG
  DigitalPin pinBtn;
#endif
  uint8_t buttonState;
  unsigned long lastDebounceTime;  // the last time the output pin was toggled
  unsigned long vlongDebounceTime;  // for verylong press
  
public:
  Btn();
  void init();

  void read();
  uint8_t shortPress();
  uint8_t longPress();
};


class Menu {
public:
  const char PROGMEM *m_Title;
  uint8_t m_CurIdx;
  
  void init(const char *firstitem);

  Menu();

  virtual void Init() = 0;
  virtual void Next() = 0;
  virtual Menu *Select() = 0;
};

class SetupMenu : public Menu {
  uint8_t m_menuCnt;
  uint8_t m_noExit;
public:
  SetupMenu();
  void Init();
  void Next();
  Menu *Select();
  void EnableExitItem(uint8_t tf) {
    m_noExit = !tf;
  }
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

class RlyChkMenu : public Menu {
public:
  RlyChkMenu();
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
  uint8_t m_SavedLcdMode;

public:
  BtnHandler();
  void init() { m_Btn.init(); }
  void ChkBtn(int8_t nosleeptoggle);
  uint8_t InMenu() { return (m_CurMenu == NULL) ? 0 : 1; }
  uint8_t GetSavedLcdMode() { return m_SavedLcdMode; }
  void SetSavedLcdMode(uint8_t mode ) { m_SavedLcdMode = mode; }
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
  unsigned long m_LastCheck;
public:
  DelayTimer(){
    m_LastCheck = - (60ul * 1000ul);
  };
  void Init();
  void CheckTime();
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
    eeprom_write_byte((uint8_t*)EOFS_TIMER_START_HOUR, m_StartTimerHour);
    eeprom_write_byte((uint8_t*)EOFS_TIMER_START_MIN, m_StartTimerMin);
    SaveSettings();
  };
  void SetStopTimer(uint8_t hour, uint8_t min){
    m_StopTimerHour = hour;
    m_StopTimerMin = min;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_STOP_HOUR, m_StopTimerHour);
    eeprom_write_byte((uint8_t*)EOFS_TIMER_STOP_MIN, m_StopTimerMin);
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

/*
extern char g_sPlus[];
extern char g_sSlash[];
extern char g_sColon[];
extern char g_sSpace[];
*/
#define g_sPlus "+"
#define g_sSlash "/"
#define g_sColon ":"
#define g_sSpace " "


char *u2a(unsigned long x,int digits=0);

#ifdef KWH_RECORDING
extern unsigned long g_WattHours_accumulated;
extern unsigned long g_WattSeconds;
#endif // KWH_RECORDING
#ifdef TEMPERATURE_MONITORING
extern TempMonitor g_TempMonitor;
#endif // TEMPERATURE_MONITORING



#include "rapi_proc.h"
