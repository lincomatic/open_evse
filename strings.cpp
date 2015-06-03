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
#include "open_evse.h"

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
#ifdef RGBLCD
char *g_BklMenuItems[] = {"RGB","Monochrome"};
#endif // RGBLCD
#endif // BTN_MENU

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

char g_sRdyLAstr[] = "L%d:%dA";
const char g_psReady[] PROGMEM = "Ready";
const char g_psCharging[] PROGMEM = "Charging";
char *g_sMaxCurrentFmt = "%s Max Current";
#endif // LCD16X2

#ifdef DELAYTIMER_MENU
char *g_YesNoMenuItems[] = {"Yes","No"};
const char g_psResetNow[] PROGMEM = "Restart Now?";
const char g_psSetDateTime[] PROGMEM = "Set Date/Time?";
char *g_DelayMenuItems[] = {"Yes/No","Set Start","Set Stop"};
#endif // DELAYTIMER_MENU
