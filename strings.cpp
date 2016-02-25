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
const char g_psSettings[] PROGMEM = STRING001;
const char g_psSetup[] PROGMEM = STRING002;
const char g_psSvcLevel[] PROGMEM = STRING003;
const char g_psMaxCurrent[] PROGMEM = STRING004;
const char g_psDiodeCheck[] PROGMEM = STRING005;
const char g_psVentReqChk[] PROGMEM = STRING006;
#ifdef RGBLCD
const char g_psBklType[] PROGMEM = STRING007;
#endif
#ifdef ADVPWR
const char g_psGndChk[] PROGMEM = STRING008;
const char g_psRlyChk[] PROGMEM = STRING009;
#endif // ADVPWR
#ifdef GFI_SELFTEST
const char g_psGfiTest[] PROGMEM = STRING010;
#endif
#ifdef TEMPERATURE_MONITORING
const char g_psTempChk[] PROGMEM = STRING011;
#endif
#endif // BTN_MENU || SHOW_DISABLED_TEST

#ifdef BTN_MENU
const char *g_YesNoMenuItems[] = STRING012;
const char g_psResetNow[] PROGMEM = STRING013;
const char g_psReset[] PROGMEM = STRING014;
const char g_psExit[] PROGMEM = STRING015;
// Add additional strings - GoldServe
#ifdef DELAYTIMER_MENU
const char g_psRTC[] PROGMEM = STRING016;
const char g_psRTC_Month[] PROGMEM = STRING017;
const char g_psRTC_Day[] PROGMEM = STRING018;
const char g_psRTC_Year[] PROGMEM = STRING019;
const char g_psRTC_Hour[] PROGMEM = STRING020;
const char g_psRTC_Minute[] PROGMEM = STRING021;
const char g_psDelayTimer[] PROGMEM = STRING022;
const char g_psDelayTimerStartHour[] PROGMEM = STRING023;
const char g_psDelayTimerStartMin[] PROGMEM = STRING024;
const char g_psDelayTimerStopHour[] PROGMEM = STRING025;
const char g_psDelayTimerStopMin[] PROGMEM = STRING026;
#endif // DELAYTIMER_MENU
#ifdef CHARGE_LIMIT
const char g_psChargeLimit[] PROGMEM = STRING027;
#endif // CHARGE_LIMIT
#ifdef TIME_LIMIT
const char g_psTimeLimit[] PROGMEM = STRING028;
#endif // TIME_LIMIT
#ifdef RGBLCD
const char *g_BklMenuItems[] = STRING029;
#endif // RGBLCD
#endif // BTN_MENU

#ifdef LCD16X2
#ifdef ADVPWR
const char g_psPwrOn[] PROGMEM = STRING030;
const char g_psSelfTest[] PROGMEM = STRING031;
const char g_psAutoDetect[] PROGMEM = STRING032;
const char g_psLevel1[] PROGMEM = STRING033;
const char g_psLevel2[] PROGMEM = STRING034;
const char g_psTestFailed[] PROGMEM = STRING035;
#endif // ADVPWR
const char g_psEvseError[] PROGMEM =  STRING036;
const char g_psSvcReq[] PROGMEM =  STRING037;
const char g_psVentReq[] PROGMEM = STRING038;
const char g_psDiodeChkFailed[] PROGMEM = STRING039;
const char g_psGfciFault[] PROGMEM = STRING040;
const char g_psGfci[] PROGMEM = STRING041;
const char g_sRetryIn[] = STRING042;

#ifdef TEMPERATURE_MONITORING
const char g_psTemperatureFault[] PROGMEM = STRING043;
#endif
const char g_psNoGround[] PROGMEM = STRING044;
const char g_psStuckRelay[] PROGMEM = STRING045;
const char g_psDisabled[] PROGMEM =  STRING046;
//const char g_psWaiting[] PROGMEM =  STRING047;
const char g_psSleeping[] PROGMEM = STRING048;
const char g_psEvConnected[] PROGMEM = STRING049;
#ifdef SHOW_DISABLED_TESTS
const char g_psDisabledTests[] PROGMEM = STRING050;
#endif

const char g_sRdyLAstr[] = STRING051;
const char g_psReady[] PROGMEM = STRING052;
const char g_psCharging[] PROGMEM = STRING053;
const char *g_sMaxCurrentFmt = STRING054;
#endif // LCD16X2

#ifdef DELAYTIMER_MENU
const char g_psSetDateTime[] PROGMEM = STRING055;
const char *g_DelayMenuItems[] = STRING056;
#endif // DELAYTIMER_MENU
