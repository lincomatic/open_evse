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
const char g_psSettings[] PROGMEM = STR_SETTINGS;
const char g_psSetup[] PROGMEM = STR_SETUP;
const char g_psSvcLevel[] PROGMEM = STR_SERVICE_LEVEL;
const char g_psMaxCurrent[] PROGMEM = STR_MAX_CURRENT;
const char g_psDiodeCheck[] PROGMEM = STR_DIODE_CHECK;
const char g_psVentReqChk[] PROGMEM = STR_VENT_REQD_CHECK;
#ifdef RGBLCD
const char g_psBklType[] PROGMEM = STR_BACKLIGHT_TYPE;
#endif
#ifdef ADVPWR
const char g_psGndChk[] PROGMEM = STR_GROUND_CHECK;
const char g_psRlyChk[] PROGMEM = STR_STUCK_RELAY_CHK;
#endif // ADVPWR
#ifdef GFI_SELFTEST
const char g_psGfiTest[] PROGMEM = STR_GFI_SELF_TEST;
#endif
#ifdef TEMPERATURE_MONITORING
const char g_psTempChk[] PROGMEM = STR_TEMPERATURE_CHK;
const char g_psHighTemp[] PROGMEM = STR_HIGH_TEMP;
#endif
#endif // BTN_MENU || SHOW_DISABLED_TEST

#ifdef BTN_MENU
const char *g_YesNoMenuItems[] = STR_YES_NO;
const char g_psResetNow[] PROGMEM = STR_RESTART_NOW;
const char g_psReset[] PROGMEM = STR_RESTART;
const char g_psExit[] PROGMEM = STR_EXIT;
// Add additional strings - GoldServe
#ifdef DELAYTIMER_MENU
const char g_psRTC[] PROGMEM = STR_DATE_TIME;
const char g_psRTC_Month[] PROGMEM = STR_MONTH;
const char g_psRTC_Day[] PROGMEM = STR_DAY;
const char g_psRTC_Year[] PROGMEM = STR_YEAR;
const char g_psRTC_Hour[] PROGMEM = STR_HOUR;
const char g_psRTC_Minute[] PROGMEM = STR_MINUTE;
const char g_psDelayTimer[] PROGMEM = STR_DELAY_TIMER;
const char g_psDelayTimerStartHour[] PROGMEM = STR_START_HOUR;
const char g_psDelayTimerStartMin[] PROGMEM = STR_START_MIN;
const char g_psDelayTimerStopHour[] PROGMEM = STR_STOP_HOUR;
const char g_psDelayTimerStopMin[] PROGMEM = STR_STOP_MIN;
#endif // DELAYTIMER_MENU
#ifdef CHARGE_LIMIT
const char g_psChargeLimit[] PROGMEM = STR_CHARGE_LIMIT;
#endif // CHARGE_LIMIT
#ifdef TIME_LIMIT
const char g_psTimeLimit[] PROGMEM = STR_TIME_LIMIT;
#endif // TIME_LIMIT
#ifdef RGBLCD
const char *g_BklMenuItems[] = STR_RGB_MONOCHROME;
#endif // RGBLCD
#endif // BTN_MENU

#ifdef LCD16X2
#ifdef ADVPWR
const char g_psPwrOn[] PROGMEM = STR_POWER_ON;
const char g_psSelfTest[] PROGMEM = STR_SELF_TEST;
const char g_psAutoDetect[] PROGMEM = STR_AUTO_DETECT;
const char g_psLevel1[] PROGMEM = STR_SVC_LEVEL_L1;
const char g_psLevel2[] PROGMEM = STR_SVC_LEVEL_L2;
const char g_psTestFailed[] PROGMEM = STR_TEST_FAILED;
#endif // ADVPWR
const char g_psEvseError[] PROGMEM =  STR_EVSE_ERROR;
const char g_psSvcReq[] PROGMEM =  STR_SERVICE_REQUIRED;
const char g_psVentReq[] PROGMEM = STR_VENT_REQUIRED;
const char g_psDiodeChkFailed[] PROGMEM = STR_DIODE_CHECK_FAILED;
const char g_psGfciFault[] PROGMEM = STR_GFCI_FAULT;
const char g_psGfci[] PROGMEM = STR_GFCI;
const char g_sRetryIn[] = STR_RETRY_IN;

#ifdef TEMPERATURE_MONITORING
const char g_psTemperatureFault[] PROGMEM = STR_OVER_TEMPERATURE;
#endif
const char g_psNoGround[] PROGMEM = STR_NO_GROUND;
const char g_psStuckRelay[] PROGMEM = STR_STUCK_RELAY;
const char g_psDisabled[] PROGMEM =  STR_DISABLED;
const char g_psSleeping[] PROGMEM = STR_SLEEPING;
const char g_psEvConnected[] PROGMEM = STR_CONNECTED;
const char g_psResetting[] PROGMEM = STR_RESETTING;

#ifdef SHOW_DISABLED_TESTS
const char g_psDisabledTests[] PROGMEM = STR_TEST_DISABLED;
#endif

const char g_sRdyLAstr[] = STRF_L_A;
const char g_psReady[] PROGMEM = STR_READY;
const char g_psCharging[] PROGMEM = STR_CHARGING;
const char *g_sMaxCurrentFmt = STRF_MAX_CURRENT;
#endif // LCD16X2

#ifdef DELAYTIMER_MENU
const char g_psSetDateTime[] PROGMEM = STR_SET_DATE_TIME;
const char *g_DelayMenuItems[] = STR_YESNO_SETSTART_SETSTOP;
#endif // DELAYTIMER_MENU

#ifdef OVERCURRENT_THRESHOLD
const char g_psOverCurrent[] PROGMEM = STR_OVER_CURRENT;
#endif // OVERCURRENT_THRESHOLD
