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

#pragma once

extern const char VERSTR[] PROGMEM;
inline void GetVerStr(char *buf) { strcpy_P(buf,VERSTR); }


#if defined(BTN_MENU) || defined(SHOW_DISABLED_TESTS)
extern const char g_psSettings[] PROGMEM;
extern const char g_psSetup[] PROGMEM;
extern const char g_psSvcLevel[] PROGMEM;
extern const char g_psMaxCurrent[] PROGMEM;
extern const char g_psDiodeCheck[] PROGMEM;
extern const char g_psVentReqChk[] PROGMEM;
#ifdef RGBLCD
extern const char g_psBklType[] PROGMEM;
#endif
#ifdef ADVPWR
extern const char g_psGndChk[] PROGMEM;
extern const char g_psRlyChk[] PROGMEM;
#endif // ADVPWR
#ifdef GFI_SELFTEST
extern const char g_psGfiTest[] PROGMEM;
#endif
#ifdef TEMPERATURE_MONITORING
extern const char g_psTempChk[] PROGMEM;
extern const char g_psHighTemp[] PROGMEM;
#endif // TEMPERATURE_MONITORING
#endif // BTN_MENU || SHOW_DISABLED_TEST

#ifdef BTN_MENU
extern const char *g_YesNoMenuItems[];
extern const char g_psResetNow[] PROGMEM;
extern const char g_psReset[] PROGMEM;
extern const char g_psExit[] PROGMEM;
// Add additional strings - GoldServe
#ifdef DELAYTIMER_MENU
extern const char g_psRTC[] PROGMEM;
extern const char g_psRTC_Month[] PROGMEM;
extern const char g_psRTC_Day[] PROGMEM;
extern const char g_psRTC_Year[] PROGMEM;
extern const char g_psRTC_Hour[] PROGMEM;
extern const char g_psRTC_Minute[] PROGMEM;
extern const char g_psDelayTimer[] PROGMEM;
extern const char g_psDelayTimerStartHour[] PROGMEM;
extern const char g_psDelayTimerStartMin[] PROGMEM;
extern const char g_psDelayTimerStopHour[] PROGMEM;
extern const char g_psDelayTimerStopMin[] PROGMEM;
#endif // DELAYTIMER_MENU
#ifdef CHARGE_LIMIT
extern const char g_psChargeLimit[] PROGMEM;
#endif // CHARGE_LIMIT
#ifdef TIME_LIMIT
extern const char g_psTimeLimit[] PROGMEM;
#endif // TIME_LIMIT
#ifdef RGBLCD
extern const char *g_BklMenuItems[];
#endif // RGBLCD
#endif // BTN_MENU

#ifdef LCD16X2
#ifdef ADVPWR
extern const char g_psPwrOn[] PROGMEM;
extern const char g_psSelfTest[] PROGMEM;
extern const char g_psAutoDetect[] PROGMEM;
extern const char g_psLevel1[] PROGMEM;
extern const char g_psLevel2[] PROGMEM;
extern const char g_psTestFailed[] PROGMEM;
#endif // ADVPWR
extern const char g_psEvseError[] PROGMEM;
extern const char g_psSvcReq[] PROGMEM;
extern const char g_psVentReq[] PROGMEM;
extern const char g_psDiodeChkFailed[] PROGMEM;
extern const char g_psGfciFault[] PROGMEM;
extern const char g_psGfci[] PROGMEM;
extern const char g_sRetryIn[];

#ifdef TEMPERATURE_MONITORING
extern const char g_psTemperatureFault[] PROGMEM;
extern const char g_psTempOnOff[] PROGMEM;
#endif
extern const char g_psNoGround[] PROGMEM;
extern const char g_psStuckRelay[] PROGMEM;
extern const char g_psDisabled[] PROGMEM;
extern const char g_psResetting[] PROGMEM;
//extern const char g_psWaiting[] PROGMEM;
extern const char g_psSleeping[] PROGMEM;
extern const char g_psEvConnected[] PROGMEM;
#ifdef SHOW_DISABLED_TESTS
extern const char g_psDisabledTests[] PROGMEM;
#endif
extern const char g_sRdyLAstr[];
extern const char g_psReady[] PROGMEM;
extern const char g_psCharging[] PROGMEM;
extern const char *g_sMaxCurrentFmt;
#endif // LCD16X2

#ifdef DELAYTIMER_MENU
extern const char g_psSetDateTime[] PROGMEM;
extern const char *g_DelayMenuItems[];
#endif // DELAYTIMER_MENU

#ifdef OVERCURRENT_THRESHOLD
extern const char g_psOverCurrent[] PROGMEM;
#endif // OVERCURRENT_THRESHOLD
