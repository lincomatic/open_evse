#pragma once
/*
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


// EVSE states for m_EvseState
#define EVSE_STATE_UNKNOWN 0x00
#define EVSE_STATE_A       0x01 // vehicle state A 12V - not connected
#define EVSE_STATE_B       0x02 // vehicle state B 9V - connected, ready
#define EVSE_STATE_C       0x03 // vehicle state C 6V - charging
#define EVSE_STATE_D       0x04 // vehicle state D 3V - vent required
#define EVSE_FAULT_STATE_BEGIN EVSE_STATE_D
#define EVSE_STATE_DIODE_CHK_FAILED 0x05 // diode check failed
#define EVSE_STATE_GFCI_FAULT 0x06       // GFCI fault
#define EVSE_STATE_NO_GROUND 0x07 //bad ground
#define EVSE_STATE_STUCK_RELAY 0x08 //stuck relay
#define EVSE_STATE_GFI_TEST_FAILED 0x09 // GFI self-test failure
#define EVSE_STATE_OVER_TEMPERATURE 0x0A // over temperature error shutdown
#define EVSE_STATE_OVER_CURRENT 0x0B // over current error shutdown
#define EVSE_FAULT_STATE_END EVSE_STATE_OVER_CURRENT
           
#define EVSE_STATE_SLEEPING 0xfe // waiting for timer
#define EVSE_STATE_DISABLED 0xff // disabled

inline int8_t IsEvseFaultState(uint8_t state) {
  if ((state >= EVSE_FAULT_STATE_BEGIN) && (state <= EVSE_FAULT_STATE_END)) return 1;
  else return 0;
}

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

// called whenever EVSE wants to transition state from curEvseState to newEvseState
// return 0 to allow transition to newEvseState
// else return desired state
typedef uint8_t (*EvseStateTransitionReqFunc)(uint8_t prevPilotState,uint8_t curPilotState,uint8_t curEvseState,uint8_t newEvseState);

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
#define ECF_TEMP_CHK_DISABLED  0x0400 // no Temperature Monitoring
#define ECF_DEFAULT            0x0000

// J1772EVSEController volatile m_bVFlags bits - not saved to EEPROM
#define ECVF_AUTOSVCLVL_SKIPPED 0x01 // auto svc level test skipped during post
#define ECVF_HARD_FAULT         0x02 // in non-autoresettable fault
#define ECVF_LIMIT_SLEEP        0x04 // currently sleeping after reaching time/charge limit
#define ECVF_AUTH_LOCKED        0x08 // locked pending authentication
#define ECVF_AMMETER_CAL        0x10 // ammeter calibration mode
#define ECVF_NOGND_TRIPPED      0x20 // no ground has tripped at least once
#define ECVF_CHARGING_ON        0x40 // charging relay is closed
#define ECVF_GFI_TRIPPED        0x80 // gfi has tripped at least once

#ifdef AUTH_LOCK
#define ECVF_DEFAULT            ECVF_AUTH_LOCKED
#else
#define ECVF_DEFAULT            0x00
#endif

// J1772EVSEController volatile m_bVFlags2 bits - not saved to EEPROM
#define ECVF2_EV_CONNECTED      0x01 // EV connected - valid only when pilot not N12
#define ECVF2_EV_CONNECTED_PREV 0x02 // prev EV connected flag
#define ECVF2_SESSION_ENDED     0x04 // used for charging session time calc

#define ECVF2_DEFAULT           ECVF2_SESSION_ENDED


class J1772EVSEController {
  J1772Pilot m_Pilot;
#ifdef GFI
  Gfi m_Gfi;
  unsigned long m_GfiFaultStartMs;
  unsigned long m_GfiRetryCnt;
  uint8_t m_GfiTripCnt; // contains tripcnt-1
#endif // GFI
  AdcPin adcPilot;
#ifdef CURRENT_PIN
  AdcPin adcCurrent;
#endif
#ifdef VOLTMETER_PIN
  AdcPin adcVoltMeter;
#endif

#ifdef CHARGING_REG
  DigitalPin pinCharging;
#endif
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
#ifdef AUTH_LOCK_REG
  DigitalPin pinAuthLock;
#endif
#ifdef ADVPWR
  unsigned long m_NoGndStart;
  unsigned long m_NoGndRetryCnt;
  uint8_t m_NoGndTripCnt; // contains tripcnt-1
  unsigned long m_StuckRelayStartTimeMS;
  uint8_t m_StuckRelayTripCnt; // contains tripcnt-1
#endif // ADVPWR
#ifdef RELAY_AUTO_PWM_PIN_TESTING
  unsigned long m_relayCloseMs; // #ms for DC pulse to close relay
  uint8_t m_relayHoldPwm; // PWM duty cycle to hold relay closed
#endif // RELAY_AUTO_PWM_PIN_TESTING
  uint16_t m_wFlags; // ECF_xxx
  uint8_t m_bVFlags; // ECVF_xxx
  uint8_t m_bVFlags2; // ECVF2_xxx
  THRESH_DATA m_ThreshData;
  uint8_t m_EvseState;
  uint8_t m_PrevEvseState;
  uint8_t m_TmpEvseState;
  uint8_t m_TmpPilotState;
  uint8_t m_PilotState;
  unsigned long m_TmpEvseStateStart;
  unsigned long m_TmpPilotStateStart;
  uint8_t m_CurrentCapacity; // max amps we can output
  unsigned long m_ChargeOnTimeMS; // millis() when relay last closed
  unsigned long m_ChargeOffTimeMS; // millis() when relay last opened
  time_t m_ChargeOnTime; // unixtime when relay last closed
  time_t m_ChargeOffTime;   // unixtime when relay last opened
  time_t m_ElapsedChargeTime;
  time_t m_ElapsedChargeTimePrev;
  time_t m_AccumulatedChargeTime;
#ifdef STATE_TRANSITION_REQ_FUNC
  EvseStateTransitionReqFunc m_StateTransitionReqFunc;
#endif // STATE_TRANSITION_REQ_FUNC
#ifdef MENNEKES_LOCK
  MennekesLock m_MennekesLock;
#endif // MENNEKES_LOCK
#ifdef OVERCURRENT_THRESHOLD
  unsigned long m_OverCurrentStartMs;
#endif // OVERCURRENT_THRESHOLD

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
  void setVFlags(uint8_t flags) { 
    m_bVFlags |= flags; 
  }
  void clrVFlags(uint8_t flags) { 
    m_bVFlags &= ~flags; 
  }
  void setVFlags2(uint8_t flags) { 
    m_bVFlags2 |= flags; 
  }
  void clrVFlags2(uint8_t flags) { 
    m_bVFlags2 &= ~flags; 
  }

#ifdef TIME_LIMIT
  uint8_t m_timeLimit15; // increments of 15min to extend charge time
  time_t m_timeLimitEnd; // end time
#endif

#ifdef AMMETER
  unsigned long m_AmmeterReading;
  int32_t m_ChargingCurrent;
  int16_t m_AmmeterCurrentOffset;
  int16_t m_CurrentScaleFactor;
#ifdef CHARGE_LIMIT
  uint8_t m_chargeLimitkWh; // kWh to extend session
  uint32_t m_chargeLimitTotWs; // total Ws limit
#endif

  void readAmmeter();
#endif // AMMETER
#ifdef VOLTMETER
  uint16_t m_VoltScaleFactor;
  uint32_t m_VoltOffset;
  uint32_t m_Voltage; // mV
#endif // VOLTMETER

public:
  J1772EVSEController();
  void Init();
  void Update(uint8_t forcetransition=0); // read sensors
  void Enable();
  void Disable(); // panic stop - open relays abruptly
  void Sleep(); // graceful stop - e.g. waiting for timer to fire- give the EV time to stop charging first
  void LoadThresholds();

  uint16_t GetFlags() { return m_wFlags; }
  uint8_t GetVFlags() { return m_bVFlags; }
  uint8_t GetVFlags2() { return m_bVFlags2; }
  uint8_t GetState() { 
    return m_EvseState; 
  }
  uint8_t GetPrevState() { 
    return m_PrevEvseState; 
  }
  int StateTransition() { 
    return (m_EvseState != m_PrevEvseState) ? 1 : 0; 
  }

  void SaveEvseFlags() {
    eeprom_write_word((uint16_t *)EOFS_FLAGS,m_wFlags);
  }

  int8_t InFaultState() {
    return ((m_EvseState >= EVSE_FAULT_STATE_BEGIN) && (m_EvseState <= EVSE_FAULT_STATE_END));
  }

#ifdef RELAY_AUTO_PWM_PIN_TESTING
  void setPwmPinParms(uint32_t delayms,uint8_t pwm) {
    m_relayCloseMs = delayms;
    m_relayHoldPwm = pwm;
  }
#endif

  void SetHardFault() { m_bVFlags |= ECVF_HARD_FAULT; }
  void ClrHardFault() { m_bVFlags &= ~ECVF_HARD_FAULT; }
  int8_t InHardFault() { return (m_bVFlags & ECVF_HARD_FAULT) ? 1 : 0; }
  unsigned long GetResetMs();

  uint8_t GetCurrentCapacity() { 
    return m_CurrentCapacity; 
  }
  uint8_t GetMaxCurrentCapacity();
  int SetCurrentCapacity(uint8_t amps,uint8_t updatelcd=0,uint8_t nosave=0);

  time_t GetElapsedChargeTime() { 
    return m_ElapsedChargeTime+m_AccumulatedChargeTime; 
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
  void SaveSettings();
#ifdef ADVPWR
  uint8_t GndChkEnabled() { 
    return (m_wFlags & ECF_GND_CHK_DISABLED) ? 0 : 1;
  }
  void EnableGndChk(uint8_t tf);
  void EnableStuckRelayChk(uint8_t tf);
  uint8_t StuckRelayChkEnabled() { 
    return (m_wFlags & ECF_STUCK_RELAY_CHK_DISABLED) ? 0 : 1;
  }
#ifdef AUTOSVCLEVEL
  uint8_t AutoSvcLevelEnabled() { return (m_wFlags & ECF_AUTO_SVC_LEVEL_DISABLED) ? 0 : 1; }
  void EnableAutoSvcLevel(uint8_t tf);
  void SetAutoSvcLvlSkipped(uint8_t tf) {
    if (tf) m_bVFlags |= ECVF_AUTOSVCLVL_SKIPPED;
    else m_bVFlags &= ~ECVF_AUTOSVCLVL_SKIPPED;
  }
  uint8_t AutoSvcLvlSkipped() { return m_bVFlags & ECVF_AUTOSVCLVL_SKIPPED; }
#else
  uint8_t AutoSvcLevelEnabled() { return 0; }
  void EnableAutoSvcLevel(uint8_t tf) {}
  void SetAutoSvcLvlSkipped(uint8_t tf) {}
  uint8_t AutoSvcLvlSkipped() { return 1; }
#endif // AUTOSVCLEVEL
  void SetNoGndTripped();
  uint8_t NoGndTripped() { return m_bVFlags & ECVF_NOGND_TRIPPED; }



  uint8_t ReadACPins();
#endif // ADVPWR

  void HardFault();

  void SetLimitSleep(uint8_t tf) {
    if (tf) m_bVFlags |= ECVF_LIMIT_SLEEP;
    else m_bVFlags &= ~ECVF_LIMIT_SLEEP;
  }
  uint8_t LimitSleepIsSet() { return m_bVFlags & ECVF_LIMIT_SLEEP; }

#ifdef GFI
  void SetGfiTripped();
  uint8_t GfiTripped() { return m_bVFlags & ECVF_GFI_TRIPPED; }
  uint8_t GetGfiTripCnt() { return m_GfiTripCnt+1; }
#ifdef GFI_SELFTEST
  uint8_t GfiSelfTestEnabled() {
    return (m_wFlags & ECF_GFI_TEST_DISABLED) ? 0 : 1;
  }
  void EnableGfiSelfTest(uint8_t tf);
#else // !GFI_SELFTEST
  uint8_t GfiSelfTestEnabled() { return 0; }
  void EnableGfiSelfTest(uint8_t tf) {}
#endif // GFI_SELFTEST
#endif // GFI

#ifdef TEMPERATURE_MONITORING  
    uint8_t TempChkEnabled() {
    return (m_wFlags & ECF_TEMP_CHK_DISABLED) ? 0 : 1;
  }
  void EnableTempChk(uint8_t tf);
#endif //TEMPERATURE_MONITORING

  uint8_t SerDbgEnabled() { 
    return (m_wFlags & ECF_SERIAL_DBG) ? 1 : 0;
  }
  void EnableSerDbg(uint8_t tf);
#ifdef RGBLCD
  int SetBacklightType(uint8_t t,uint8_t update=1); // BKL_TYPE_XXX
#endif // RGBLCD

#ifdef VOLTMETER
  uint16_t GetVoltScaleFactor() { return m_VoltScaleFactor; }
  uint32_t GetVoltOffset() { return m_VoltOffset; }
  void SetVoltmeter(uint16_t scale,uint32_t offset);
  uint32_t ReadVoltmeter();
  int32_t GetVoltage() { return m_Voltage; }
#else
  uint32_t GetVoltage() { return (uint32_t)-1; }
#endif // VOLTMETER
#ifdef AMMETER
  int32_t GetChargingCurrent() { return m_ChargingCurrent; }
#ifdef FAKE_CHARGING_CURRENT
  void SetChargingCurrent(int32_t current) {
    m_ChargingCurrent = current;
    m_AmmeterReading = current;
  }
#endif

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
  void ZeroChargingCurrent() { m_ChargingCurrent = 0; }
  uint8_t GetInstantaneousChargingAmps() {
    readAmmeter();
    return m_AmmeterReading / 1000;
  }
#ifdef CHARGE_LIMIT
  void ClrChargeLimit() { m_chargeLimitTotWs = 0; m_chargeLimitkWh = 0; }
  void SetChargeLimitkWh(uint8_t kwh);
  uint32_t GetChargeLimitTotWs() { return m_chargeLimitTotWs; }
  uint8_t GetChargeLimitkWh() { return m_chargeLimitkWh; }
#endif // CHARGE_LIMIT
#else // !AMMETER
  int32_t GetChargingCurrent() { return -1; }
#endif // AMMETER
  uint8_t LimitsAllowed() {
    return ((GetState() == EVSE_STATE_B) || (GetState() == EVSE_STATE_C)) ? 1 : 0;
  }
#ifdef TIME_LIMIT
  void ClrTimeLimit() { m_timeLimitEnd = 0; m_timeLimit15 = 0; }
  void SetTimeLimitEnd(time_t limit) { m_timeLimitEnd = limit; }
  void SetTimeLimit15(uint8_t mind15);
  uint8_t GetTimeLimit15() { return m_timeLimit15; }
  time_t GetTimeLimitEnd() { return m_timeLimitEnd; }
#endif // TIME_LIMIT
  void ReadPilot(uint16_t *plow=NULL,uint16_t *phigh=NULL);
  void Reboot();
#ifdef SHOW_DISABLED_TESTS
  void DisabledTest_P(PGM_P message);
  void ShowDisabledTests();
#endif
#ifdef ADVPWR
  uint8_t GetNoGndTripCnt() { return m_NoGndTripCnt+1; }
  uint8_t GetStuckRelayTripCnt() { return m_StuckRelayTripCnt+1; }
#endif // ADVPWR

#ifdef STATE_TRANSITION_REQ_FUNC
  void SetStateTransitionReqFunc(EvseStateTransitionReqFunc statetransitionreqfunc) {
    m_StateTransitionReqFunc = statetransitionreqfunc;
  }
#endif
  J1772Pilot *GetPilot() { return &m_Pilot; }
  uint8_t GetPilotState() { return m_PilotState; }
  void CloseRelay() {
    chargingOn();
  }
  void OpenRelay() {
    chargingOff();
  }
  uint8_t RelayIsClosed() { return m_bVFlags & ECVF_CHARGING_ON; }
#ifdef AUTH_LOCK
  void AuthLock(uint8_t tf);
  uint8_t AuthLockIsOn() { return m_bVFlags & ECVF_AUTH_LOCKED; }
#endif // AUTH_LOCK

  void SetEvConnected() { setVFlags2(ECVF2_EV_CONNECTED); }
  void ClrEvConnected() {
    clrVFlags2(ECVF2_EV_CONNECTED);
    setVFlags2(ECVF2_SESSION_ENDED);
 }
  void SetEvConnectedPrev() { setVFlags2(ECVF2_EV_CONNECTED_PREV); }
  void ClrEvConnectedPrev() { clrVFlags2(ECVF2_EV_CONNECTED_PREV); }
  // EvConnected value valid when pilot state not N12
  uint8_t EvConnected() { return m_bVFlags2 & ECVF2_EV_CONNECTED; }
  uint8_t EvConnectedTransition() {
    if (((m_bVFlags2 & (ECVF2_EV_CONNECTED|ECVF2_EV_CONNECTED_PREV)) == 0) ||
	((m_bVFlags2 & (ECVF2_EV_CONNECTED|ECVF2_EV_CONNECTED_PREV)) == (ECVF2_EV_CONNECTED|ECVF2_EV_CONNECTED_PREV))) return 0;
    else return 1;
  }
};

#ifdef FT_ENDURANCE
extern int g_CycleCnt;
extern long g_CycleHalfStart;
extern uint8_t g_CycleState;
#endif 

extern J1772EVSEController g_EvseController;
