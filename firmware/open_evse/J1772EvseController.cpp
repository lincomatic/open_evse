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
#include "open_evse.h"

#ifdef FT_ENDURANCE
int g_CycleCnt = -1;
long g_CycleHalfStart;
uint8_t g_CycleState;
#endif 

//                                 A/B B/C C/D D DS
THRESH_DATA g_DefaultThreshData = {875,780,690,0,260};

J1772EVSEController g_EvseController;

#ifdef AMMETER
static inline unsigned long ulong_sqrt(unsigned long in)
{
  unsigned long out = 0;
  unsigned long bit = 0x40000000ul;

  // "bit" starts at the highest power of four <= the argument.
  while (bit > in)
    bit >>= 2;

  while (bit) {
    unsigned long sum = out + bit;
    if (in >= sum) {
      in -= sum;
      out = (out >> 1) + bit;
    }
    else
      out >>= 1;
    bit >>= 2;
  }

  return out;
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

J1772EVSEController::J1772EVSEController() :
  adcPilot(PILOT_PIN)
#ifdef CURRENT_PIN
  , adcCurrent(CURRENT_PIN)
#endif
#ifdef VOLTMETER_PIN
  , adcVoltMeter(VOLTMETER_PIN)
#endif
{
#ifdef STATE_TRANSITION_REQ_FUNC
  m_StateTransitionReqFunc = NULL;
#endif // STATE_TRANSITION_REQ_FUNC
}

void J1772EVSEController::SaveSettings()
{
  uint8_t *dest;
  // n.b. should we add dirty bits so we only write the changed values? or should we just write them on the fly when necessary?
  if (GetCurSvcLevel() == 1) {
    dest = (uint8_t *)EOFS_CURRENT_CAPACITY_L1;
  }
  else {
    dest = (uint8_t *)EOFS_CURRENT_CAPACITY_L2;
  }
  eeprom_write_byte(dest, GetCurrentCapacity());
  SaveEvseFlags();
}


#ifdef AUTH_LOCK
void J1772EVSEController::AuthLock(uint8_t tf)
{
  if (tf) setVFlags(ECVF_AUTH_LOCKED);
  else clrVFlags(ECVF_AUTH_LOCKED);
  Update(1);
  g_OBD.Update(OBD_UPD_FORCE);
}
#endif // AUTH_LOCK


// use watchdog to perform a reset
void J1772EVSEController::Reboot()
{
  m_Pilot.SetState(PILOT_STATE_P12);

#ifdef LCD16X2
  g_OBD.LcdPrint_P(1,g_psResetting);
#endif

  if (chargingIsOn()) {
   // give the EV some time to open its contactor in response to P12
    wdt_delay(3000);
  }

  // hardware reset by forcing watchdog to timeout
  wdt_enable(WDTO_1S);   // enable watchdog timer
  delay(1500);
}



#ifdef SHOW_DISABLED_TESTS
void J1772EVSEController::DisabledTest_P(PGM_P message)
{
#ifdef LCD16X2
  g_OBD.LcdMsg_P(g_psDisabledTests, message);
#endif
#ifndef NOCHECKS
  wdt_delay(SHOW_DISABLED_DELAY);
#endif
}

void J1772EVSEController::ShowDisabledTests()
{
  if (m_wFlags & (ECF_DIODE_CHK_DISABLED|
		  ECF_VENT_REQ_DISABLED|
		  ECF_GND_CHK_DISABLED|
		  ECF_STUCK_RELAY_CHK_DISABLED|
		  ECF_GFI_TEST_DISABLED|
                  ECF_TEMP_CHK_DISABLED)) {
#ifdef LCD16X2
    g_OBD.LcdSetBacklightColor(YELLOW);
#endif // #ifdef LCD16X2

    if (!DiodeCheckEnabled()) {
      DisabledTest_P(g_psDiodeCheck);
    }
    if (!VentReqEnabled()) {
      DisabledTest_P(g_psVentReqChk);
    }
#ifdef ADVPWR
    if (!GndChkEnabled()) {
      DisabledTest_P(g_psGndChk);
    }
    if (!StuckRelayChkEnabled()) {
      DisabledTest_P(g_psRlyChk);
    }
#endif // ADVPWR
#ifdef GFI_SELFTEST
    if (!GfiSelfTestEnabled()) {
      DisabledTest_P(g_psGfiTest);
    }
#endif // GFI_SELFTEST
#ifdef TEMPERATURE_MONITORING
    if (!TempChkEnabled()) {
      DisabledTest_P(g_psTempChk);
    }
#endif // TEMPERATURE_MONITORING

#ifdef LCD16X2
    g_OBD.LcdSetBacklightColor(WHITE);
#endif
  }
}
#endif //SHOW_DISABLED_TESTS

void J1772EVSEController::chargingOn()
{  // turn on charging current
#ifdef RELAY_AUTO_PWM_PIN
  // turn on charging pin to close relay
  digitalWrite(RELAY_AUTO_PWM_PIN,HIGH);
  delay(m_relayCloseMs);
  // switch to PWM to hold closed
  analogWrite(RELAY_AUTO_PWM_PIN,m_relayHoldPwm);
#else // !RELAY_AUTO_PWM_PIN
#ifdef CHARGING_REG
  pinCharging.write(1);
#endif
#ifdef CHARGING2_REG
  pinCharging2.write(1);
#endif // CHARGING2_REG
#ifdef CHARGINGAC_REG
  pinChargingAC.write(1);
#endif
#endif // RELAY_AUTO_PWM_PIN

  m_bVFlags |= ECVF_CHARGING_ON;
  
  if (m_bVFlags2 & ECVF2_SESSION_ENDED) {
    m_AccumulatedChargeTime = 0;
    m_bVFlags2 &= ~ECVF2_SESSION_ENDED;
  }
  else {
    m_AccumulatedChargeTime += m_ElapsedChargeTime;
  }

  m_ChargeOnTime = now();
  m_ChargeOnTimeMS = millis();
}

void J1772EVSEController::chargingOff()
{ // turn off charging current
#ifdef RELAY_AUTO_PWM_PIN
  digitalWrite(RELAY_AUTO_PWM_PIN,LOW);
#else // !RELAY_AUTO_PWM_PIN
#ifdef CHARGING_REG
  pinCharging.write(0);
#endif
#ifdef CHARGING2_REG
  pinCharging2.write(0);
#endif
#ifdef CHARGINGAC_REG
  pinChargingAC.write(0);
#endif
#endif // RELAY_AUTO_PWM_PIN

  m_bVFlags &= ~ECVF_CHARGING_ON;

  m_ChargeOffTime = now();
  m_ChargeOffTimeMS = millis();

#ifdef AMMETER
  m_ChargingCurrent = 0;
#endif
} 

void J1772EVSEController::HardFault()
{
  SetHardFault();
  g_OBD.Update(OBD_UPD_HARDFAULT);
#ifdef RAPI
  RapiSendEvseState();
#endif
#ifdef MENNEKES_LOCK
  m_MennekesLock.Unlock();
#endif // MENNEKES_LOCK
  while (1) {
    ProcessInputs(); // spin forever or until user resets via menu
    // if pilot not in N12 state, we can recover from the hard fault when EV
    // is unplugged
    if (m_Pilot.GetState() != PILOT_STATE_N12) {
      ReadPilot(); // update EV connect state
      if (!EvConnected()) {
	// EV disconnected - cancel fault
	m_EvseState = EVSE_STATE_UNKNOWN;
	break;
      }
    }
  }
  ClrHardFault();
}

#ifdef GFI
void J1772EVSEController::SetGfiTripped()
{
#ifdef GFI_SELFTEST
  if (m_Gfi.SelfTestInProgress()) {
    m_Gfi.SetTestSuccess();
    return;
  }
#endif
  m_bVFlags |= ECVF_GFI_TRIPPED;

  // this is repeated in Update(), but we want to keep latency as low as possible
  // for safety so we do it here first anyway
  chargingOff(); // turn off charging current
  // turn off the PWM
  m_Pilot.SetState(PILOT_STATE_P12);

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

#ifdef TEMPERATURE_MONITORING
void J1772EVSEController::EnableTempChk(uint8_t tf)
{
  if (tf) {
    m_wFlags &= ~ECF_TEMP_CHK_DISABLED;
  }
  else {
    m_wFlags |= ECF_TEMP_CHK_DISABLED;
  }
  SaveEvseFlags();
}
#endif // TEMPERATURE_MONITORING

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

#ifdef AUTOSVCLEVEL
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
#endif // AUTOSVCLEVEL



#endif // ADVPWR

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
#endif // RGBLCD
  return 0;
}
#endif // RGBLCD
void J1772EVSEController::Enable()
{
  if ((m_EvseState == EVSE_STATE_DISABLED)||
      (m_EvseState == EVSE_STATE_SLEEPING)) {
#ifdef SLEEP_STATUS_REG
    if (m_EvseState == EVSE_STATE_SLEEPING) {
      pinSleepStatus.write(0);
    }
#endif // SLEEP_STATUS_REG
#if defined(TIME_LIMIT) || defined(CHARGE_LIMIT)
	SetLimitSleep(0);
#endif //defined(TIME_LIMIT) || defined(CHARGE_LIMIT)
    
    m_PrevEvseState = EVSE_STATE_DISABLED;
    m_EvseState = EVSE_STATE_UNKNOWN;
    m_Pilot.SetState(PILOT_STATE_P12);
  }
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
    RapiSendEvseState();
#endif // RAPI
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
    RapiSendEvseState();
#endif // RAPI
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
#ifdef SERDBG
  if (SerDbgEnabled()) {
    Serial.print("SetSvcLevel: ");Serial.println((int)svclvl);
  }
#endif //#ifdef SERDBG
  if (svclvl == 2) {
    m_wFlags |= ECF_L2; // set to Level 2
  }
  else {
    svclvl = 1; // force invalid value to L1
    m_wFlags &= ~ECF_L2; // set to Level 1
  }

  SaveEvseFlags();

  uint8_t ampacity = GetMaxCurrentCapacity();


  LoadThresholds();

  SetCurrentCapacity(ampacity,0,1);

  if (updatelcd) {
    g_OBD.Update(OBD_UPD_FORCE);
  }
}

uint8_t J1772EVSEController::GetMaxCurrentCapacity()
{
  uint8_t svclvl = GetCurSvcLevel();
  uint8_t ampacity =  eeprom_read_byte((uint8_t*)((svclvl == 1) ? EOFS_CURRENT_CAPACITY_L1 : EOFS_CURRENT_CAPACITY_L2));

  if ((ampacity == 0xff) || (ampacity == 0)) {
    ampacity = (svclvl == 1) ? DEFAULT_CURRENT_CAPACITY_L1 : DEFAULT_CURRENT_CAPACITY_L2;
  }
  
  if (ampacity < MIN_CURRENT_CAPACITY_J1772) {
    ampacity = MIN_CURRENT_CAPACITY_J1772;
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

  return ampacity;
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

  uint8_t RelayOff;
#ifndef OPENEVSE_2
  uint8_t Relay1, Relay2; //Relay Power status
#endif
  uint8_t svcState = UD;	// service state = undefined

#ifdef SERDBG
  if (SerDbgEnabled()) {
    Serial.print("POST start...");
  }
#endif //#ifdef SERDBG


  m_Pilot.SetState(PILOT_STATE_P12); //check to see if EV is plugged in

  g_OBD.SetRedLed(1); 
#ifdef LCD16X2 //Adafruit RGB LCD
  g_OBD.LcdMsg_P(g_psPwrOn,g_psSelfTest);
#endif //Adafruit RGB LCD 

#ifdef AUTOSVCLEVEL
  if (AutoSvcLevelEnabled()) {
#ifdef OPENEVSE_2
    // For OpenEVSE II, there is a voltmeter for auto L1/L2.
    uint32_t long ac_volts = ReadVoltmeter();
    if (ac_volts > L2_VOLTAGE_THRESHOLD) {
      svcState = L2;
    } else {
      svcState = L1;
    }
#ifdef SERDBG
    if (SerDbgEnabled()) {
      Serial.print("AC millivolts: ");Serial.println(ac_volts);
      Serial.print("SvcState: ");Serial.println((int)svcState);
    }  
#endif //#ifdef SERDBG
#ifdef LCD16X2
    g_OBD.LcdMsg_P(g_psAutoDetect,(svcState == L2) ? g_psLevel2 : g_psLevel1);
#endif //LCD16x2

#else //!OPENEVSE_2

    delay(150); // delay reading for stable pilot before reading
    int reading = adcPilot.read(); //read pilot
#ifdef SERDBG
    if (SerDbgEnabled()) {
      Serial.print("Pilot: ");Serial.println((int)reading);
    }
#endif //#ifdef SERDBG

    m_Pilot.SetState(PILOT_STATE_N12);
    if (reading > 900) {  // IF EV is not connected its Okay to open the relay the do the L1/L2 and ground Check

      // save state with both relays off - for stuck relay state
      RelayOff = ReadACPins();
          
      // save state with Relay 1 on 
#ifdef CHARGING_REG
      pinCharging.write(1);
#endif
#ifdef CHARGINGAC_REG
      pinChargingAC.write(1);
#endif
      delay(RelaySettlingTime);
      Relay1 = ReadACPins();
#ifdef CHARGING_REG
      pinCharging.write(0);
#endif
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
#ifdef SERDBG
      if (SerDbgEnabled()) {
	Serial.print("RelayOff: ");Serial.println((int)RelayOff);
	Serial.print("Relay1: ");Serial.println((int)Relay1);
	Serial.print("Relay2: ");Serial.println((int)Relay2);
	Serial.print("SvcState: ");Serial.println((int)svcState);
      }  
#endif //#ifdef SERDBG

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
#endif // AUTOSVCLEVEL
#ifndef OPENEVSE_2
  stuckrelaychk:
#endif
    if (StuckRelayChkEnabled()) {
      RelayOff = ReadACPins();
      if ((RelayOff & 3) != 3) {
	svcState = SR;
#ifdef LCD16X2
	g_OBD.LcdMsg_P(g_psTestFailed,g_psStuckRelay);
#endif // LCD16X2
      }
    }
#ifdef AUTOSVCLEVEL
  } // endif AutoSvcLevelEnabled
#endif // AUTOSVCLEVEL
  
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
#ifdef LCD16X2
    g_OBD.LcdSetBacklightColor(RED);
#endif // LCD16X2
    g_OBD.SetGreenLed(0);
    g_OBD.SetRedLed(1);
  }
  else {
    g_OBD.SetRedLed(0);
  }
  m_Pilot.SetState(PILOT_STATE_P12);

#ifdef SERDBG
  if (SerDbgEnabled()) {
    Serial.print("POST result: ");
    Serial.println((int)svcState);
  }
#endif //#ifdef SERDBG

  WDT_RESET();

  return svcState;
}
#endif // ADVPWR

void J1772EVSEController::Init()
{
#ifdef MENNEKES_LOCK
  m_MennekesLock.Init();
#endif // MENNEKES_LOCK

  m_EvseState = EVSE_STATE_UNKNOWN;
  m_PrevEvseState = EVSE_STATE_UNKNOWN;

#ifdef RAPI
  RapiSendEvseState(0);
#endif

  // read settings from EEPROM
  uint16_t rflgs = eeprom_read_word((uint16_t*)EOFS_FLAGS);

#ifdef RGBLCD
  if ((rflgs != 0xffff) && (rflgs & ECF_MONO_LCD)) {
    g_OBD.LcdSetBacklightType(BKL_TYPE_MONO);
  }
#endif // RGBLCD

#ifdef RELAY_AUTO_PWM_PIN_TESTING
  m_relayHoldPwm = eeprom_read_byte((uint8_t*)EOFS_RELAY_HOLD_PWM);
  m_relayCloseMs = eeprom_read_dword((uint32_t*)EOFS_RELAY_CLOSE_MS);
  if (m_relayCloseMs > 5000) {
    m_relayCloseMs = 0;
    m_relayHoldPwm = 248;
  }
  Serial.print("\nrelayCloseMs: ");Serial.println(m_relayCloseMs);
  Serial.print("relayHoldPwm: ");Serial.println(m_relayHoldPwm);
#endif


#ifdef RELAY_AUTO_PWM_PIN
  pinMode(RELAY_AUTO_PWM_PIN,OUTPUT);
#else // !RELAY_AUTO_PWM_PIN
#ifdef CHARGING_REG
  pinCharging.init(CHARGING_REG,CHARGING_IDX,DigitalPin::OUT);
#endif
#ifdef CHARGING2_REG
  pinCharging2.init(CHARGING2_REG,CHARGING2_IDX,DigitalPin::OUT);
#endif
#endif // RELAY_AUTO_PWM_PIN

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
#ifdef AUTH_LOCK_REG
  pinAuthLock.init(AUTH_LOCK_REG,AUTH_LOCK_IDX,DigitalPin::INP_PU);
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
  m_wFlags |= ECF_DIODE_CHK_DISABLED|ECF_VENT_REQ_DISABLED|ECF_GND_CHK_DISABLED|ECF_STUCK_RELAY_CHK_DISABLED|ECF_GFI_TEST_DISABLED|ECF_TEMP_CHK_DISABLED;
#endif

#ifdef SERDBG
  EnableSerDbg(1);
#endif

#ifdef AMMETER
  m_AmmeterCurrentOffset = eeprom_read_word((uint16_t*)EOFS_AMMETER_CURR_OFFSET);
  m_CurrentScaleFactor = eeprom_read_word((uint16_t*)EOFS_CURRENT_SCALE_FACTOR);
  
  if (m_AmmeterCurrentOffset == (int16_t)0xffff) {
    m_AmmeterCurrentOffset = DEFAULT_AMMETER_CURRENT_OFFSET;
  }
  if (m_CurrentScaleFactor == (int16_t)0xffff) {
    m_CurrentScaleFactor = DEFAULT_CURRENT_SCALE_FACTOR;
  }
  
  m_AmmeterReading = 0;
  m_ChargingCurrent = 0;
#ifdef OVERCURRENT_THRESHOLD
  m_OverCurrentStartMs = 0;
#endif //OVERCURRENT_THRESHOLD

#endif // AMMETER

#ifdef VOLTMETER
  m_VoltOffset = eeprom_read_dword((uint32_t*)EOFS_VOLT_OFFSET);
  m_VoltScaleFactor = eeprom_read_word((uint16_t*)EOFS_VOLT_SCALE_FACTOR);
  
  if (m_VoltOffset == 0xffffffff) {
    m_VoltOffset = DEFAULT_VOLT_OFFSET;
  }
  if (m_VoltScaleFactor == 0xffff) {
    m_VoltScaleFactor = DEFAULT_VOLT_SCALE_FACTOR;
  }
#endif // VOLTMETER

#ifndef RGBLCD
  m_wFlags |= ECF_MONO_LCD;
#endif

  m_bVFlags = ECVF_DEFAULT;
  m_bVFlags2 = ECVF2_DEFAULT;
#ifdef GFI
  m_GfiRetryCnt = 0;
  m_GfiTripCnt = eeprom_read_byte((uint8_t*)EOFS_GFI_TRIP_CNT);
#endif // GFI
#ifdef ADVPWR
  m_NoGndRetryCnt = 0;
  m_NoGndTripCnt = eeprom_read_byte((uint8_t*)EOFS_NOGND_TRIP_CNT);

  m_StuckRelayStartTimeMS = 0;
  m_StuckRelayTripCnt = eeprom_read_byte((uint8_t*)EOFS_STUCK_RELAY_TRIP_CNT);

  m_NoGndRetryCnt = 0;
  m_NoGndStart = 0;
#endif // ADVPWR

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
 
  uint8_t fault; 
  do {
    fault = 0; // reset post fault
    uint8_t psvclvl = doPost(); // auto detect service level overrides any saved values
    
#ifdef AUTOSVCLEVEL
    if ((AutoSvcLevelEnabled()) && ((psvclvl == L1) || (psvclvl == L2)))  svclvl = psvclvl; //set service level
#endif // AUTOSVCLEVEL
    if ((GndChkEnabled()) && (psvclvl == OG))  { m_EvseState = EVSE_STATE_NO_GROUND; fault = 1;} // set No Ground error
    if ((StuckRelayChkEnabled()) && (psvclvl == SR)) { m_EvseState = EVSE_STATE_STUCK_RELAY; fault = 1; } // set Stuck Relay error
#ifdef GFI_SELFTEST
    if ((GfiSelfTestEnabled()) && (psvclvl == FG)) { m_EvseState = EVSE_STATE_GFI_TEST_FAILED; fault = 1; } // set GFI test fail error
#endif
    if (fault) {
#ifdef UL_COMPLIANT
      // UL wants EVSE to hard fault until power cycle if POST fails
#ifdef RAPI
      RapiSendEvseState();
#endif
      while (1) { // spin forever
	  ProcessInputs();
      }
#else // !UL_COMPLIANT
      unsigned long faultms = millis();
      // keep retrying POST every 2 minutes
      while ((millis() - faultms) < 2*60000ul) {
	ProcessInputs();
      }
#endif
    }
  } while ( fault && ( m_EvseState == EVSE_STATE_GFI_TEST_FAILED || m_EvseState == EVSE_STATE_NO_GROUND ||  m_EvseState == EVSE_STATE_STUCK_RELAY ));
#endif // ADVPWR  

  SetSvcLevel(svclvl);

#ifdef DELAYTIMER
  if (g_DelayTimer.IsTimerEnabled()) {
    Sleep();
  }
#endif

  g_OBD.SetGreenLed(0);
}

void J1772EVSEController::ReadPilot(uint16_t *plow,uint16_t *phigh)
{
  uint16_t pl = 1023;
  uint16_t ph = 0;

  // 1x = 114us 20x = 2.3ms 100x = 11.3ms
  for (int i=0;i < PILOT_LOOP_CNT;i++) {
    uint16_t reading = adcPilot.read();  // measures pilot voltage
    
    if (reading > ph) {
      ph = reading;
    }
    else if (reading < pl) {
      pl = reading;
    }
  }

  if (m_Pilot.GetState() != PILOT_STATE_N12) {
    // update prev state
    if (EvConnected()) SetEvConnectedPrev();
    else ClrEvConnectedPrev();

    // can determine connected state only if not -12VDC
    if (ph >= m_ThreshData.m_ThreshAB) ClrEvConnected();
    else SetEvConnected();
  }

  if (plow) {
    *plow = pl;
    *phigh = ph;
  }
}


//TABLE A1 - PILOT LINE VOLTAGE RANGES (recommended.. adjust as necessary
//                           Minimum Nominal Maximum 
//Positive Voltage, State A  11.40 12.00 12.60 
//Positive Voltage, State B  8.36 9.00 9.56 
//Positive Voltage, State C  5.48 6.00 6.49 
//Positive Voltage, State D  2.62 3.00 3.25 
//Negative Voltage - States B, C, D, and F -11.40 -12.00 -12.60 
void J1772EVSEController::Update(uint8_t forcetransition)
{
  uint16_t plow;
  uint16_t phigh = 0xffff;

  unsigned long curms = millis();

  if (m_EvseState == EVSE_STATE_DISABLED) {
    m_PrevEvseState = m_EvseState; // cancel state transition
    return;
  }
  else if (m_EvseState == EVSE_STATE_SLEEPING) {
    ReadPilot(&plow,&phigh); // always read so we can update EV connect state, too

    int8_t cancelTransition = 1;
    if (chargingIsOn()) {
      // wait for pilot voltage to go > STATE C. This will happen if
      // a) EV reacts and goes back to state B (opens its contacts)
      // b) user pulls out the charge connector
      // if it doesn't happen within 3 sec, we'll just open our relay anyway
      // c) no current draw means EV opened its contacts even if it stays in STATE C
      //    allow 3A slop for ammeter inaccuracy
      if ((phigh >= m_ThreshData.m_ThreshBC)
#ifdef AMMETER
	  || (m_AmmeterReading <= 3000)
#endif // AMMETER
	  || ((curms - m_ChargeOffTimeMS) >= 3000)) {
	chargingOff();
#ifdef FT_SLEEP_DELAY
	sprintf(g_sTmp,"SLEEP OPEN %d",(int)phigh);
	g_OBD.LcdMsg(g_sTmp,(phigh >= m_ThreshData.m_ThreshBC) ? "THRESH" : "TIMEOUT");
	wdt_delay(2000);
#endif
      }
    }
    else { // not charging
#if defined(TIME_LIMIT) || defined(CHARGE_LIMIT)
      if (LimitSleepIsSet()) {
	if (!EvConnected()) {
	  // if we went into sleep due to time/charge limit met, then
	  // automatically cancel the sleep when the car is unplugged
	  cancelTransition = 0;
	  SetLimitSleep(0);
	  m_EvseState = EVSE_STATE_UNKNOWN;
	}
      }
#endif //defined(TIME_LIMIT) || defined(CHARGE_LIMIT)

      if (EvConnectedTransition()) {
#ifdef DELAYTIMER
	if (!EvConnected()) {
	  g_DelayTimer.ClrManualOverride();
      }
#endif // DELAYTIMER
	g_OBD.Update(OBD_UPD_FORCE);
      }
    }

    if (cancelTransition) {
      m_PrevEvseState = m_EvseState; // cancel state transition
      return;
    }
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
	if ((prevevsestate != EVSE_STATE_NO_GROUND) && (((uint8_t)(m_NoGndTripCnt+1)) < 254)) {
	  m_NoGndTripCnt++;
	  eeprom_write_byte((uint8_t*)EOFS_NOGND_TRIP_CNT,m_NoGndTripCnt);
	}
	m_NoGndStart = curms;
	
	nofault = 0;
      }

#ifdef AUTOSVCLEVEL
      // if EV was plugged in during POST, we couldn't do AutoSvcLevel detection,
      // so we had to hardcode L1. During first charge session, we can probe and set to L2 if necessary
      if (AutoSvcLvlSkipped() && (m_EvseState == EVSE_STATE_C)) {
	if (!acpinstate) {
	  // set to L2
	  SetSvcLevel(2,1);
	}
	SetAutoSvcLvlSkipped(0);
      }
#endif // AUTOSVCLEVEL
    }
  }
  else { // !chargingIsOn() - relay open
    if (prevevsestate == EVSE_STATE_NO_GROUND) {
      // check to see if EV disconnected
      if (phigh == 0xffff) {
	ReadPilot();
      }
      if (!EvConnected()) {
	// EV disconnected - cancel fault
	m_EvseState = EVSE_STATE_UNKNOWN;
	return;
      }

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
	  if ((prevevsestate != EVSE_STATE_STUCK_RELAY) && (((uint8_t)(m_StuckRelayTripCnt+1)) < 254)) {
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

    if (prevevsestate != EVSE_STATE_GFCI_FAULT) { // state transition
      if (((uint8_t)(m_GfiTripCnt+1)) < 254) {
	m_GfiTripCnt++;
	eeprom_write_byte((uint8_t*)EOFS_GFI_TRIP_CNT,m_GfiTripCnt);
      }
      m_GfiRetryCnt = 0;
      m_GfiFaultStartMs = curms;
    }
    else { // was already in GFI fault
      // check to see if EV disconnected
      ReadPilot();  // update EV connect state
      if (!EvConnected()) {
	// EV disconnected - cancel fault
	m_EvseState = EVSE_STATE_UNKNOWN;
	m_Gfi.Reset();
	return;
      }

      if ((curms - m_GfiFaultStartMs) >= GFI_TIMEOUT) {
#ifdef FT_GFI_RETRY
	g_OBD.LcdMsg("Reset","GFI");
	delay(250);
#endif // FT_GFI_RETRY
	m_GfiRetryCnt++;
	
	if ((GFI_RETRY_COUNT != 255) && (m_GfiRetryCnt > GFI_RETRY_COUNT)) {
	  HardFault();
	  return;
	}
	else {
	  m_Gfi.Reset();
	  m_GfiFaultStartMs = 0;
	}
      }
    }

    nofault = 0;
  }
#endif // GFI


#ifdef TEMPERATURE_MONITORING                 //  A state for OverTemp fault
if (TempChkEnabled()) {
  if ((g_TempMonitor.m_TMP007_temperature >= TEMPERATURE_INFRARED_PANIC)  || 
      (g_TempMonitor.m_MCP9808_temperature >= TEMPERATURE_AMBIENT_PANIC)  ||
      (g_TempMonitor.m_DS3231_temperature >= TEMPERATURE_AMBIENT_PANIC))  { 
    tmpevsestate = EVSE_STATE_OVER_TEMPERATURE;
    m_EvseState = EVSE_STATE_OVER_TEMPERATURE;
    nofault = 0;
  }
}
#endif // TEMPERATURE_MONITORING

 uint8_t prevpilotstate = m_PilotState;
 uint8_t tmppilotstate = EVSE_STATE_UNKNOWN;

  if (nofault) {
    if ((prevevsestate >= EVSE_FAULT_STATE_BEGIN) &&
	(prevevsestate <= EVSE_FAULT_STATE_END)) {
      // just got out of fault state - pilot back on
      m_Pilot.SetState(PILOT_STATE_P12);
      prevevsestate = EVSE_STATE_UNKNOWN;
      m_EvseState = EVSE_STATE_UNKNOWN;
    }

    ReadPilot(&plow,&phigh);

    if (DiodeCheckEnabled() && (m_Pilot.GetState() == PILOT_STATE_PWM) && (plow >= m_ThreshData.m_ThreshDS)) {
      // diode check failed
      tmpevsestate = EVSE_STATE_DIODE_CHK_FAILED;
      tmppilotstate = EVSE_STATE_DIODE_CHK_FAILED;
    }
    else if (phigh >= m_ThreshData.m_ThreshAB) {
      // 12V EV not connected
      tmpevsestate = EVSE_STATE_A;
      tmppilotstate = EVSE_STATE_A;
    }
    else if (phigh >= m_ThreshData.m_ThreshBC) {
      // 9V EV connected, waiting for ready to charge
      tmpevsestate = EVSE_STATE_B;
      tmppilotstate = EVSE_STATE_B;
    }
    else if (phigh  >= m_ThreshData.m_ThreshCD) {
      // 6V ready to charge
      tmppilotstate = EVSE_STATE_C;
      if (m_Pilot.GetState() == PILOT_STATE_PWM) {
	tmpevsestate = EVSE_STATE_C;
      }
      else {
	// PWM is off so we can't charge.. force to State B
	tmpevsestate = EVSE_STATE_B;
      }
    }
    else if (phigh > m_ThreshData.m_ThreshD) {
      tmppilotstate = EVSE_STATE_D;
      // 3V ready to charge vent required
      if (VentReqEnabled()) {
	tmpevsestate = EVSE_STATE_D;
      }
      else {
	tmpevsestate = EVSE_STATE_C;
      }
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

  // debounce state transitions
  if (tmppilotstate != prevpilotstate) {
    if (tmppilotstate != m_TmpPilotState) {
      m_TmpPilotStateStart = curms;
    }
    else if ((curms - m_TmpPilotStateStart) >= DELAY_STATE_TRANSITION) {
      m_PilotState = tmppilotstate;
    }
  }


  m_TmpPilotState = tmppilotstate;
  m_TmpEvseState = tmpevsestate;

#ifdef FT_GFI_RETRY
  if (nofault && (prevevsestate == EVSE_STATE_C) && 
      ((curms - m_ChargeOnTimeMS) > 10000)) {
    g_OBD.LcdMsg("Induce","Fault");
    for(int i = 0; i < GFI_TEST_CYCLES; i++) {
      m_Gfi.pinTest.write(1);
      delayMicroseconds(GFI_PULSE_ON_US);
      m_Gfi.pinTest.write(0);
      delayMicroseconds(GFI_PULSE_OFF_US);
      if (m_Gfi.Fault()) break;
    }
  }
#endif // FT_GFI_RETRY

  //
  // check request for state transition to A/B/C
  //

#ifdef AUTH_LOCK
#ifdef AUTH_LOCK_REG
  {
    int8_t locked;
    if (pinAuthLock.read()) locked = ECVF_AUTH_LOCKED;
    else locked = 0;
    if (locked != (m_bVFlags & ECVF_AUTH_LOCKED)) {
      AuthLock(locked);
      g_OBD.Update(OBD_UPD_FORCE);
      forcetransition = 1;
    }
  }
#endif // AUTH_LOCK_REG

  if (AuthLockIsOn() && (m_EvseState == EVSE_STATE_C)) {
    // force to STATE B when lock is on
    m_EvseState = EVSE_STATE_B;
  }
#endif // AUTH_LOCK

  if (
#ifdef STATE_TRANSITION_REQ_FUNC
      m_StateTransitionReqFunc &&
#endif // STATE_TRANSITION_REQ_FUNC
      (m_EvseState != prevevsestate) &&
      ((m_EvseState >= EVSE_STATE_A) && (m_EvseState <= EVSE_STATE_C))) {
    m_PilotState = tmppilotstate;
#ifdef STATE_TRANSITION_REQ_FUNC
    uint8_t newstate = (*m_StateTransitionReqFunc)(prevpilotstate,m_PilotState,prevevsestate,m_EvseState);
    if (newstate) {
      m_EvseState = newstate;
    }
#endif // STATE_TRANSITION_REQ_FUNC
  }

  
  // state transition
  if (forcetransition || (m_EvseState != prevevsestate)) {
#ifdef MENNEKES_LOCK
    if (m_EvseState == MENNEKES_LOCK_STATE) {
      m_MennekesLock.Lock();
    }
    else {
      m_MennekesLock.Unlock();
    }
#endif // MENNEKES_LOCK


    if (m_EvseState == EVSE_STATE_A) { // EV not connected
      chargingOff(); // turn off charging current
      m_Pilot.SetState(PILOT_STATE_P12);
#ifdef CHARGE_LIMIT
	ClrChargeLimit();
#endif // CHARGE_LIMIT
#ifdef TIME_LIMIT
	ClrTimeLimit();
#endif // TIME_LIMIT
#ifdef DELAYTIMER
	g_DelayTimer.ClrManualOverride();
#endif // DELAYTIMER
#ifdef TEMPERATURE_MONITORING
    g_TempMonitor.ClrOverTemperatureLogged();
#endif
    }
    else if (m_EvseState == EVSE_STATE_B) { // connected 
      chargingOff(); // turn off charging current
#ifdef AUTH_LOCK
      // if locked, don't turn on PWM
      if (AuthLockIsOn()) {
	m_Pilot.SetState(PILOT_STATE_P12);
      }
      else {
	m_Pilot.SetPWM(m_CurrentCapacity);
      }
#else
      m_Pilot.SetPWM(m_CurrentCapacity);
#endif // AUTH_LOCK
    }
    else if (m_EvseState == EVSE_STATE_C) {
      m_Pilot.SetPWM(m_CurrentCapacity);
#ifdef UL_GFI_SELFTEST
      // test GFI before closing relay
      if (GfiSelfTestEnabled() && m_Gfi.SelfTest()) {
       // GFI test failed - hard fault
        m_EvseState = EVSE_STATE_GFI_TEST_FAILED;
	m_Pilot.SetState(PILOT_STATE_P12);
	HardFault();
	return;
      }
#endif // UL_GFI_SELFTEST

#ifdef FT_GFI_LOCKOUT
      for(int i = 0; i < GFI_TEST_CYCLES; i++) {
	m_Gfi.pinTest.write(1);
	delayMicroseconds(GFI_PULSE_ON_US);
	m_Gfi.pinTest.write(0);
	delayMicroseconds(GFI_PULSE_OFF_US);
	if (m_Gfi.Fault()) break;
      }
      g_OBD.LcdMsg("Closing","Relay");
      delay(150);
#endif // FT_GFI_LOCKOUT

      chargingOn(); // turn on charging current
    }
    else if (m_EvseState == EVSE_STATE_D) {
      // vent required not supported
      chargingOff(); // turn off charging current
      m_Pilot.SetState(PILOT_STATE_P12);
      HardFault();
    }
    else if (m_EvseState == EVSE_STATE_GFCI_FAULT) {
      // vehicle state F
      chargingOff(); // turn off charging current
      m_Pilot.SetState(PILOT_STATE_P12);
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
      m_Pilot.SetState(PILOT_STATE_P12);
      HardFault();
    }
    else if (m_EvseState == EVSE_STATE_NO_GROUND) {
      // Ground not detected
      chargingOff(); // turn off charging current
      m_Pilot.SetState(PILOT_STATE_P12);
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
#if defined(RAPI) && !defined(AUTH_LOCK)
    RapiSendEvseState();
#endif // RAPI && !AUTH_LOCK
#ifdef SERDBG
    if (SerDbgEnabled()) {
      Serial.print("state: ");
      switch (m_Pilot.GetState()) {
      case PILOT_STATE_P12: Serial.print("P12"); break;
      case PILOT_STATE_PWM: Serial.print("PWM"); break;
      case PILOT_STATE_N12: Serial.print("N12"); break;
      }
      Serial.print(" ");
      Serial.print((int)prevevsestate);
      Serial.print("->");
      Serial.print((int)m_EvseState);
      Serial.print(" p ");
      Serial.print(plow);
      Serial.print(" ");
      Serial.println(phigh);
    }
#endif //#ifdef SERDBG
  } // state transition

#ifdef AUTH_LOCK
  if ((m_EvseState != prevevsestate) ||
      (m_PilotState != prevpilotstate)) {
#ifdef RAPI
    RapiSendEvseState(0);
#endif // RAPI
    g_OBD.Update(OBD_UPD_FORCE);
  }
#endif // AUTH_LOCK

#ifdef UL_COMPLIANT
  if (!nofault && (prevevsestate == EVSE_STATE_C)) {
    // if fault happens immediately (within 2 sec) after charging starts, hard fault
    if ((curms - m_ChargeOnTimeMS) <= 2000) {
      HardFault();
      return;
    }
  }
#endif // UL_COMPLIANT

  m_PrevEvseState = prevevsestate;

#ifdef VOLTMETER
  ReadVoltmeter();
#endif // VOLTMETER
#ifdef AMMETER
  if (((m_EvseState == EVSE_STATE_C) && (m_CurrentScaleFactor > 0)) || AmmeterCalEnabled()) {
    
#ifndef FAKE_CHARGING_CURRENT
    readAmmeter();
    uint32_t ma = MovingAverage(m_AmmeterReading);
    if (ma != 0xffffffff) {
      m_ChargingCurrent = ma * m_CurrentScaleFactor - m_AmmeterCurrentOffset;  // subtract it
      if (m_ChargingCurrent < 0) {
	m_ChargingCurrent = 0;
      }
      g_OBD.SetAmmeterDirty(1);
    }
#endif // !FAKE_CHARGING_CURRENT
  }

#ifdef OVERCURRENT_THRESHOLD
  if (m_EvseState == EVSE_STATE_C) {
    //testing    m_ChargingCurrent = (m_CurrentCapacity+OVERCURRENT_THRESHOLD+12)*1000L;
    if (m_ChargingCurrent >= ((m_CurrentCapacity+OVERCURRENT_THRESHOLD)*1000L)) {
      if (m_OverCurrentStartMs) { // already in overcurrent state
	if ((millis()-m_OverCurrentStartMs) >= OVERCURRENT_TIMEOUT) {
	  //
	  // overcurrent for too long. stop charging and hard fault
	  //
	  m_EvseState = EVSE_STATE_OVER_CURRENT;

	  m_Pilot.SetState(PILOT_STATE_P12); // Signal the EV to pause
	  curms = millis();
	  while ((millis()-curms) < 1000) { // give EV 1s to stop charging
	    wdt_reset();
	  }
	  chargingOff(); // open the EVSE relays hopefully the EV has already discon

	  // spin until EV is disconnected
	  HardFault();
	  
	  m_OverCurrentStartMs = 0; // clear overcurrent
	}
      }
      else {
	m_OverCurrentStartMs = millis();
      }
    }
    else {
      m_OverCurrentStartMs = 0; // clear overcurrent
    }
  }
  else {
    m_OverCurrentStartMs = 0; // clear overcurrent
  }
#endif // OVERCURRENT_THRESHOLD    
#endif // AMMETER
  if (m_EvseState == EVSE_STATE_C) {
    m_ElapsedChargeTimePrev = m_ElapsedChargeTime;
    m_ElapsedChargeTime = now() - m_ChargeOnTime;


#ifdef TEMPERATURE_MONITORING
  if(TempChkEnabled()) {
    if (m_ElapsedChargeTime != m_ElapsedChargeTimePrev) {
      uint8_t currcap = GetMaxCurrentCapacity();
      uint8_t setit = 0;

      if (!g_TempMonitor.OverTemperature() && ((g_TempMonitor.m_TMP007_temperature   >= TEMPERATURE_INFRARED_THROTTLE_DOWN ) ||  // any sensor reaching threshold trips action
					       (g_TempMonitor.m_MCP9808_temperature  >= TEMPERATURE_AMBIENT_THROTTLE_DOWN ) ||
					       (g_TempMonitor.m_DS3231_temperature  >= TEMPERATURE_AMBIENT_THROTTLE_DOWN ))) {   // Throttle back the L2 current advice to the EV
	currcap /= 2;   // set to the throttled back level
	setit = 2;
      }

      else if (g_TempMonitor.OverTemperature() && ((g_TempMonitor.m_TMP007_temperature   <= TEMPERATURE_INFRARED_RESTORE_AMPERAGE ) &&  // all sensors need to show return to lower levels
					      (g_TempMonitor.m_MCP9808_temperature  <= TEMPERATURE_AMBIENT_RESTORE_AMPERAGE  ) &&
					      (g_TempMonitor.m_DS3231_temperature  <= TEMPERATURE_AMBIENT_RESTORE_AMPERAGE  ))) {  // restore the original L2 current advice to the EV
	setit = 1;    // set to the user's original setting for current
      }           
      
      
      else if (!g_TempMonitor.OverTemperatureShutdown() && ((g_TempMonitor.m_TMP007_temperature   >= TEMPERATURE_INFRARED_SHUTDOWN ) ||  // any sensor reaching threshold trips action
						       (g_TempMonitor.m_MCP9808_temperature  >= TEMPERATURE_AMBIENT_SHUTDOWN  )  ||
						       (g_TempMonitor.m_DS3231_temperature  >= TEMPERATURE_AMBIENT_SHUTDOWN  ))) {   // Throttle back the L2 current advice to the EV
  currcap /= 4;
	setit = 4;
      }
      
      else if (g_TempMonitor.OverTemperatureShutdown() && ((g_TempMonitor.m_TMP007_temperature   <= TEMPERATURE_INFRARED_THROTTLE_DOWN ) &&  // all sensors need to show return to lower levels
						      (g_TempMonitor.m_MCP9808_temperature  <= TEMPERATURE_AMBIENT_THROTTLE_DOWN )  &&
						      (g_TempMonitor.m_DS3231_temperature  <= TEMPERATURE_AMBIENT_THROTTLE_DOWN ))) {   //  restore the throttled down current advice to the EV since things have cooled down again
	currcap /= 2;    // set to the throttled back level
	setit = 3;
      }    
      if (setit) {
        if (setit <= 2) {
          g_TempMonitor.SetOverTemperature(setit-1);
	}
        else {
	  g_TempMonitor.SetOverTemperatureShutdown(setit-3);
	}
	SetCurrentCapacity(currcap,0,1);
    	if (m_Pilot.GetState() != PILOT_STATE_PWM) {
    	  m_Pilot.SetPWM(m_CurrentCapacity);
	      }
      }
    }
  }
#endif // TEMPERATURE_MONITORING
#ifdef CHARGE_LIMIT
    if (m_chargeLimitTotWs && (g_EnergyMeter.GetSessionWs() >= m_chargeLimitTotWs)) {
      ClrChargeLimit(); // clear charge limit
#ifdef TIME_LIMIT
      ClrTimeLimit(); // clear time limit
#endif // TIME_LIMIT
      SetLimitSleep(1);
      Sleep();
    }
#endif
#ifdef TIME_LIMIT
    if (m_timeLimitEnd) {
      // must call millis() below because curms is sampled before transition to
      // to State C, so m_ChargeOnTimeMS will be > curms from the start
      if (GetElapsedChargeTime() >= m_timeLimitEnd) {
	ClrTimeLimit(); // clear time limit
#ifdef CHARGE_LIMIT
	ClrChargeLimit(); // clear charge limit
#endif // CHARGE_LIMIT
	SetLimitSleep(1);
	Sleep();
      }
    }
#endif // TIME_LIMIT
  }

  return;
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

  if (nosave) {
    // temporary amps can't be > max set in EEPROM
    maxcurrentcap = GetMaxCurrentCapacity();
  }

#ifdef PP_AUTO_AMPACITY
  if ((GetState() >= EVSE_STATE_B) && (GetState() <= EVSE_STATE_C)) {
    uint8_t mcc = g_ACCController.ReadPPMaxAmps();
    if (mcc && (mcc < maxcurrentcap)) {
      maxcurrentcap = mcc;
    }
  }
#endif // PP_AUTO_AMPACITY

  if ((amps >= MIN_CURRENT_CAPACITY_J1772) && (amps <= maxcurrentcap)) {
    m_CurrentCapacity = amps;
  }
  else if (amps < MIN_CURRENT_CAPACITY_J1772) {
    m_CurrentCapacity = MIN_CURRENT_CAPACITY_J1772;
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

#if defined(GFI) || defined(ADVPWR)
unsigned long J1772EVSEController::GetResetMs()
{
#ifdef GFI
  return GFI_TIMEOUT - (millis() - ((m_EvseState == EVSE_STATE_GFCI_FAULT) ? m_GfiFaultStartMs : m_NoGndStart));
#else
  return GFI_TIMEOUT - (millis() - m_NoGndStart);
#endif // GFI
}
#endif // GFI || ADVPWR


#ifdef VOLTMETER
void J1772EVSEController::SetVoltmeter(uint16_t scale,uint32_t offset)
{
  m_VoltScaleFactor = scale;
  eeprom_write_word((uint16_t*)EOFS_VOLT_SCALE_FACTOR,scale);
  m_VoltOffset = offset;
  eeprom_write_dword((uint32_t*)EOFS_VOLT_OFFSET,offset);
}

uint32_t J1772EVSEController::ReadVoltmeter()
{
  unsigned int peak = 0;
  for(uint32_t start_time = millis(); (millis() - start_time) < VOLTMETER_POLL_INTERVAL; ) {
    unsigned int val = adcVoltMeter.read();
    if (val > peak) peak = val;
  }
  m_Voltage = ((uint32_t)peak) * ((uint32_t)m_VoltScaleFactor) + m_VoltOffset;
  return m_Voltage;
}
#endif // VOLTMETER

#ifdef CHARGE_LIMIT
void J1772EVSEController::SetChargeLimitkWh(uint8_t kwh)
{
  if (kwh) {
    m_chargeLimitkWh = kwh;
    // extend session by kwh
    m_chargeLimitTotWs = g_EnergyMeter.GetSessionWs() + (3600000ul * (uint32_t)kwh);
#ifdef DELAYTIMER
  g_DelayTimer.SetManualOverride();
#endif // DELAYTIMER
  }
  else {
    ClrChargeLimit();
#ifdef DELAYTIMER
    g_DelayTimer.ClrManualOverride();
#endif // DELAYTIMER
  }
}
#endif // CHARGE_LIMIT

#ifdef TIME_LIMIT
void J1772EVSEController::SetTimeLimit15(uint8_t mind15)
{
  if (mind15) {
    m_timeLimit15 = mind15;
    // extend session by mind15 15 min increments
    m_timeLimitEnd = GetElapsedChargeTime() + (time_t)(15lu*60lu * (unsigned long)mind15);
#ifdef DELAYTIMER
    g_DelayTimer.SetManualOverride();
#endif // DELAYTIMER
  }
  else {
    ClrTimeLimit();
#ifdef DELAYTIMER
    g_DelayTimer.ClrManualOverride();
#endif // DELAYTIMER
  }

}
#endif // TIME_LIMIT

//-- end J1772EVSEController
