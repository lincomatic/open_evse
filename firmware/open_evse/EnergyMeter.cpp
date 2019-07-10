#include "open_evse.h"

#ifdef KWH_RECORDING

EnergyMeter g_EnergyMeter;


EnergyMeter::EnergyMeter()
{
  m_bFlags = 0;
  m_wattSeconds = 0;

  if (eeprom_read_dword((uint32_t*)EOFS_KWH_ACCUMULATED) == 0xffffffff) { // Check for unitialized eeprom condition so it can begin at 0kWh
    eeprom_write_dword((uint32_t*)EOFS_KWH_ACCUMULATED,0); //  Set the four bytes to zero just once in the case of unitialized eeprom
  }
  
  m_wattHoursTot = eeprom_read_dword((uint32_t*)EOFS_KWH_ACCUMULATED);        // get the stored value for the kWh from eeprom
}

void EnergyMeter::Update()
{
  // 1. charging session begins when EV is connected, ends when EV disconnected
  // 2. we record only when the relay is closed
  // 3. total kWh updated only when session ends
  uint8_t evconnected = g_EvseController.EvConnected();

  if (!evConnected() && evconnected) {
    startSession();
  }
  else if (!evconnected && evConnected()) {
    endSession();
  }

  if (inSession()) {
    uint8_t relayclosed = g_EvseController.RelayIsClosed();

    
    if (relayclosed) {
      if (!relayClosed()) {
	// relay just closed - don't calc, just reset timer
	m_lastUpdateMs = millis();
      }
      else {
	calcUsage();
      }
    }

    if (relayclosed) setRelayClosed();
    else clrRelayClosed();
  }

  if (evconnected) setEvConnected();
  else clrEvConnected();
}

void EnergyMeter::calcUsage()
{
  unsigned long curms = millis();
  unsigned long dms = curms - m_lastUpdateMs;
  if (dms > KWH_CALC_INTERVAL_MS) {
      uint32_t ma = g_EvseController.GetChargingCurrent();
#ifdef VOLTMETER
      m_wattSeconds += ((g_EvseController.GetVoltage()/1000UL) * (ma/1000UL) * dms) / 1000UL;
#else // !VOLTMETER
#ifdef THREEPHASE //Multiple L1 current by the square root of 3 to get 3-phase energy
      m_wattSeconds += (((g_EvseController.GetCurSvcLevel() == 2) ? VOLTS_FOR_L2:VOLTS_FOR_L1) * (ma/1000UL) * dms * 3) / 1000UL;
#else // !THREEPHASE
     m_wattSeconds += (((g_EvseController.GetCurSvcLevel() == 2) ? VOLTS_FOR_L2:VOLTS_FOR_L1) * (ma/1000UL) * dms) / 1000UL;
#endif // THREEPHASE
#endif // VOLTMETER

      m_lastUpdateMs = curms;
  }
}

void EnergyMeter::startSession()
{
  endSession();
  m_wattSeconds = 0;
  m_lastUpdateMs = millis();
  setInSession();
}

void EnergyMeter::endSession()
{
  if (inSession()) {
    clrInSession();
    if (m_wattSeconds) {
    m_wattHoursTot += (m_wattSeconds / 3600UL);
      SaveTotkWh();
    }
  }
}

void EnergyMeter::SaveTotkWh()
{
  eeprom_write_dword((uint32_t*)EOFS_KWH_ACCUMULATED,m_wattHoursTot);
}

#endif // KWH_RECORDING

