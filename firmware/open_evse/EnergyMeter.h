// -*- C++ -*-
#pragma once

#ifdef KWH_RECORDING

#define VOLTS_FOR_L1 120       // conventional for North America
//  #define VOLTS_FOR_L2 230   // conventional for most of the world
#define VOLTS_FOR_L2 240       // conventional for North America


// m_bFlags
#define EMF_IN_SESSION 0x01 // in a charging session
#define EMF_EV_CONNECTED 0x02
#define EMF_RELAY_CLOSED 0x04
class EnergyMeter {
  unsigned long m_lastUpdateMs;
  uint32_t m_wattHoursTot; // accumulated across all charging sessions
  uint32_t m_wattSeconds;  // current charging session
  uint8_t m_bFlags;

  uint8_t inSession() { return m_bFlags & EMF_IN_SESSION ? 1 : 0; }
  void setInSession() { setBits(m_bFlags,EMF_IN_SESSION); }
  void clrInSession() { clrBits(m_bFlags,EMF_IN_SESSION); }

  uint8_t evConnected() { return m_bFlags & EMF_EV_CONNECTED ? 1 : 0; }
  void setEvConnected() { setBits(m_bFlags,EMF_EV_CONNECTED); }
  void clrEvConnected() { clrBits(m_bFlags,EMF_EV_CONNECTED); }

  uint8_t relayClosed() { return m_bFlags & EMF_RELAY_CLOSED ? 1 : 0; }
  void setRelayClosed() { setBits(m_bFlags,EMF_RELAY_CLOSED); }
  void clrRelayClosed() { clrBits(m_bFlags,EMF_RELAY_CLOSED); }


  void calcUsage();
  void startSession();
  void endSession();

public:
  EnergyMeter();

  void Update();
  void SaveTotkWh();
  void SetTotkWh(uint32_t whtot) { m_wattHoursTot = whtot; }
  uint32_t GetTotkWh() { return m_wattHoursTot; }
  uint32_t GetSessionWs() { return m_wattSeconds; }
};


extern EnergyMeter g_EnergyMeter;
#endif // KWH_RECORDING


