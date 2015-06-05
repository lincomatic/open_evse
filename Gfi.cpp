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
