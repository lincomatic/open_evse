// Copyright (C) 2016 Sam C. Lin
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// See LICENSE for a copy of the GNU General Public License or see
// it online at <http://www.gnu.org/licenses/>.

#include "open_evse.h"

#ifdef PP_AUTO_AMPACITY

#ifdef PP_TABLE_IEC
static PP_AMPS s_ppAmps[] = {
  {0,0},
  {93,63},  // 100 = 93
  {185,32}, // 220 = 185
  {415,20}, // 680 = 415
  {615,13}, // 1.5K = 615
  {1023,0}
};
#endif //PP_TABLE_IEC

#ifdef PP_TABLE_TESLA
static PP_AMPS s_ppAmps[] = {
  {877,0},
  {923,40}, // 9.08K = 923
  {995,24}, // 33.16K = 995
  {1012,16}, // 75K = 1012
  {1018,12}, // 140K = 1018
  {1023,0}
};
#endif // PP_TABLE_FF_TESLA

AutoCurrentCapacityController::AutoCurrentCapacityController() :
  adcPP(PP_PIN), curAmps(0), maxAmps(0)
{
}

void AutoCurrentCapacityController::AutoSetCurrentCapacity()
{
  // n.b. should probably sample a few times and average it
  uint16_t adcval = adcPP.read();

  uint8_t amps = 0;
  for (int8_t i=1;i < sizeof(s_ppAmps)/sizeof(s_ppAmps[0]);i++) {
    if (adcval <= (s_ppAmps[i].adcVal - (s_ppAmps[i].adcVal - s_ppAmps[i-1].adcVal)/2)) {
      amps = s_ppAmps[i-1].amps;
      break;
    }
  }

  if (amps > maxAmps) {
    amps = maxAmps;
  }

  g_EvseController.SetCurrentCapacity(amps,0,1);
  curAmps = amps;

  //  Serial.print("pp: ");Serial.print(adcval);Serial.print(" amps: ");Serial.println(amps);
}

#endif //PP_AUTO_AMPACITY


