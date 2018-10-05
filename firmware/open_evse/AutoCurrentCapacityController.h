// -*- C++ -*-
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

#include "avrstuff.h"

typedef struct pp_amps {
  uint16_t adcVal;
  uint8_t amps;
} PP_AMPS;

class AutoCurrentCapacityController {
  AdcPin adcPP;

public:
  AutoCurrentCapacityController();
  uint8_t ReadPPMaxAmps();
  uint8_t AutoSetCurrentCapacity();
};


