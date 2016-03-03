// -*- C++ -*-
/*
 * Open EVSE Firmware
 *
 * Copyright (c) 2013-2014 Sam C. Lin <lincomatic@gmail.com>
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

#ifdef MENNEKES_LOCK

void MennekesLock::Init()
{
  pinA.init(MENNEKES_LOCK_PINA_REG,MENNEKES_LOCK_PINA_IDX,DigitalPin::OUT);
  pinB.init(MENNEKES_LOCK_PINB_REG,MENNEKES_LOCK_PINB_IDX,DigitalPin::OUT);

  Unlock();
}

void MennekesLock::Lock()
{
  pinA.write(1);
  pinB.write(0);
  delay(300);
  pinA.write(0);
  pinB.write(0);
}

void MennekesLock::Unlock()
{
  pinA.write(0);
  pinB.write(1);
  delay(300);
  pinA.write(0);
  pinB.write(0);
}

#endif // MENNEKES_LOCK
