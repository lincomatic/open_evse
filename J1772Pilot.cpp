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


#define TOP ((F_CPU / 2000000) * 1000) // for 1KHz (=1000us period)

void J1772Pilot::Init()
{
#ifdef PAFC_PWM
  // set up Timer for phase & frequency correct PWM
  TCCR1A = 0;  // set up Control Register A
  ICR1 = TOP;
  // WGM13 -> select P&F mode CS10 -> prescaler = 1
  TCCR1B = _BV(WGM13) | _BV(CS10);
 
#if (PILOT_IDX == 1) // PB1
  DDRB |= _BV(PORTB1);
  TCCR1A |= _BV(COM1A1);
#else // PB2
  DDRB |= _BV(PORTB2);
  TCCR1A |= _BV(COM1B1);
#endif // PILOT_IDX
#else // fast PWM
  pin.init(PILOT_REG,PILOT_IDX,DigitalPin::OUT);
#endif

  SetState(PILOT_STATE_P12); // turns the pilot on 12V steady state
}


// no PWM pilot signal - steady state
// PILOT_STATE_P12 = steady +12V (EVSE_STATE_A - VEHICLE NOT CONNECTED)
// PILOT_STATE_N12 = steady -12V (EVSE_STATE_F - FAULT) 
void J1772Pilot::SetState(PILOT_STATE state)
{
  AutoCriticalSection asc;

#ifdef PAFC_PWM
  if (state == PILOT_STATE_P12) {
#if (PILOT_IDX == 1)
    OCR1A = TOP;
#else
    OCR1B = TOP;
#endif
  }
  else {
#if (PILOT_IDX == 1) // PB1
    OCR1A = 0;
#else // PB2
    OCR1B = 0;
#endif
  }
#else // fast PWM
  TCCR1A = 0; //disable pwm by turning off COM1A1,COM1A0,COM1B1,COM1B0
  pin.write((state == PILOT_STATE_P12) ? 1 : 0);
#endif // PAFC_PWM

  m_State = state;
}


// set EVSE current capacity in Amperes
// duty cycle 
// outputting a 1KHz square wave to digital pin 10 via Timer 1
//
int J1772Pilot::SetPWM(int amps)
{

#ifdef PAFC_PWM
  // duty cycle = OCR1A(B) / ICR1 * 100 %

  unsigned cnt;
  if ((amps >= 6) && (amps <= 51)) {
    // amps = (duty cycle %) X 0.6
    cnt = amps * (TOP/60);
  } else if ((amps > 51) && (amps <= 80)) {
    // amps = (duty cycle % - 64) X 2.5
    cnt = (amps * (TOP/250)) + (64*(TOP/100));
  }
  else {
    return 1;
  }


#if (PILOT_IDX == 1) // PB1
  OCR1A = cnt;
#else // PB2
  OCR1B = cnt;
#endif
  
  m_State = PILOT_STATE_PWM;

  return 0;
#else // fast PWM
  uint8_t ocr1b = 0;
  if ((amps >= 6) && (amps <= 51)) {
    ocr1b = 25 * amps / 6 - 1;  // J1772 states "Available current = (duty cycle %) X 0.6"
  } else if ((amps > 51) && (amps <= 80)) {
    ocr1b = amps + 159;  // J1772 states "Available current = (duty cycle % - 64) X 2.5"
  }
  else {
    return 1; // error
  }

  if (ocr1b) {
    AutoCriticalSection asc;
    // Timer1 initialization:
    // 16MHz / 64 / (OCR1A+1) / 2 on digital 9
    // 16MHz / 64 / (OCR1A+1) on digital 10
    // 1KHz variable duty cycle on digital 10, 500Hz fixed 50% on digital 9
    // pin 10 duty cycle = (OCR1B+1)/(OCR1A+1)

    TCCR1A = _BV(COM1A0) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
    OCR1A = 249;

    // 10% = 24 , 96% = 239
    OCR1B = ocr1b;

    m_State = PILOT_STATE_PWM;
    return 0;
  }
  else { // !duty
    // invalid amps
    return 1;
  }
#endif // PAFC_PWM
}
