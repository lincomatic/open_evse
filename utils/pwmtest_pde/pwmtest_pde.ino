
#define TOP ((F_CPU / 2000000) * 1000) // for 1KHz (=1000us period)
#define PILOT_PIN 10
void setup()
{

  // set up Timer for phase & frequency correct PWM
  TCCR1A = 0;  // set up Control Register A
  ICR1 = TOP;
  // WGM13 -> select P&F mode CS10 -> prescaler = 1
  TCCR1B = _BV(WGM13) | _BV(CS10);
 
 #if (PILOT_PIN == 9)
  DDRB |= _BV(PORTB1);
  TCCR1A |= _BV(COM1A1);
 #else // PILOT_PIN == 10
  DDRB |= _BV(PORTB2);
  TCCR1A |= _BV(COM1B1);
 #endif // PILOT_PIN


}

int amps = 6;
void loop()
{
if (amps < 80) amps++;
else amps = 6;
  unsigned cnt;
  if ((amps >= 6) && (amps <= 51)) {
  // amps = (duty cycle %) X 0.6
    cnt = amps * (TOP/60);
  } else if ((amps > 51) && (amps <= 80)) {
    // amps = (duty cycle % - 64) X 2.5
    cnt = (amps * (TOP/250)) + (64*(TOP/100));
  }
#if (PILOT_PIN == 9)
  OCR1A = cnt;
#else // PILOT_PIN == 10
    OCR1B = cnt;
#endif
delay(100);
}

