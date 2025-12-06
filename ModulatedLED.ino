#include <avr/io.h>
#include <avr/interrupt.h>

#define LED_MASK ((1 << PB1) | (1 << PB2) | (1 << PB3))

// Three modulation speeds (Timer1 prescaler = 1024)
const uint16_t OCR_SLOW = 7812;  // ~2 Hz
const uint16_t OCR_MED  = 781;   // ~20 Hz
const uint16_t OCR_FAST = 155;   // ~100 Hz

volatile uint8_t mode = 0;       // 0 = all blink, 1 = chase, 2 = binary
volatile uint8_t stepIndex = 0;  // step for chase/binary

// ---------------- Timer1 Compare A Interrupt ----------------
ISR(TIMER1_COMPA_vect) {
  switch (mode) {
    case 0:  // Mode 0: ALL THREE BLINK TOGETHER (in assembly)
      asm volatile(
        "in   r16, %[port]      \n\t"  // r16 = PORTB
        "ldi  r17, %[mask]      \n\t"  // r17 = LED_MASK bits
        "eor  r16, r17          \n\t"  // flip those bits
        "out  %[port], r16      \n\t"  // write back
        :
        : [port] "I" (_SFR_IO_ADDR(PORTB)),
          [mask] "I" (LED_MASK)
        : "r16", "r17"
      );
      break;

    case 1: {  // Mode 1: chasing pattern 9 -> 10 -> 11
      uint8_t newPort = PORTB & ~LED_MASK;  // clear LED bits

      if (stepIndex == 0) {
        newPort |= (1 << PB1);
      } else if (stepIndex == 1) {
        newPort |= (1 << PB2);
      } else { // stepIndex == 2
        newPort |= (1 << PB3);
      }

      stepIndex = (stepIndex + 1) % 3;
      PORTB = newPort;
      break;
    }

    case 2: {  // Mode 2: 3-bit binary counter on LEDs
      uint8_t newPort = PORTB & ~LED_MASK;  // clear LED bits
      uint8_t pattern = stepIndex & 0x07;   // 0..7

      if (pattern & 0x01) newPort |= (1 << PB1);
      if (pattern & 0x02) newPort |= (1 << PB2);
      if (pattern & 0x04) newPort |= (1 << PB3);

      stepIndex = (stepIndex + 1) & 0x07;  // stay in 0..7
      PORTB = newPort;
      break;
    }
  }
}

// ----------------------------- setup -----------------------------
void setup() {
  // LEDs: PB1, PB2, PB3 (pins 9,10,11) as outputs
  DDRB |= (1 << DDB1) | (1 << DDB2) | (1 << DDB3);

  // Button: PD2 (pin 2) as input with pull-up
  DDRD &= ~(1 << DDD2);    // input
  PORTD |= (1 << PORTD2);  // enable internal pull-up

  // Reset Timer1
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // CTC mode
  TCCR1B |= (1 << WGM12);

  // Start in slow mode (mode 0)
  OCR1A = OCR_SLOW;

  // Enable Timer1 Compare A interrupt
  TIMSK1 |= (1 << OCIE1A);

  // Start Timer1 with prescaler 1024
  TCCR1B |= (1 << CS12) | (1 << CS10);

  // Enable global interrupts
  sei();
}

// ----------------------------- loop -----------------------------
void loop() {
  // Button edge detect (HIGH -> LOW = press) with simple debounce
  static uint8_t lastState = 1;  // 1 = HIGH (not pressed)
  uint8_t reading = (PIND & (1 << PIND2)) ? 1 : 0;

  if (reading != lastState) {
    delay(10); // debounce
    reading = (PIND & (1 << PIND2)) ? 1 : 0;

    if (lastState == 1 && reading == 0) {
      // Button press: cycle mode
      mode = (mode + 1) % 3;
      stepIndex = 0;  // restart pattern

      uint16_t newOCR;
      if (mode == 0) {
        newOCR = OCR_SLOW;   // all blink slowly
      } else if (mode == 1) {
        newOCR = OCR_MED;    // chase medium speed
      } else {
        newOCR = OCR_FAST;   // binary fast
      }

      cli();
      OCR1A = newOCR;
      sei();
    }
    lastState = reading;
  }
}
