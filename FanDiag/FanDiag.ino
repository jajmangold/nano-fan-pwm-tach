/*
 * FanDiag - bring-up diagnostics for the FanPwmTach rig. Debug tool, not the
 * controller: it answers "is a tach signal actually arriving, and is Timer1
 * configured the way I think it is?"
 *
 *   D9   fan PWM out -> N-MOSFET module signal +   (Timer1 OC1A, 25.000 kHz)
 *   D2   tach in     <- PC817 module OUT
 *
 * At three fan commands it reports, for a 1-second window:
 *   D2_high / D2_low  - raw sample counts, so a stuck line is obvious
 *   polled_edges       - transitions seen by brute-force polling
 *   isr_edges          - transitions seen by the INT0 handler
 *
 * Reading those together separates the failure modes. All zeros with the line
 * stuck at one level means no signal is arriving at all. Polled edges without
 * ISR edges means the interrupt is misconfigured. Both counting means the
 * tach path is healthy and any RPM error is in the maths.
 *
 * Note a hard-low reading on a pin wired to a driver module is NOT proof of a
 * short: those modules carry a gate pull-down that beats the AVR internal
 * pull-up. That mistake cost real time on this rig.
 */

#include <Arduino.h>
#include <util/atomic.h>

static const uint8_t  PIN_FAN_PWM = 9;   /* OC1A */
static const uint8_t  PIN_TACH    = 2;   /* INT0 */
static const uint16_t PWM_TOP     = 639; /* 16 MHz / (1 * 640) = 25.000 kHz */

/* the low-side MOSFET inverts: gate high = fan line grounded = fan at 0% */
static const bool INVERT_DRIVE = true;

static volatile uint32_t v_isrEdges = 0;

static void tachIsr(void) { v_isrEdges++; }

static void applyPinCount(uint16_t cnt)
{
  if (cnt == 0 || cnt > PWM_TOP) {
    TCCR1A &= ~_BV(COM1A1);
    digitalWrite(PIN_FAN_PWM, cnt ? HIGH : LOW);
    return;
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { OCR1A = cnt; }
  TCCR1A |= _BV(COM1A1);
}

static void setFanPercent(float pct)
{
  float pin = INVERT_DRIVE ? (100.0f - pct) : pct;
  applyPinCount((uint16_t)(pin * (float)(PWM_TOP + 1) / 100.0f + 0.5f));
}

static void probe(const char *label, float fanPct)
{
  setFanPercent(fanPct);
  delay(2500);                                  /* let the fan settle */

  uint32_t hi = 0, lo = 0, trans = 0;
  uint8_t  prev = (uint8_t)digitalRead(PIN_TACH);

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v_isrEdges = 0; }

  uint32_t t0 = millis();
  while (millis() - t0 < 1000) {                /* poll D2 for 1 s */
    uint8_t s = (uint8_t)((PIND & _BV(PD2)) ? 1 : 0);
    if (s) hi++; else lo++;
    if (s != prev) { trans++; prev = s; }
  }

  uint32_t isr;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { isr = v_isrEdges; }

  Serial.print(label);
  Serial.print(F("  D2_high="));      Serial.print(hi);
  Serial.print(F(" D2_low="));        Serial.print(lo);
  Serial.print(F("  polled_edges=")); Serial.print(trans);
  Serial.print(F("  isr_edges="));    Serial.println(isr);
}

void setup(void)
{
  Serial.begin(115200);

  pinMode(PIN_FAN_PWM, OUTPUT);
  digitalWrite(PIN_FAN_PWM, LOW);
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  ICR1   = PWM_TOP;
  OCR1A  = 0;
  /* mode 14: fast PWM, TOP = ICR1; prescaler 1 */
  TCCR1A = _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);

  pinMode(PIN_TACH, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_TACH), tachIsr, FALLING);

  Serial.println(F("FanDiag - D9 pwm / D2 tach"));
}

void loop(void)
{
  Serial.print(F("regs TCCR1A=0x")); Serial.print(TCCR1A, HEX);
  Serial.print(F(" TCCR1B=0x"));     Serial.print(TCCR1B, HEX);
  Serial.print(F(" ICR1="));         Serial.print(ICR1);
  Serial.print(F(" OCR1A="));        Serial.println(OCR1A);

  probe("fan=0%  ",   0.0f);
  probe("fan=50% ",  50.0f);
  probe("fan=100%", 100.0f);
  Serial.println();
}
