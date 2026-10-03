/*
 * FanPwmTach - 4-wire PC fan control + tachometer for the classic Arduino Nano
 *
 *   D9   fan PWM out -> N-MOSFET module signal +   (Timer1 OC1A, 25.000 kHz)
 *   D2   tach in     <- PC817 module OUT           (INT0, falling edge)
 *   5V   -> PC817 VCC
 *   GND  -> PC817 GND, N-MOSFET signal -, PSU/fan GND
 *
 * Timer1 is reclaimed for the fan carrier, which costs analogWrite() on D10
 * and the Servo library. Timer0 (millis/micros) and Timer2 (tone, D3/D11)
 * are left alone.
 *
 * Serial @ 115200:
 *   <0-100>    set duty directly, open loop
 *   r<rpm>     hold that RPM, closed loop  (r0 returns to open loop)
 *   ?          print status now
 */

#include <Arduino.h>
#include <util/atomic.h>

/* ---- configuration ----------------------------------------------------- */

static const uint8_t PIN_FAN_PWM = 9;   /* OC1A */
static const uint8_t PIN_TACH    = 2;   /* INT0 */

/* fast PWM, TOP = ICR1, prescaler 1:  16 MHz / (1 * (639+1)) = 25.000 kHz */
static const uint16_t PWM_TOP = 639;

static const uint8_t PULSES_PER_REV = 2;

/*
 * The low-side MOSFET pulls the fan's internally pulled-up PWM line to GND,
 * so the gate drive is inverted: gate high = fan line low = fan commanded 0%.
 * Set false only if the fan PWM pin is ever driven push-pull instead.
 */
static const bool INVERT_DRIVE = true;

/*
 * Measured on this fan: 20% duty stalls it (~100 RPM, below the 30% figure),
 * so anything in 1..MIN_DUTY-1 is lifted to MIN_DUTY. 0 is left alone and
 * means "minimum" - this fan floors at ~620 RPM rather than stopping.
 */
static const uint8_t MIN_DUTY = 30;

/*
 * Measured slope is ~54 RPM per % duty, so 1/54 = 0.0185 would be a one-step
 * deadbeat correction. Kp stays well under that on purpose: rpm here is an
 * average over the previous second, so the proportional term is always acting
 * on a stale measurement and a large Kp just makes the loop hunt. The integral
 * does the real work and is what pins the steady-state error to ~0.
 */
static const float CTRL_KP = 0.010f;    /* %duty per RPM            */
static const float CTRL_KI = 0.006f;    /* %duty per RPM per second */

static const uint16_t TACH_MIN_EDGE_US = 500;        /* opto glitch reject */
static const uint32_t TACH_TIMEOUT_US  = 1500000UL;  /* silence => 0 RPM */
static const uint16_t REPORT_MS        = 1000;

/* ---- tach state, shared ISR <-> loop ----------------------------------- */

static volatile uint32_t v_lastEdgeUs    = 0;
static volatile uint32_t v_intervalSumUs = 0;
static volatile uint16_t v_intervalCount = 0;
static volatile uint32_t v_totalPulses   = 0;
static volatile bool     v_haveFirstEdge = false;

static float    s_fanPercent   = 0.0f;  /* fractional: see applyFanPercent */
static uint32_t s_lastReportMs = 0;

static uint16_t s_targetRpm = 0;        /* 0 = open loop */
static float    s_integral  = 0.0f;

/* ---- tach -------------------------------------------------------------- */

static void tachIsr(void)
{
  uint32_t now = micros();
  uint32_t dt  = now - v_lastEdgeUs;

  if (v_haveFirstEdge) {
    if (dt < TACH_MIN_EDGE_US) return;   /* ringing: hold off, keep lastEdge */
    if (dt <= TACH_TIMEOUT_US) {         /* skip the gap after a stall */
      v_intervalSumUs += dt;
      v_intervalCount++;
    }
  }
  v_haveFirstEdge = true;
  v_lastEdgeUs    = now;
  v_totalPulses++;
}

static void tachBegin(void)
{
  pinMode(PIN_TACH, INPUT_PULLUP);     /* opto output may be open-collector */
  attachInterrupt(digitalPinToInterrupt(PIN_TACH), tachIsr, FALLING);
}

static uint32_t readRpm(void)
{
  uint32_t sum, last;
  uint16_t cnt;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    sum  = v_intervalSumUs;
    cnt  = v_intervalCount;
    last = v_lastEdgeUs;
    v_intervalSumUs = 0;
    v_intervalCount = 0;
  }

  if (cnt == 0) return 0;
  if ((uint32_t)(micros() - last) > TACH_TIMEOUT_US) return 0;

  uint32_t avgUs = sum / cnt;
  if (avgUs == 0) return 0;

  return 60000000UL / (avgUs * PULSES_PER_REV);
}

/* ---- fan PWM ----------------------------------------------------------- */

static void fanPwmBegin(void)
{
  pinMode(PIN_FAN_PWM, OUTPUT);
  digitalWrite(PIN_FAN_PWM, LOW);

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  ICR1   = PWM_TOP;
  OCR1A  = 0;

  /* mode 14: fast PWM, TOP = ICR1; OC1A non-inverting; prescaler 1 */
  TCCR1A = _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
}

/* raw compare value for the Nano pin itself, 0..PWM_TOP+1, after inversion */
static void applyPinCount(uint16_t cnt)
{
  if (cnt == 0 || cnt > PWM_TOP) {
    TCCR1A &= ~_BV(COM1A1);           /* detach OC1A and hold the rail, so */
    digitalWrite(PIN_FAN_PWM, cnt ? HIGH : LOW);  /* 0% has no output spike */
    return;
  }
  /* 16-bit write: keep it out of reach of the tach ISR */
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { OCR1A = cnt; }
  TCCR1A |= _BV(COM1A1);
}

/*
 * Raw setter, no stall clamp - the controller clamps for itself. Duty is kept
 * fractional and converted straight to timer counts: rounding to whole percent
 * would waste 639 of the 640 available steps down to 101, and at ~54 RPM per
 * percent that quantization alone puts a +-27 RPM floor under the closed loop.
 */
static void applyFanPercent(float pct)
{
  if (pct < 0.0f)   pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  s_fanPercent = pct;

  float pin = INVERT_DRIVE ? (100.0f - pct) : pct;
  applyPinCount((uint16_t)(pin * (float)(PWM_TOP + 1) / 100.0f + 0.5f));
}

/* ---- control ----------------------------------------------------------- */

static void setOpenLoop(float pct)
{
  s_targetRpm = 0;
  if (pct > 0.0f && pct < (float)MIN_DUTY) {
    Serial.print(F("note: "));
    Serial.print(pct, 1);
    Serial.print(F("% is in the stall band, lifting to "));
    Serial.print(MIN_DUTY);
    Serial.println(F("%"));
    pct = (float)MIN_DUTY;
  }
  applyFanPercent(pct);
  Serial.print(F("open loop -> "));
  Serial.print(s_fanPercent, 1);
  Serial.println(F("%"));
}

static void setTargetRpm(uint16_t rpm)
{
  if (rpm == 0) { setOpenLoop(s_fanPercent); return; }

  s_targetRpm = rpm;
  /* bumpless transfer: seed the integrator so output starts at the duty
   * already being applied, instead of jumping to the clamp */
  s_integral = s_fanPercent / CTRL_KI;

  Serial.print(F("closed loop -> "));
  Serial.print(rpm);
  Serial.println(F(" rpm"));
}

static void controlTick(uint32_t rpm)
{
  if (s_targetRpm == 0) return;

  float err = (float)s_targetRpm - (float)rpm;
  s_integral += err * (REPORT_MS / 1000.0f);

  /* hold the integral where Ki*integral still lands inside 0..100% */
  const float iMax = 100.0f / CTRL_KI;
  if (s_integral > iMax) s_integral = iMax;
  if (s_integral < 0.0f) s_integral = 0.0f;

  float duty = CTRL_KP * err + CTRL_KI * s_integral;
  if (duty < (float)MIN_DUTY) duty = (float)MIN_DUTY;
  if (duty > 100.0f)          duty = 100.0f;

  applyFanPercent(duty);
}

/* ---- serial ------------------------------------------------------------ */

static void report(uint32_t rpm)
{
  uint32_t pulses;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { pulses = v_totalPulses; }

  if (s_targetRpm) {
    Serial.print(F("mode=rpm tgt="));
    Serial.print(s_targetRpm);
  } else {
    Serial.print(F("mode=open      "));
  }
  Serial.print(F("  duty="));
  Serial.print(s_fanPercent, 1);
  Serial.print(F("%  rpm="));
  Serial.print(rpm);
  Serial.print(F("  pulses="));
  Serial.println(pulses);
}

static void handleSerial(void)
{
  static uint16_t val    = 0;
  static bool     digits = false;
  static bool     rpmCmd = false;

  while (Serial.available()) {
    int c = Serial.read();

    if (c == 'r' || c == 'R') {
      rpmCmd = true; val = 0; digits = false;
    } else if (c >= '0' && c <= '9') {
      val = val * 10 + (uint16_t)(c - '0');
      if (val > 20000) val = 20000;
      digits = true;
    } else if (c == '?') {
      report(readRpm());
    } else if (c == '\n' || c == '\r') {
      if (digits) {
        if (rpmCmd) setTargetRpm(val);
        else        setOpenLoop((float)(val > 100 ? 100 : val));
      }
      val = 0; digits = false; rpmCmd = false;
    }
  }
}

/* ---- main -------------------------------------------------------------- */

void setup(void)
{
  Serial.begin(115200);
  fanPwmBegin();
  tachBegin();
  applyFanPercent(0);

  Serial.println(F("FanPwmTach ready"));
  Serial.println(F("  PWM  D9 @ 25.000 kHz (Timer1 OC1A, 640 steps)"));
  Serial.println(F("  tach D2 @ 2 pulses/rev (INT0, falling)"));
  Serial.print  (F("  min duty "));
  Serial.print(MIN_DUTY);
  Serial.println(F("% (below this the fan stalls)"));
  Serial.println(F("  <0-100>=duty   r<rpm>=hold rpm   ?=status"));
}

void loop(void)
{
  handleSerial();

  uint32_t now = millis();
  if (now - s_lastReportMs >= REPORT_MS) {
    s_lastReportMs = now;
    uint32_t rpm = readRpm();
    controlTick(rpm);
    report(rpm);
  }
}
