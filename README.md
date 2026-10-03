# FanPwmTach

4-wire PC fan controller and tachometer for a classic Arduino Nano (ATmega328P),
with optional closed-loop RPM holding.

- 25.000 kHz fan PWM (Intel 4-wire spec wants 21–28 kHz)
- Tachometer with RPM derived from averaged pulse intervals
- Open-loop duty control, or a PI loop that holds a commanded RPM
- ~6.1 kB flash, 222 B RAM

## Wiring

| Nano | Goes to |
|------|---------|
| `D9` | N-MOSFET module signal `+` (fan PWM drive) |
| `D2` | PC817 module `OUT` (tach input) |
| `5V` | PC817 `VCC` |
| `GND` | PC817 `GND`, N-MOSFET signal `-`, PSU/fan GND |

Fan side:

| Fan | Goes to |
|-----|---------|
| +12 V | PSU +12 V |
| GND | PSU GND |
| PWM (pin 4, blue) | N-MOSFET module `OUT-` |
| TACH (pin 3, green) | PC817 isolated input `IN-` |
| — | PC817 isolated input `IN+` ← PSU +12 V |

N-MOSFET module `DC-` → PSU GND.

With a fan splitter, read tach from one fan only; most splitters pass tach from
a single "master" connector so multiple open-collector outputs are not tied
together.

## The drive is inverted

The low-side MOSFET pulls the fan's PWM line to ground, and that line is pulled
up *inside the fan*. So a **high** gate means the fan is commanded **0%** — fan
percent is the complement of the Nano pin duty. `INVERT_DRIVE` handles this and
defaults to `true`. Set it `false` only if the fan PWM pin is driven push-pull.

A side effect: at power-on, before `setup()` runs, the module's gate pull-down
holds the MOSFET off, the fan line floats high, and the fan spins at full speed
until the sketch commands otherwise.

## Timer choice

25 kHz on an ATmega328P is only reachable two ways, and the pin picks the timer:

| Timer | Pins | Config | Steps | Cost |
|-------|------|--------|-------|------|
| **Timer1** (used) | D9, D10 | mode 14, `ICR1=639`, presc. 1 | 640 | `analogWrite(D10)`, Servo |
| Timer2 | D3, D11 | mode 7, `OCR2A=79`, presc. 8 | 80 | `tone()`, `analogWrite(D11)` |

Timer2 with prescaler 1 cannot go below ~31 kHz, so it needs prescaler 8 and
gives only 80 steps. Timer1 is the better choice. Timer0 is untouched either
way, so `millis()`/`micros()` keep working.

Note `A6`/`A7` are analog-input-only on a classic Nano and cannot drive output.

## Serial interface

115200 baud.

| Send | Effect |
|------|--------|
| `0`–`100` | set duty directly, open loop |
| `r<rpm>` | hold that RPM, closed loop (e.g. `r2500`) |
| `r0` | return to open loop at the current duty |
| `?` | print status immediately |

Status line, once a second:

```
mode=rpm tgt=2500  duty=61.9%  rpm=2455  pulses=4950
```

`pulses` is a raw monotonic edge count, kept for bring-up: dividing its rate by
`PULSES_PER_REV` is an independent check on the reported RPM.

## Implementation notes

- **Tach** is `INT0` on D2, falling edge, with a 500 µs minimum spacing to
  reject optocoupler ringing (caps at ~30,000 RPM, far above any PC fan). The
  PC817 inverts, but that does not matter — one falling edge per pulse either
  way, so the frequency is preserved.
- **RPM** comes from averaged pulse *intervals*, not a one-second pulse count.
  At 2 pulses/rev a plain count would only resolve 30 RPM.
- **0% and 100%** detach `COM1A1` and hold the pin at the rail rather than
  setting `OCR1A = 0`, which emits a one-cycle spike on AVR timers.
- **Duty is kept fractional** and converted straight to timer counts. Rounding
  to whole percent would throw away 640 steps down to 101, and at ~54 RPM per
  percent that quantization alone puts a ±27 RPM floor under the closed loop.
- **The PI loop** runs at 1 Hz with anti-windup clamping and bumpless transfer
  (the integrator is seeded from the current duty when the mode is entered).
  `Kp` stays well under the deadbeat value of `1/54 = 0.0185` on purpose: `rpm`
  is averaged over the previous second, so the proportional term always acts on
  a stale measurement and a large `Kp` just makes the loop hunt.

## Measured behaviour

Measured with **30 s dwell per step**, ascending from a dead stop then back
down. Every point settled to a spread of 0–3 RPM over the final 8 s, so these
are genuine steady states.

| Duty | RPM (up) | RPM (down) |
|------|----------|------------|
| 0% | **0** | **0** |
| 20% | 532 | 535 |
| 30% | 534 | 535 |
| 40% | 1283 | — |
| 50% | 1915 | 1909 |
| 60% | 2390 | — |
| 70% | 2756 | 2770 |
| 80% | 3078 | — |
| 90% | 3303 | — |
| 100% | 3763 | — |

Four things fall out of this:

- **0% stops the fan.** Holding the PWM line at DC ground halts it completely.
  There is no minimum-speed floor.
- **No hysteresis.** Ascending and descending agree within 0.5%, so direction
  of approach does not matter once settled.
- **Below ~30% there is no control authority.** 20% and 30% both give ~533 RPM;
  the fan clamps to its own minimum. 20% *does* start the fan from a dead stop,
  so this is not a stall threshold. `MIN_DUTY = 30` therefore costs nothing and
  marks the knee of the curve rather than guarding against a stall.
- **The response is strongly nonlinear** — about 75 RPM per percent across
  30→40%, but only 23 RPM per percent across 80→90%, a 3x spread. See the note
  on `CTRL_KP` below for what that means for tuning.

Two oddities worth knowing:

- **100% is not just "99% plus one".** 90→100% adds 460 RPM where 80→90% added
  only 225. At 100% the code detaches the compare output and holds the line at
  DC high, and the fan appears to treat the absence of PWM as a full-speed
  override. It is also the only noisy point (±17 RPM vs ±1–3 elsewhere).
- **Output drifts downward over a session.** 60% duty read 2601 RPM early on,
  2410 later, and 2390 in this sweep; 100% went from 4382 to 3763. Cause
  unconfirmed — plausibly motor warm-up or a sagging 12 V rail. The closed loop
  absorbs it; open-loop duty does not. Re-measure before trusting absolute
  figures.

A warning on method, since it produced two wrong conclusions here: a sweep with
**5-second dwell does not settle**, and its numbers are points on a transient.
It made 0% look like it had a ~620 RPM floor (coastdown) and made the fan look
hysteretic (1333 RPM at 40% ascending vs 1681 descending). Both vanished at
30 s dwell.

Closed-loop step response settles inside ~1% of target with no overshoot.

`PULSES_PER_REV = 2` is the standard for PC fans but is **assumed, not
verified** here. If your fan's spec sheet disagrees with the reported top-end
RPM by 2x, that constant is why.

## FanDiag

`FanDiag/` is a separate bring-up sketch for when the tach reads zero and you
need to know why. At three fan commands it reports, over a one-second window,
the raw high/low sample counts on D2, the edge count seen by brute-force
polling, and the edge count seen by the interrupt handler — plus the live
Timer1 registers.

Reading those together separates the failure modes:

| Symptom | Means |
|---------|-------|
| All zero, line stuck at one level | no signal arriving at all |
| Polled edges but no ISR edges | interrupt misconfigured |
| Both counting | tach path healthy; any RPM error is in the maths |

One warning it encodes: a hard-low reading on a pin wired to a driver module is
**not** proof of a short. Those modules carry a gate pull-down that beats the
AVR internal pull-up, so probing a PWM output pin as `INPUT_PULLUP` reads a
convincing dead zero. That mistake cost real time on this rig.

## Build

```sh
arduino-cli core install arduino:avr
arduino-cli compile --fqbn arduino:avr:nano .
arduino-cli upload  --fqbn arduino:avr:nano -p COM4 .
```

`FanDiag` builds the same way, pointed at its own folder:

```sh
arduino-cli compile --fqbn arduino:avr:nano FanDiag
```

CH340-based Nano clones are often assumed to need the `atmega328old` 57600
bootloader variant; this board does not. Plain `arduino:avr:nano` at 115200
works.
