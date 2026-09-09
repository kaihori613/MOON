# Wiring

The hardware `actuator_v1/` expects. One brushed DC linear actuator, driven by
a single H-bridge, with position closed around the actuator's internal reed
switch. Everything else on this page is optional and compiled out by default.

**Nothing on this page has been built.** The pin assignments are reservations,
made so that `Config.h` and this document cannot drift apart; the motor and
reed rows match hardware that has run, the buttons, buzzer and IMU rows do not.

## Pin map

| Pin | Net | Notes |
|---|---|---|
| D0, D1 | USB serial | Console. Keep clear — every sketch here is driven from it |
| D2 | Reed switch | **INT0.** The only external interrupt the position loop can use |
| D5 | L298N IN2 | `digitalWrite` only, never PWM |
| D6 | L298N IN1 | `digitalWrite` only, never PWM |
| D9 | L298N ENA | PWM, Timer1 |
| D8 | *reserved* | HW-039 EN, when `MOTOR_DRIVER = DRV_HW039` |
| D10 | *reserved* | HW-039 LPWM, when `MOTOR_DRIVER = DRV_HW039` |
| D3 | MPU6050 INT | INT1. Optional — leave unwired, the part polls fine |
| D11 | Buzzer | Via a transistor, see below |
| A0 | Button — extend | To GND, `INPUT_PULLUP` |
| A1 | Button — retract | To GND, `INPUT_PULLUP` |
| A2 | Button — stop / home | To GND, `INPUT_PULLUP` |
| A4 | I2C SDA | LCD backpack 0x27 **and** MPU6050 0x68, same bus |
| A5 | I2C SCL | " |
| D4, D7, D12, D13, A3 | free | |

D8 and D10 stay reserved even on the L298N, so swapping to the BTS7960 is a
driver change in `Config.h` and not a rewiring job.

**I2C is A4/A5 and cannot be moved.** `Wire` on a 328P is tied to that
peripheral; there is no software I2C in this repo. An IMU on D12/D13 will not
enumerate no matter what the sketch says.

The LCD and the IMU share the bus without conflict — 0x27 and 0x68 do not
collide, and `Wire` is already initialised at 400 kHz for the display.

## Power

Two rails, one ground, and the ground is the part that matters.

```
  actuator supply (+)  ─────────────┬──── L298N  12V
                                    │
                                   470uF (bulk, observe polarity)
                                    │
  actuator supply (−)  ──┬──────────┴──── L298N  GND
                         │
                         └──────────────── Arduino GND      <- separate wire
```

The actuator supply is drawn as 12 V. **Confirm that against the actuator's
own plate before connecting anything** — plenty of TVRO units are 36 V, and
that changes both the supply and whether the L298N is usable at all.

### Ground

Star the two ground wires at the **supply negative**, as drawn. Do not run the
Arduino's ground to the L298N's GND terminal and call it done. That terminal
carries the motor return current — amps, switched at the PWM frequency — and
the IR drop along that wire lands directly on top of the reed input. This is
the pickup path the noise-floor issue in the README is about.

### 5 V

The L298N's `5V` pin is the **output** of an onboard 7805 whenever the `5V-EN`
jumper is fitted. So:

- **Bench, USB attached (this is the normal case today).** USB powers the
  logic. Leave `5V-EN` fitted; leave the L298N's `5V` pin **unconnected**.
  Buttons, buzzer and IMU take 5 V from the Arduino's `5V` pin as an output —
  a few tens of mA, well inside the USB budget.
- **Standalone, no PC.** Unplug USB, then `5V-EN` fitted and L298N `5V` →
  Arduino `5V`.

Never both. Driving the Arduino's `5V` pin from an external supply while USB is
connected puts that supply in a fight with the host through the USB polyfuse —
the Uno's auto-switchover arbitrates VIN against USB and does nothing for the
5 V pin. And feeding a supply *into* the L298N's `5V` pin with the jumper
fitted back-drives the 7805's output.

A separate bench 5 V supply is not needed for any of this.

## Reed switch

D2 to GND through the reed, plus the filter from the README's open issue:

- 4.7k pull-up from D2 to 5 V — the internal pullup is 20–50k, which is too
  weak to hold the line against a switching bridge a few inches away
- 220R in series with the reed
- 220nF from D2 to GND

The falling edge — the one `attachInterrupt` triggers on — sees only the 220R,
about 50 us. The rise is slower, roughly 1.4 ms through the parallel pull-ups,
which sits well under the 50 ms minimum real pulse gap measured on the bench
and well under `REED_DEBOUNCE_US = 3000`.

Keep `INPUT_PULLUP` set regardless: it costs nothing and it means a
disconnected reed reads high rather than floating.

## Capacitors

| Where | Value | Why |
|---|---|---|
| L298N 12V ↔ GND | 470 uF electrolytic | Bulk. Breakaway current comes from here, not from the supply leads |
| MPU6050 VCC ↔ GND | 100 nF ceramic | Decoupling, at the module |
| Motor terminals | 100 nF **ceramic**, optional | Brush noise, with a reed switch nearby |

**No electrolytic across the motor.** It is polarised and the motor reverses,
so it spends half its life reverse-biased, which is how electrolytics vent. A
capacitor across a PWM'd bridge output is also a near-short at every switching
edge — current through the L298N's transistors that does no work. If brush
noise turns out to need suppression, it is ceramics only, 100 nF or less, and
the bulk capacitance belongs on the supply input instead.

## Buzzer

Through a small NPN — 2N3904 or BC547, 1k from the pin to the base, emitter to
GND, buzzer between 5 V and the collector. A magnetic buzzer is inductive, so
put a 1N4148 across it, cathode to 5 V.

Direct off the pin only if it is a bare piezo element drawing under 10 mA. The
common black cased buzzers pull 30–50 mA, above the 20 mA a 328P pin is
comfortable with and near the 40 mA absolute maximum. Cased buzzers with red
and black leads are polarised — red to the collector side is wrong, red goes
to 5 V.

`tone()` is safe here. It owns Timer2, which drives PWM on D3 and D11, and
neither the L298N (D9, Timer1) nor the HW-039 (D9/D10, both Timer1) uses those
for motor PWM. A passive buzzer on D11 does not disturb speed control.

## Buttons

Each button shorts its pin to GND, no external parts. `INPUT_PULLUP` in
firmware. Debounce in software — these are human presses at a few hundred
milliseconds apart, which is a different problem from the reed and does not
want the reed's treatment.

Do not put a button on D2: that is the reed interrupt.

## What the IMU can and cannot do

The MPU6050 is a six-axis part — three gyro, three accelerometer. **There is no
magnetometer in it.**

That matters because the single actuator here is assigned to **yaw**, and yaw
is the one axis this sensor cannot measure absolutely. The accelerometer finds
the gravity vector, which gives pitch and roll outright; rotation *about* the
gravity vector leaves that vector unchanged, so it is invisible. The gyro gives
yaw rate, and integrating it drifts without bound with no magnetic reference to
correct against — degrees per minute for a consumer MEMS part.

So an MPU6050 cannot replace the compass sighting behind `a` and `b`, and
cannot verify the degrees readout. An MPU9250 or a separate magnetometer could;
this part cannot. Wire it anyway if you want it, but wire it for the three jobs
it is genuinely good at:

- **Pitch readout.** From gravity, absolute, no calibration drift. The README
  has pitch set by hand — this is the axis the sensor actually serves.
- **Back-drive detection.** Gyro rate with the motor commanded off. The README
  calls a back-driven dish the failure with no symptom, and this is a symptom.
- **An independent witness that the dish is moving.** Reed pulses stopping
  while the motor is on currently reads as either a cam cut or a jam. A gyro
  that still sees motion means neither — it means counts are being lost, which
  nothing here can presently detect.

None of that is implemented. The pins are reserved and the flag exists;
`actuator_v1/` does not read them yet.

## Changes from the Cirkit schematic

For anyone comparing against the drawing this came from:

- Stepper motor → the brushed DC linear actuator, on one bridge. A stepper
  knows its own position, which makes the entire reed-counting architecture
  here pointless; MOON is not a stepper project.
- Reed switch **added** on D2. It was absent from the drawing and it is the
  only position sensor the loop has.
- MPU6050 SDA/SCL moved off D13/D12 to A4/A5.
- Second bridge unused rather than half-wired. OUT1/OUT2 stay empty.
- Separate 5 V supply removed; see the 5 V section above for why it fights
  both the L298N regulator and USB.
- 1 uF electrolytic across the motor removed, replaced by bulk capacitance
  across the supply input.
- Buzzer given a transistor.
