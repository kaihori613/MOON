# Wiring — Rev B

The hardware `actuator_v2/` expects. One brushed DC linear actuator on one
H-bridge, positioned against an **absolute encoder on the yaw pivot**, with
the reed switch kept as a health witness and travel bounded by ±15° limit
switches that the firmware reads, backed by the actuator's own internal cams.

**Nothing on this page has been built.** Rev A — the reed-counting design —
is still in git history and `actuator_v1/` still runs it.

## What changed from Rev A, and why

| | Rev A | Rev B |
|---|---|---|
| Position | reed pulses, signed by commanded direction | AS5600 absolute angle on the pivot |
| Homing | drive into the retract cam every boot, required before any move | no homing; a separate mid-travel reference switch checks the stored zero |
| Bridge | L298N | still the L298N on the bench; `MOTOR_DRIVER` selects the G2 when it arrives |
| Reed | the position sensor | motion witness only |
| IMU | MPU6050 planned | dropped |
| Limits | actuator's internal cams only | ±15° limit switches read by the firmware, plus the actuator's own cams |

Three things drove it. The reed had no direction and sat on the motor side of
the leadscrew backlash, so it measured the wrong quantity in the wrong place.
It went blind whenever wind back-drove the dish with the motor off. And the
L298N is a 2 A part with a 2–3 V drop being asked to run a jack at 25 V.

## Pin map

| Pin | Net | Notes |
|---|---|---|
| D0, D1 | USB serial | Console, and the target-angle channel from the host |
| D2 | Reed switch | **INT0.** Health witness — nothing integrates it any more |
| D3 | Extend button | Push button, NO to GND. Same pin as v1 and `reed_switch_test` |
| D4 | Retract button | Push button, NO to GND. Same pin as v1 and `reed_switch_test` |
| D5 · D6 · D9 | L298N channel A — **dead** | Held LOW from the first line of `setup()`. OUT1/OUT2 stay unconnected |
| D7 | **Kill button** | Push button, NO to GND. Latches; release, then `k` |
| D8 | L298N **IN3** | `digitalWrite` only, never PWM |
| D10 | L298N **ENB** | PWM, Timer1 |
| D11 | Buzzer | Via a transistor |
| D12 | L298N **IN4** | `digitalWrite` only, never PWM |
| A0 | Limit switch 1, +15° | Microswitch COM–NC to GND, signal only. The one reached while **extending** — check it, see below |
| A1 | Limit switch 2, −15° | Microswitch COM–NC to GND, signal only |
| A2 | **Home (reference) switch** | Mid-travel, NO to GND. Not a stop — see below. `USE_REF_SWITCH 1` |
| A3 | free | The G2's current sense if that driver is ever fitted |
| A4 | I2C SDA | AS5600 at 0x36, through a level shifter |
| A5 | I2C SCL | " |
| D13 | free | Carries the on-board LED |

Every switch input uses the filter in *Switch inputs* below, and
`INPUT_PULLUP` stays on as well. There is no separate stop button; the kill
switch stops everything. `USE_STOP_BUTTON` puts one on A3 if it is ever added.

**Check which limit is which before any powered move.** Trip each limit
switch by hand and send `s`: the extend-side switch must light the `+` in
`lim=`. If it lights the `-`, swap `PIN_LIM_POS` and `PIN_LIM_NEG` in
`Config.h`. Swapped, the firmware blocks the direction that escapes a limit
and allows the one that drives further in — and nothing else stops it short
of the actuator's internal cams.

**I2C is A4/A5 and cannot be moved.** `Wire` on a 328P is tied to that
peripheral. An encoder on any other pin will not enumerate.

`MOTOR_DRIVER` in `Config.h` picks the pin set. It defaults to `DRV_L298N` on
channel B (`L298N_CHANNEL 2`), the same pins v1 and `reed_switch_test` use —
channel A on this module has a shorted high side, see the README. `DRV_G2`
switches to the Pololu's two-pin interface: PWM on D9, DIR on D12, `/SLP` on
D8. D7 stays the kill input on both drivers.

**Take the ENA and ENB jumpers off the module.** They tie the enables to 5 V.
With the ENA jumper on, holding D9 low does nothing, and the shorted high side
becomes a short across the 25 V rail the first time IN1 goes low.

`/SLP` is the pin most easily forgotten on the G2. It boots asleep, and a
perfectly correct PWM into a sleeping bridge moves nothing while looking
exactly like a dead motor. `actuator_v2` drives it low in `setup()` and only
raises it inside `driveSigned()`, so it doubles as the hard off switch.

### Running the L298N at 25 V

It is inside the part's voltage rating — Vs goes to 46 V — so the volts are
not the problem. Three other things are, and all are survivable for bring-up
because the duty cycle is tiny: a move takes seconds and then the dish sits
for hours, so thermal mass matters more than the continuous rating.

1. **Current.** 2 A per channel. On a healthy module the two bridges can be
   paralleled for about 4 A. **Not on this one:** channel A's high side is
   shorted, and joining OUT1 to OUT3 would put that short on the motor lead.
   This board runs channel B alone, so 2 A is the ceiling.
2. **Heat.** The outputs are Darlingtons and drop 2–3 V regardless of load. At
   3 A that is ~7 W in the package, and the postage-stamp heatsink on the red
   modules will not carry it. Fit a real one.
3. **The drop comes off the motor.** 25 V in, roughly 22 V at the actuator.
   Harmless, but the duty numbers found during tuning will not transfer
   unchanged to a MOSFET bridge later.

**Measure the stall current before deciding this is fine.** Over about 3 A
even paralleled and the L298N is the wrong part.

## The encoder

AS5600 on the **yaw pivot**, not on the actuator rod. That matters twice
over: the pivot is where the quantity you actually care about lives, and
putting the sensor there places the leadscrew backlash *inside* the control
loop instead of corrupting the measurement.

- 12 bits over one turn — 4096 counts, **0.0879° each**
- Address is fixed in silicon at **0x36** and cannot be strapped
- Needs a **diametrically** magnetised magnet (6×2.5 mm N35 is typical),
  centred on the shaft axis within ~0.25 mm, at a 0.5–3 mm air gap

Run `as5600_test/` before wiring any of this to the motor. It reports the
`STATUS` magnet bits and the AGC value live, which is the only way to tell a
correctly mounted magnet from one that merely produces plausible numbers on
the bench and drifts once it is on the mount.

**Keep the working range away from the wrap.** The part reads 0–4095 and rolls
over. Fit the magnet so ±15° of travel sits near raw 2048. `angleDiff()` is
wrap-safe either way, but a rollover inside the travel makes every number on
the console confusing to read.

**Check the counting direction.** If the encoder counts down when the driver
drives the boom positive, set `ENCODER_INVERT` to 1 in `Config.h`. Get this
wrong and the loop is positive feedback: the first move runs to a cam at
whatever duty the PID asked for. `as5600_test`'s sweep prints `counting UP` /
`counting DOWN` so this is a two-minute check, not a guess.

### Level shifting

Run at 3.3 V, the AS5600 cannot be wired straight to the Uno's 5 V I2C. A 5 V
AVR reading a 3.3 V high is marginal by specification rather than comfortably
correct, so fit a BSS138-based bidirectional shifter and pull the encoder side
up to 3.3 V. Don't run it direct and rely on it working on the bench.

- **The shifter does not make 3.3 V.** Its LV pin needs 3.3 V from somewhere:
  the Uno's `3.3V` pin (about 50 mA available; the AS5600 draws under 10 mA)
  feeds both LV and the encoder's VCC. HV takes the Uno's `5V`.
- **Tie DIR to GND** (or VCC). A floating DIR lets the counting direction
  change on its own. Check whether the module already straps it.
- The chip also has a 5 V operating mode. If the module is built for it, it
  can go straight onto 5 V, A4 and A5 with no shifter. Check the module's own
  documentation before relying on this.

## Travel limits

Three layers:

1. **Soft limit**, ±13° in `Config.h`, refused before a move starts
2. **Limit switches** at ±15° on the pivot, on A0 and A1. Signal only: they
   tell the firmware, which stops the motor and refuses to drive further in
3. **The actuator's own internal cams** at the ends of the stroke, which cut
   motor current inside the housing and have their own bypass diodes

Normal operation should never reach layer 2. A limit trip means something
went wrong, not "Tuesday".

**The ±15° switches do not cut motor current.** They carry only the pin's
signal, so stopping at ±15° depends on the firmware running. If the sketch
hangs or the bridge fails on, nothing stops the boom until the actuator's
internal cams at the end of the stroke. That is the reason for a hardware way
to cut the 25 V (see *Kill switch*).

**Microswitch contacts: use COM and NC.** Wired NC to GND, a broken wire
reads as a tripped limit rather than as permission to keep going. That is what
`LIMITS_NORMALLY_CLOSED 1` expects. With COM and NO instead, set it to 0 —
it works, but a broken wire then silently removes that limit.

**Put the switches on the pivot, not on the rod.** ±15° of boom is not a fixed
amount of rod travel — the linkage is a triangle, so the ratio changes along
the stroke. Switches on the pivot measure the angle directly and stay correct
everywhere; switches on the rod would only be right at one extension.

## The reference switch — designed, probably not needed

**This bench has one fitted, as the home switch on A2, so `USE_REF_SWITCH`
is now `1`.** It does nothing until asked: `h` crosses it and reports drift,
`H` crosses it and adopts the crossing as zero, then `w` saves. Set
`REF_ANGLE_DEG` to where it really sits before trusting `H`. The case below
for leaving it out still stands; it is kept for why the switch is a check,
not a necessity.

The original recommendation was to leave `USE_REF_SWITCH` at `0`.

The case for it was that the encoder's zero can creep with no symptom: zero is
a raw count in EEPROM, and a magnet slipping on its hub makes every angle wrong
by a constant. The first half is true. **"No symptom" is not.**

**The satellite is a reference** — continuously available and far more
sensitive than any microswitch. If the zero drifts, the metric at the aimed
angle falls, and you are watching the receiver anyway. Better still, the drift
repairs itself: a calibration run does not care what "zero" means, it finds the
encoder reading with the best signal and stores *that*, so a shifted zero is
absorbed into the next run.

What the switch adds over that is the ability to tell **"the mount moved"**
from **"the sensor moved."** Both look like "signal is worse than it was," and
for *recovery* the distinction is irrelevant — you re-peak either way. That is
a diagnostic convenience costing a switch, a pin, a cam lobe, outdoor wiring
and one more thing to fail on a mast.

`host/peak.py`'s `check_pointing()` does the detecting instead, and catches a
shifted mount, a slipped magnet, a wet LNA and a failing feed in one test.

A reference switch **would** earn its place on a system that cannot verify
against its payload — a telescope that only observes at night, a machine tool
where a bad zero ruins the work before anyone notices. None of that is this.

The firmware branch stays, guarded and compiling, because it costs nothing at
`0`. Fit the switch only if a reason appears that the signal cannot cover.

### What is actually lost, precisely

**Nothing about relative angle.** The encoder still reads the boom
continuously and absolutely, to 0.0879°. "Move 3°" still means three degrees,
and the sweep still knows the exact angle of every sample it takes.

| | Known? |
|---|---|
| Relative angle — "the peak is 2.1° right of here" | **Always** |
| True compass heading — "the boom is at 218.7° azimuth" | Only if zero was calibrated against something external |

What is arbitrary is the *label* on the scale: `0.0°` means wherever the boom
was when you typed `z`. Pointing at a geostationary bird needs only the
relative part, which is why this is a small loss.

To put a real heading on the scale without a switch: sight the boom against a
known bearing, then `z` and `w`.

## Hard stops

The ±15° limit switches are purely protective. `actuator_v2` treats driving
into one as a **fault**, not a waypoint: the soft limit at ±13° should have
stopped the move first, so reaching a limit means something went wrong. Clear
it with `k` and jog away — the firmware blocks the direction that would go
further in and allows the one that escapes.

## Kill switch

A **push button**, normally open, from D7 to GND — the same as
`reed_switch_test` (`KILL_IS_NORMALLY_CLOSED 0`). Pressing it latches the
`kill` fault: the motor stops on the first sample and stays stopped after you
let go. Release it, then send `k`. `k` refuses while the button is still held,
and booting with it held boots into the fault.

It is a request to the firmware, not a power cut. It asks for `motorOff()`,
and a shorted bridge does not listen — that is exactly how channel A failed.
Keep a hardware way to cut the 25 V within reach while the motor can move:
the supply's output switch, or a switch in the +25 V lead rated for the
motor's DC current.

## Power

Two rails, one ground, and the ground is the part that matters.

```
  25V supply (+)  ───────┬────────────── DRIVER  VIN
                          │
              1000uF/50V ─┴─ 100nF/50V   (in parallel, at the terminals)
                          │
  25V supply (−)  ──┬────┴────────────── DRIVER  GND
                    │
                    └──────────────────── Arduino GND    <- separate wire
```

The rail is 25 V. **Confirm that against the actuator's own plate before
connecting anything** — TVRO units come as 12, 24 and 36 V.

The G2 24v13 runs 6.5–40 V, which leaves real headroom above 25 V for what the
bridge pumps back into the rail on reversal. That headroom is why it replaced
both the L298N (46 V but only 2 A) and the BTS7960 (13 A but specified only to
27 V, which is not headroom).

### Ground

Star the two ground wires at the **supply negative**, as drawn. Do not run the
Arduino's ground to the driver's GND terminal and call it done. That terminal
carries the motor return current — amps, switched at the PWM frequency — and
the IR drop along that wire lands directly on top of the reed input. This is
the pickup path the noise-floor issue in the README is about.

### 5 V

USB makes it. Feed the driver's logic supply and the level shifter's HV side
from the Arduino's `5V` pin. The shifter's LV side and the encoder take 3.3 V
from the Arduino's `3.3V` pin — the shifter does not make it.

Never drive the Arduino's `5V` pin from an external supply while USB is
connected — that puts the two in a fight through the USB polyfuse, and the
Uno's auto-switchover arbitrates VIN against USB and does nothing for the 5 V
pin. **Do not feed 25 V to `VIN`** either; the onboard regulator would drop
20 V across itself.

## Reed switch

D2 to GND through the reed, plus the filter from the README's open issue:

- 4.7 kΩ pull-up from D2 to 5 V — the internal pull-up is 20–50 kΩ, too weak
  to hold the line against a switching bridge a few inches away
- 220 Ω in series with the reed
- 220 nF from D2 to GND

The falling edge — the one `attachInterrupt` triggers on — sees only the
220 Ω, about 50 µs. The rise is slower, roughly 1.4 ms through the parallel
pull-ups, which is still far under the 50 ms minimum real pulse gap measured
on the bench and under `REED_DEBOUNCE_US = 3000`.

Keep `INPUT_PULLUP` set regardless: it costs nothing and a disconnected reed
then reads high rather than floating.

## Switch inputs

Every switch gets the same filter as the reed, fitted at the **Arduino end**
of its cable — the long wire is what picks up motor noise, so the filter has
to sit where the pin reads it:

- 4.7 kΩ pull-up from the pin to 5 V
- 100 nF from the pin to GND
- 220 Ω in series between the pin and the cable to the switch

| Pin | Switch | Contact |
|---|---|---|
| D3 | Extend push button | NO |
| D4 | Retract push button | NO |
| D7 | Kill push button | NO |
| A0 | Limit switch 1, +15° | NC (COM–NC) |
| A1 | Limit switch 2, −15° | NC (COM–NC) |
| A2 | Home (reference) switch | NO |

Closed, the pin sits at about 0.2 V (5 V × 220 / 4920), a solid LOW. It falls
in about 20 µs and rises in about 0.5 ms, so the kill button and the limits
still act at once and the buttons' 25 ms debounce is untouched. The 220 Ω also
limits the capacitor's discharge through the contacts and protects the pin
from a spike on an outdoor wire. Return each switch to the **Arduino's** GND,
never the driver's.

`actuator_v2` adds a software filter behind this, not instead of it. A limit
must read tripped for `LIMIT_TRIP_MS` (2 ms) to count, and clear for
`LIMIT_CLEAR_MS` (20 ms) before a tripped limit is believed clear again; the
home switch must hold `REF_STABLE_MS` (2 ms) either way. The buttons already
need 25 ms steady. The kill button is left unfiltered on purpose: it acts on
the first reading, because a false kill only stops the motor.

## Capacitors

Four that matter, six to do it properly. **The voltage ratings are not
decoration.**

| # | Value | Rating | Where | Why |
|---|---|---|---|---|
| 1 | 1000 uF electrolytic | **50 V** | Driver VIN ↔ GND | Bulk. Breakaway current comes from here, not down the supply leads |
| 2 | 100 nF ceramic | 50 V | Same terminals, parallel with #1 | An electrolytic's ESL makes it useless at PWM edge speeds |
| 3 | 100 nF ceramic | any | AS5600 VCC ↔ GND | Decoupling, at the module |
| 4 | 220 nF ceramic | any | Reed, D2 ↔ GND | Part of the reed filter above |
| 5 | 100 uF electrolytic | 16 V | 5 V rail at the panel end | The buzzer pulses current down a long wire |
| 6 | 100 nF ceramic | any | Driver logic supply | Only if that run from the Arduino is long |

### Nothing 25 V rated goes on a 25 V rail

Electrolytics want 1.5–2x headroom over working voltage, so 25 V working means
**50 V parts**. Three reasons stack:

- A 25 V supply is not 25 V. Unloaded, or nominal-24 at +10%, it sits at
  26–27 V.
- **Reversing or braking a motor pumps energy back into the rail.** The bridge
  runs backwards as a boost converter into the supply and the rail lifts above
  nominal. `REVERSE_DEAD_MS` exists for that transient and capacitor #1
  absorbs what is left of it.
- Derating is what buys an electrolytic its lifetime.

Buy low-ESR parts, and check the **ripple current** rating rather than only
the capacitance.

### No capacitor across the motor

An electrolytic there is polarised and the motor reverses, so it spends half
its life reverse-biased, which is how electrolytics vent. And any capacitor
across a PWM'd bridge output is a near-short at every switching edge. If brush
noise ever needs suppression it is ceramics only, 100 nF or less, rated for
the full rail — and only once the reed actually shows pickup.

## Buzzer

Through a small NPN — 2N3904 or BC547, 1 kΩ from the pin to the base, emitter
to GND, buzzer between 5 V and the collector, and a 1N4148 across it if it is
magnetic. Direct off the pin only for a bare piezo under 10 mA.

`tone()` is safe here: it owns Timer2, which drives PWM on D3 (an input here) and D11, and
motor PWM is Timer1 on D10 (D9 on the G2). `USE_BUZZER` is on, and
the patterns are one short beep at a good boot, two short beeps when a
reference pass finishes, and three long beeps on any fault, the kill switch
included. None of them blocks the loop. Set `USE_BUZZER` to 0 if nothing is fitted.

## Build order

1. **Check the actuator's plate against 25 V.** Nothing else is safe first.
2. **Run `as5600_test/`** with no motor power at all. Magnet status ok, AGC
   mid-scale, sweep the full range by hand, note the counting direction and
   confirm the range does not straddle the wrap.
3. **Set `ENCODER_INVERT` and `ENCODER_ZERO_DEFAULT`** from what step 2 told
   you.
4. **Wire the two grounds to the supply negative separately.**
5. **Fit C1 and C2 at the driver's supply terminals**, watching C1's polarity.
6. **Limit switches.** Trip each by hand and check `lim=` in `s` (see the
   pin map). Then jog slowly onto each one and confirm the firmware stops
   there and still lets you jog back off. After this, nothing should reach a
   limit again in normal use.
7. **Fit the reference switch** somewhere the boom crosses but never parks,
   and run `H` to adopt it. Note the lobe+hysteresis figure it prints — that
   is the number `REF_DRIFT_WARN_DEG` has to sit above.
8. **`actuator_v2/` with the loop disabled** — jog with `e`/`r` only, and
   watch `m` to confirm the encoder moves the way you expect under power.
9. **Then tune.** The procedure is in the README.
