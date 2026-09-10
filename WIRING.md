# Wiring — Rev B

The hardware `actuator_v2/` expects. One brushed DC linear actuator on one
H-bridge, positioned against an **absolute encoder on the yaw pivot**, with
the reed switch kept as a health witness and travel bounded by cam switches
that cut motor current in hardware.

**Nothing on this page has been built.** Rev A — the reed-counting design —
is still in git history and `actuator_v1/` still runs it.

## What changed from Rev A, and why

| | Rev A | Rev B |
|---|---|---|
| Position | reed pulses, signed by commanded direction | AS5600 absolute angle on the pivot |
| Homing | drive into the retract cam every boot, required before any move | on demand only, as a check that the stored zero is still right |
| Bridge | L298N | Pololu G2 24v13 |
| Reed | the position sensor | motion witness only |
| IMU | MPU6050 planned | dropped |
| Limits | actuator's internal cams only | cams at ±10° that cut current; the minus one doubles as home |

Three things drove it. The reed had no direction and sat on the motor side of
the leadscrew backlash, so it measured the wrong quantity in the wrong place.
It went blind whenever wind back-drove the dish with the motor off. And the
L298N is a 2 A part with a 2–3 V drop being asked to run a jack at 25 V.

## Pin map

| Pin | Net | Notes |
|---|---|---|
| D0, D1 | USB serial | Console, and the target-angle channel from the host |
| D2 | Reed switch | **INT0.** Health witness — nothing integrates it any more |
| D3 | Limit flag, +10° | NC to GND, `INPUT_PULLUP` |
| D4 | Limit flag, −10° | NC to GND, `INPUT_PULLUP`. **Also the home reference** |
| D7 | Driver DIR | |
| D8 | Driver **/SLP** | **Must be driven HIGH** or the bridge stays asleep |
| D9 | Driver PWM | Timer1 |
| D11 | Buzzer | Via a transistor |
| A0 · A1 · A2 | Buttons | To GND, `INPUT_PULLUP` |
| A3 | Driver current sense | Analog. Stall detection, free with this driver |
| A4 | I2C SDA | AS5600 at 0x36, through a level shifter |
| A5 | I2C SCL | " |
| D5, D6, D10, D12, D13 | free | Five spare |

**I2C is A4/A5 and cannot be moved.** `Wire` on a 328P is tied to that
peripheral. An encoder on any other pin will not enumerate.

`/SLP` is the pin most easily forgotten. The G2 boots asleep; a perfectly
correct PWM into a sleeping bridge moves nothing, and it looks exactly like a
dead motor. `actuator_v2` drives it low in `setup()` and only raises it inside
`driveSigned()`, so it doubles as the hard off switch.

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
over. Fit the magnet so ±10° of travel sits near raw 2048. `angleDiff()` is
wrap-safe either way, but a rollover inside the travel makes every number on
the console confusing to read.

**Check the counting direction.** If the encoder counts down when the driver
drives the boom positive, set `ENCODER_INVERT` to 1 in `Config.h`. Get this
wrong and the loop is positive feedback: the first move runs to a cam at
whatever duty the PID asked for. `as5600_test`'s sweep prints `counting UP` /
`counting DOWN` so this is a two-minute check, not a guess.

### Level shifting

The AS5600 is a 3.3 V part and the Uno's I2C is 5 V. A 5 V AVR reading a 3.3 V
high is marginal by specification rather than comfortably correct, so fit a
BSS138-based bidirectional shifter and pull the encoder side up to 3.3 V.
Don't run it direct and rely on it working on the bench.

## Travel limits

Three layers, and they are deliberately not redundant with each other:

1. **Soft limit**, ±9° in `Config.h`, refused before a move starts
2. **Cam microswitches** at ±10° on the pivot, which cut motor current
3. **The actuator's own internal cams** at the ends of the stroke

Normal operation should never reach layer 2. A cam trip means something went
wrong, not "Tuesday".

**Put the cams on the pivot, not on the rod.** ±10° of boom is not a fixed
amount of rod travel — the linkage is a triangle, so the ratio changes along
the stroke. Cams on the pivot measure the angle directly and stay correct
everywhere; cams on the rod would only be right at one extension.

### The escape diodes

A switch that simply opens the motor circuit leaves you stuck at the limit
with no way back. Put both switches in series in one motor lead, each bypassed
by a power diode, with the diodes facing opposite ways:

```
  DRIVER M1 ──┬── SW+ ──┬──┬── SW− ──┬────── MOTOR ────── DRIVER M2
              └── D+ ───┘  └── D− ───┘
                  →            ←
```

- Neither tripped → both switches closed, no drop, no diode current
- `SW+` open → only the direction `D+` passes still flows. You can drive off
- `SW−` open → likewise, the other way
- Both open → the mechanism is past both cams. Investigate, do not power

Use **normally-closed** contacts so a broken wire reads as a tripped limit
rather than as permission to keep going.

### The minus cam is also the home reference

`SW−` does double duty. No third switch, no extra pin.

The encoder is absolute, but its **zero is not** — zero is a raw count in
EEPROM, and nothing in the encoder can tell you the magnet has crept on its
hub or that the EEPROM was wiped. Every angle would be wrong by a constant,
confidently, with no symptom. So the firmware can drive to the minus cam and
compare what the encoder reads there against a stored reference:

- `h` — go to the cam, report the drift, change nothing
- `H` — go to the cam and **adopt** the reading, re-deriving zero from
  `HOME_ANGLE_DEG`

Nothing homes at boot, and no move requires it. It is a check you run after
remounting or when an angle looks wrong.

Two details make it work:

**Approach from one side, twice.** A microswitch's trip point moves with
approach speed, so the firmware clears the switch, seeks it fast, retreats
1°, then creeps back at `HOME_SPEED_CREEP`. Only the creep pass is believed.

**A microswitch is a coarse reference** — expect a couple of tenths of a
degree of repeatability against the encoder's 0.088°. That is fine here,
because the pointing requirement is about ±1.1°, so a home good to 0.2°
costs nothing measurable. It is *not* fine as a position sensor, which is
why it only ever sets the origin.

The escape diodes and homing get along: the approach drives into the cam
until the hardware cuts that direction, and the retreat drives back out
through the diode.

Size the diodes for **full motor current** — they carry it whenever you are
driving off a limit — so a 15 A, 45 V Schottky on a small heatsink, not a
1N4007. At a few amps and ~0.5 V the escape path dissipates a couple of watts,
which is fine briefly and not fine continuously.

**Check each diode's orientation on the bench before trusting it.** Which way
round they go depends on how the motor happens to be wired, and one fitted
backwards traps you at exactly the limit it was meant to let you escape. Drive
onto each cam slowly and confirm you can still drive off it.

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

USB makes it. Feed the driver's logic supply and the level shifter from the
Arduino's `5V` pin; the shifter's low side makes the 3.3 V for the encoder.

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

`tone()` is safe here: it owns Timer2, which drives PWM on D3 and D11, and
motor PWM is Timer1 on D9.

## Build order

1. **Check the actuator's plate against 25 V.** Nothing else is safe first.
2. **Run `as5600_test/`** with no motor power at all. Magnet status ok, AGC
   mid-scale, sweep the full range by hand, note the counting direction and
   confirm the range does not straddle the wrap.
3. **Set `ENCODER_INVERT` and `ENCODER_ZERO_DEFAULT`** from what step 2 told
   you.
4. **Wire the two grounds to the supply negative separately.**
5. **Fit C1 and C2 at the driver's supply terminals**, watching C1's polarity.
6. **Limits and escape diodes**, then drive onto each cam by hand and confirm
   you can drive off it.
7. **`actuator_v2/` with the loop disabled** — jog with `e`/`r` only, and
   watch `m` to confirm the encoder moves the way you expect under power.
8. **Then tune.** The procedure is in the README.
