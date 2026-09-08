# Bench procedure — is the reed count a distance sensor?

Written 8 Sep 2026, to be run at the bench. One question, three tests, in
increasing order of cost. Stop as soon as one of them answers.

**The question.** On 4 Sep, two steps of the same run at the same physical
speed disagreed by 7.5× on counts per mm. Either the sensor is not reporting
distance, or something is adding counts that are not travel. Everything
downstream — `mm_per_count`, the triangle model, closed-loop positioning,
autonomous pointing — waits on the answer.

**Division of labour.** Claude drives the serial console over USB. You hold
the ruler and call out the numbers. Nothing energises the motor without you
confirming first.

---

## Pre-flight — all four before anything moves

These are from the repo's own notes, and three of them silently invalidate
every measurement if wrong.

- [ ] **`reed_switch_test` is the flashed sketch**, not `actuator_v1`. The
      `m` command only exists in the former. 115200 baud.
- [ ] **Supply current limit set to 8–10 A.** At 2 A the supply drops into
      constant-current, the voltage collapses, and the run dies looking like
      a stall. If the CC indicator lights during a run, nothing measured in
      that run is valid.
- [ ] **Channel B wiring**: ENB→D10, IN3→D8, IN4→D12, motor across
      OUT3/OUT4. **OUT1 must be unconnected** — its high side is shorted on,
      and the firmware parks D9/D6/D5 low to keep that fault inert.
- [ ] **The arc is clear.** The kill switch cannot stop a shorted output
      stage; only opening the 24 V supply can. Know where the supply switch
      is before the first move.

Two standing rules for the whole session:

- **Time-cap every run.** A 30 s continuous move is how the driver got
  cooked. Channel B shares the die and heat slug with the dead channel A.
- **Let it cool between runs.** Successive runs dying progressively sooner is
  the thermal signature; if you see it, stop and wait.

---

## Test 1 — back-drive by hand (5 minutes, no power)

The cleanest experiment available, if the mechanism allows it.

1. **Power off entirely.** Supply off, not just the kill switch.
2. Try to move the carriage by hand. Many screw actuators will not
   back-drive — if it will not budge, skip to Test 2, no loss.
3. If it moves: measure the exposed rod, move it by hand a good distance,
   measure again. Claude reads the pulse count across the move.

**Reading it.** This has zero motor current, zero PWM, zero brush noise. If
counts per mm comes out consistent here and inconsistent under power, the
problem is electrical and no amount of magnet adjustment will fix it. If it
is inconsistent even by hand, the problem is magnetic or mechanical and
Test 3 is where the answer lives.

---

## Test 2 — the same distance at two duties (the decisive one)

**The principle.** Millimetres per pulse is a property of the screw pitch and
the gear ratio. It is fixed metal. It *cannot* legitimately change with duty
— the sketch's own `m` output says so: "mm/pulse is the mechanism and holds
at any duty." So if it does change, the count is not measuring distance, and
that is a proof rather than an argument.

Duty 255 is the right second point on an AVR because `analogWrite(255)` stops
switching altogether. PWM coupling does not merely change, it disappears.
Note `PIN_ENA` is D10 — Timer1, 490 Hz — and the 4 Sep figure of 250 counts/s
at a 4.0 ms mean gap sits close to half of that. Possibly coincidence. This
test does not care which, it just separates them.

### Run A — duty 200

1. Claude: `z` to zero the statistics, then `w 200`.
2. **You: measure the exposed rod length and call it out.** Rod length from
   the housing face, the same convention as 4 Sep.
3. Claude: `e 10000` — ten seconds of extend, about 20 mm at 2 mm/s.
4. **You: measure the exposed rod again.** The difference is the distance.
5. Claude: `m <difference>` → mm per pulse at duty 200.

### Run B — duty 255

6. Let the driver cool. Minutes, not seconds.
7. Claude: `z`, then `w 255`.
8. **You: measure and call out.**
9. Claude: `e 10000`. Faster at this duty, so expect a longer move.
10. **You: measure again.**
11. Claude: `m <difference>` → mm per pulse at duty 255.

### Reading it

| outcome | meaning |
|---|---|
| the two mm/pulse figures **agree** | the sensor is measuring distance; the 4 Sep inconsistency is elsewhere and Test 3 is next |
| they **disagree** | the count is not a distance sensor — it tracks something that varies with duty |

Keep both runs in the **same direction** so backlash does not enter, and keep
them clear of both hard stops so nothing is truncated by the stall watchdog.
From home there is 77 mm to the extend cam, so two 10 s runs at ~20 mm fit
comfortably.

Treat Run B as diagnostic only, not calibration: the 1500 µs debounce was
validated at duty 200, and at 255 the pulse gaps shorten toward it.

---

## Test 3 — physical inspection

Only worth a session if Tests 1 and 2 point at the magnet rather than the
wiring.

- Mounting security of both reed and magnet — anything that can shift under
  vibration will.
- Air gap, and **whether the gap varies along the stroke.** A gap that opens
  in one region drops counts there, which is one shape the 7.5× could take.
- Whether more than one magnet is in play, or a multi-pole ring. That
  multiplies counts per revolution by a fixed integer, which would show as a
  clean ratio rather than a messy one.
- Cable routing: does the reed's run share a bundle or a connector shell with
  the motor leads? That is the coupling path if Test 2 says electrical.

---

## While you are there — close the breakaway bracket

Independent of everything above, and sensor-independent: it only asks "did it
move at all", not how far.

The actuator did not move from rest at duty 140 and moved cleanly at 200, so
breakaway is somewhere in (140, 200] and every speed constant inside that
bracket is provisional. `SPEED_HOMING` is **150** — if that is below
breakaway, homing fails to start and the stall watchdog reports it as a jam.

Walk `w` up from 140 in steps of 10, attempting a short move from a standstill
at each, and note the first duty that reliably breaks away in both directions.

---

## What this session cannot produce

`mm_per_count` fit for the triangle model — not until the count is trusted.
And when it is measured, denominate it in **pin-to-pin** millimetres, not rod
millimetres: the 48 mm perpendicular offset at the bottom pin makes the pins
gain 76.707 mm over a 77 mm stroke. See
[linkage-geometry-2026-09-08.md](linkage-geometry-2026-09-08.md).

Everything else in the pointing chain is already done and waiting on this.
