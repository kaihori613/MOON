# Bench procedure — is the reed count a distance sensor?

> **Tests 1–3 were run on 8 Sep 2026 and the question is answered.** Results
> in [calibration-2026-09-08.md](calibration-2026-09-08.md). Test 1 was
> impossible (the actuator will not back-drive), Test 2 found PWM pickup at
> 490.20 Hz and a second direction-dependent fault under it, and Test 3 was
> cancelled — the reed and magnet are probably innocent, so the actuator
> should not be opened. They are kept below as the record of how it was
> found. **The live procedure is [Session 2](#session-2--two-compass-sightings)
> at the end.**

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

---

# Session 2 — two compass sightings

The last missing input. Everything measured on 8 Sep was rod millimetres; no
heading has ever been taken. Two sightings convert counts straight to degrees
via `LinearLinkage`, which needs no `mm_per_count`, no triangle and no
geometry — just two points.

## Do these first, in this order

1. **A supply that reaches 8–10 A.** Every number from 8 Sep was taken at a
   2.5 A limit against a documented 8–10 A requirement. This is the largest
   caveat on all of it, and if the limit was contributing to the retract
   asymmetry then the diagnosis itself changes. Cheapest doubt to remove.
2. **RC filter and stronger pull-up on the reed line**, and check whether its
   cable shares a bundle or connector shell with the motor leads. Without
   this you are locked to duty 255 — the only condition with no PWM carrier —
   and you have no speed control at all.
3. **Confirm or kill the slip hypothesis.** Watch and feel the actuator
   through a loaded retract. If it judders, that is the answer and it is a
   mechanical repair, not a filter. This was inferred from count behaviour on
   8 Sep and never observed directly.

## The sighting itself

Only the **extend** direction counts honestly, so both legs are extends.

1. Home the actuator, or start from a marked position you can return to.
2. **Sight the boom and record the heading.** Take the bearing two or three
   times and average — see the compass notes below.
3. Extend by count over as large a span as the stroke allows. The full usable
   travel is about 77 mm, roughly 32°.
4. **Sight again and record.** Same technique, same standing position.
5. The two `(counts, heading)` pairs are `counts_a / angle_a_deg` and
   `counts_b / angle_b_deg` in `host/config.json`. Set
   `headings.magnetic: true` and your local `declination_deg`.

## Compass technique — the two things that matter

**Use the biggest span you can.** A handheld compass reads to perhaps ±1–2°.
Over an 11° step that is ±20% on the slope, which is worse than no
measurement. Over the full ~32° it is about ±5%.

**Keep the compass away from the structure.** A large steel dish frame, a DC
motor and the actuator's own position magnet will all pull a needle. Sight
along the boom from several feet back, or take a bearing to a distant
landmark instead of reading beside the metal.

The second matters less than it appears, and it is worth knowing why: a
**constant** error — declination, local deviation, a consistent parallax in
how you sight — **cancels out of the slope entirely**, because the fit uses
the difference between the two readings. The absolute offset is then absorbed
by the manual trim you would peak on signal anyway. Only random scatter
between the two readings hurts, which is the argument for averaging each one.

## The cross-check

The linkage geometry predicts **0.0755 °/count** near home, from
513.390 mm pin-to-pin and ~0.23 mm/count measured on 8 Sep. If the sighting
fit lands near that, then CAD, linkage math, rod measurement and compass all
corroborate through independent routes, and the pointing model is real rather
than merely self-consistent.

If they disagree badly, that is worth knowing before anything is built on it.

## Also while you are there

**Close the breakaway bracket** — still not done. The actuator did not move
from rest at duty 140 and moved cleanly at 200, so breakaway is somewhere in
(140, 200] and every speed constant inside that bracket is provisional.
`SPEED_HOMING` is **150**. If that is below breakaway, homing fails to start
and the stall watchdog reports it as a jam. Walk `w` up from 140 in steps of
10 and note the first duty that reliably breaks away in both directions.
