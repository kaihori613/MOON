# Bench session — 8 Sep 2026

`reed_switch_test` on L298N **channel B**, 25 V supply limited to **2.5 A**
(3 A is the maximum this supply offers), debounce 900 µs, console driven over
USB from Claude while the rod was measured by hand at every stop.

Home mark set at the antenna's home position and called 0. All positions
below are millimetres of rod from that mark.

## Headline

Two distinct faults, one proven and one strongly indicated.

1. **PWM pickup was inflating the count.** At duty 200 the reed line carried
   edges at exactly the enable pin's carrier frequency. They stopped when the
   carrier stopped. **73% of the counts at duty 200 were phantom.**
2. **Retracting loses real counts, progressively, while extending does not.**
   Extend is consistent to ~15% at both ends of the stroke. Retract varies by
   60% and under-counts by two to four times.

The reed and magnet are probably **not** the fault. Do not open the actuator.

## Run log

| # | dir | duty | from → to | counts | travel | mm/count | rejected | fastest gap |
|---|---|---:|---|---:|---:|---:|---:|---:|
| 1 | ext | 200 | 0 → +6 | 52 | 6 mm | 0.115 | 23 | **2.04 ms** |
| 2 | ext | 255 | +6 → +16 | 50 | 10 mm | 0.200 | 21 | 7.63 ms |
| 3 | ret | 255 | +16 → −7 | 50 | 23 mm | 0.460 | 38 | — |
| 4 | ret | 255 | −7 → −44 | 50 | 37 mm | 0.740 | 44 | — |
| 5 | ext | 255 | −44 → −32 | 51 | 12 mm | 0.235 | **3** | — |

## Fault 1 — PWM pickup on the reed line (proven)

`PIN_ENA` is D10, which is Timer1 on an Uno, free-running at **490.20 Hz**.
The sensor report at duty 200 gave:

```
fastest gap : 2.04 ms  -> 490.20 Hz
```

That is the carrier frequency to three significant figures, recovered from
the reed input. Thirteen edges sat in the 2–5 ms band. At duty 255 —
`analogWrite(255)` stops switching altogether on AVR — the same band was
**empty** and the fastest gap moved to 7.63 ms.

The 900 µs debounce cannot reject this, because 2.04 ms is *slower* than the
window. **A longer debounce cannot fix it either:** PWM edges arrive
continuously, so a 3 ms window rejects the edge at 2.04 ms and accepts the
one at 4.08 ms. It halves the phantom rate and disguises the problem. The
sketch's own verdict is the right one — **RC filter and a stronger pull-up**,
plus checking whether the reed cable shares a bundle or a connector shell
with the motor leads.

The contaminated "coast" figure is worth noting too: 2 counts of coast at
duty 200, **zero** at duty 255. Even the overshoot measurement was phantom.

### What this retrospectively explains

Every finding invalidated in [calibration-2026-09-04.md](calibration-2026-09-04.md)
is consistent with a count inflated in proportion to PWM activity — which
varies with duty, load, position and temperature — while real travel stayed
near 2 mm/s. The 4219-count stroke, the "position-dependent load profile",
the "thermal limp", the 74% direction asymmetry and the 14,722-count retract
are all the same artefact seen from different angles.

## Fault 2 — retract loses counts (indicated, not yet proven)

With the carrier gone, a second fault is visible underneath.

**The out-and-back test failed.** `+50` then `−50` at the same duty should
return the rod to its mark within backlash. It went 0 → +16 → **−7**: on a
10 mm move, the return missed by **13 mm**, an error larger than the move.

The asymmetry is not a fixed ratio that could be calibrated out. Retract gave
0.460 then 0.740 mm/count on consecutive runs, degrading as it went, with
count rate falling 6.28 → 4.80 pulses/s and rejections climbing 38 → 44.

**It is direction, not position.** Run 5 extended from −44 mm — the far
retracted end, where retract had been at its worst — and came back clean
immediately: 0.235 mm/count and only **3** rejected edges against 44 on the
retract run that ended there. A magnet air gap that varied along the stroke
could not do that; it would be equally bad in both directions.

### The mechanical hypothesis

On retract the rod covers ground the magnet does not account for, while the
reed is shaken hard enough to produce a flood of sub-0.2 ms edges. Distance
without counts, plus heavy vibration, is the signature of **the load driving
the mechanism faster than the screw turns it** — slip, not sensing. Many TVRO
actuators carry a slip clutch, and a dish retracting under its own weight is
the load case that finds it. It worsened as the rod retracted and the moment
arm grew, which fits.

If that is right, the reed is honest and reports motor revolutions faithfully;
it is the rod-to-motor relationship that breaks. That is a mechanical repair,
not a sensor one, and no amount of filtering will touch it.

Note this is inference from count behaviour, not a direct observation of slip.
It has not been confirmed by watching or feeling the mechanism.

## What can now be said about mm per count

Extending, at duty 255, with no carrier:

- 0.200 mm/count mid-stroke
- 0.235 mm/count at the retracted end

A ±0.5 mm ruler reading on a 10–12 mm move is ±5%, so that 15% spread is
real and not measurement noise. **Good enough to size the problem, not good
enough to calibrate pointing.** It is also extend-only, and a pointing system
that can only be trusted in one direction is not yet a pointing system.

For reference against the linkage: at 0.2 mm/count and 0.3295 °/mm at home,
one count is about **0.066°** — far finer than the dish beamwidth. The
resolution was never the problem.

## Conditions and caveats

- **The supply was limited to 2.5 A, against a documented requirement of
  8–10 A.** 3 A is this supply's maximum. Every run above is therefore taken
  in a condition the repo's own notes say invalidates measurements. The
  direction asymmetry is large enough that current starvation seems unlikely
  to be its cause, but it cannot be ruled out from here, and a better supply
  should be the first thing tried before any of this is treated as settled.
- Duty 255 is the only clean counting condition available today, and it
  costs speed control entirely.
- Noise floor with the driver powered and the carriage still: **0 edges in
  10 s**, twice. There is no ambient pickup. Everything here appears only
  when the motor turns.

## Next

1. **A supply that can deliver 8–10 A.** It is the cheapest way to remove the
   largest caveat on everything above.
2. **RC filter and stronger pull-up on the reed line**, and check its cable
   routing against the motor leads. This fixes fault 1 properly and lets duty
   control come back.
3. **Confirm or kill the slip hypothesis by observation** — watch and feel
   the actuator through a retract under load. If it judders, that is the
   answer. Compare against a retract with the dish load removed if that can
   be arranged.
4. Only then re-measure mm per count, in both directions, and denominate it
   in **pin-to-pin** millimetres per
   [linkage-geometry-2026-09-08.md](linkage-geometry-2026-09-08.md).

## Position at end of session

Rod sits at **−32 mm** from the home mark. It was not driven back, because
count-based moves cannot be trusted to return it and the return needs a ruler.
