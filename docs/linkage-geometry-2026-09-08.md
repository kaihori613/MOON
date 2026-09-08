# Linkage geometry from CAD — 8 Sep 2026

Source: SolidWorks sketch of the actuator–antenna system at the **home
position**, plus a photograph of the mounted assembly, supplied 8 Sep 2026.
This is the `triangle` model's tape-measure input — the half of the pointing
chain that was never blocked by the reed sensor.

**Status: resolved.** The geometry closes exactly. What remains missing is
`mm_per_count`, which this document does not and cannot supply.

## The pins

Taking the top-right corner of the sketch as origin, +x right, +y up:

| point | position | what it is |
|---|---|---|
| pin 1 | `(-288, 0)` | top-left pin joint, on the dish rib |
| pin 2 | `(0, -115)` | middle-right pin joint — the antenna **pivot** |
| pin 3 | `(0, -425)` | bottom-right pin joint, on the mast bracket |

## The 48 mm offset, and why it looked like an error

The sketch's `288 / 115 / 310` put pins 1 and 3 exactly **513.390 mm** apart,
while the actuator was given as **511.141 mm** — a 2.249 mm gap that was too
large to be rounding on a three-decimal driven dimension.

It is not an error. The actuator's bottom pin joint is offset
**perpendicular to the actuator axis by 48 mm**: the rod's line of action
passes to one side of the bolt it pivots on, which is visible in the
photograph as the bracket clamped to the mast pipe. So the two lengths are
the legs and hypotenuse of a right triangle:

```
sqrt(511.141² + 48²) = 513.3898 mm
sqrt(288²     + 425²) = 513.3897 mm
                        ---------
             residual = 0.12 microns
```

That is an exact reconciliation, not a plausible one. Both readings of the
sketch are correct and they describe different distances:

- **511.141 mm** is the actuator's own axial length, rod end to mount.
- **513.390 mm** is the pin-to-pin distance.

**The triangle model wants the pin-to-pin distance.** Kinematics are set by
where the joints are, not by where the tube runs.

## Parameters

| key | value | source |
|---|---:|---|
| `pivot_to_base_mm` | **310.000** | pin 2 → pin 3, given directly |
| `pivot_to_carriage_mm` | **310.111** | pin 2 → pin 1 = √(288² + 115²) |
| `retracted_length_mm` | **513.390** | pin 1 → pin 3 at home — *not* 511.141 |
| `angle_at_retract_deg` | — | bench: sight the boom with the actuator homed |
| `direction` | — | bench: `+1` if extending increases heading |
| `counts_at_retract` | — | bench: position the sketch reports after `h` |
| `mm_per_count` | — | **blocked** on the reed sensor |

`a` and `b` are symmetric in the law of cosines, so it does not matter which
end of the actuator is fixed and which moves. That only sets `direction`, and
the bench settles it.

Derived:

| quantity | value |
|---|---:|
| included angle at the pivot, θ₀ | **111.767°** |
| geometric ceiling (pins collinear at `a + b` = 620.111) | **+106.721 mm** pin-to-pin |
| the same ceiling in rod travel | **+107.11 mm** |
| arc swept over the 77 mm measured stroke | **32.611°** |
| angular resolution at home | **0.3295 °/mm** |
| angular resolution at full extension | ~0.58 °/mm |

## The calibration trap this creates

Because the offset makes the actuator the *leg* and the pin-to-pin span the
*hypotenuse*, the two do not extend at the same rate:

| rod extension | pin-to-pin gain | ratio |
|---:|---:|---:|
| +20 mm | 19.916 mm | 0.99594 |
| +40 mm | 39.837 mm | 0.99623 |
| +77 mm | 76.707 mm | 0.99669 |

Over the full 77 mm stroke the pins gain **76.707 mm, not 77.000** — short by
0.293 mm, or **0.38%**.

So when `mm_per_count` is finally measured, it matters *which distance was
put on the ruler*. Measuring exposed rod against counts yields millimetres of
**rod**, and feeding that straight into the triangle model overstates the
angle by about 0.38%, or roughly **0.12° across the stroke**. Either measure
pin-to-pin directly, or scale a rod-derived figure by ~0.9963.

0.12° is currently far below the sensor problem and below backlash, so this
is recorded rather than corrected. `TriangleLinkage` does not model the
offset; it assumes pin-to-pin length is linear in counts. That assumption is
right to 0.1% over this stroke and is not worth a parameter today.

## What this settles elsewhere

**The 4 Sep travel figure is independently corroborated.** That session
measured 77 mm from home to the extend hard stop with a ruler. CAD admits
77 mm with 29.7 mm of margin to the singularity. Two independent methods
agree, and neither depends on the reed count.

**The 4 Sep stroke figure is wrong.**
[calibration-2026-09-04.md](calibration-2026-09-04.md) records "full stroke,
retract limit to extend limit, is therefore about 328 mm". A 328 mm stroke is
geometrically impossible here — the triangle cannot close past +107 mm. 328 mm
was the exposed rod length at full extension, mislabelled as travel. The
77 mm figure in that same document is the good one.

**252 mm and 173.5 mm are not the stroke either.** Both exceed the ceiling.
They are internal actuator dimensions — body length, exposed rod at home, or
similar. Still worth confirming which is which, and note the supplied
description said "511.141 − 153.5" where the sketch reads 173.5.

## The ceiling is a real mechanical limit

At `L = a + b` the three pins go collinear. Past that the triangle cannot
close: `TriangleLinkage` raises `LinkageError`, and the mechanism itself
would jam or snap through. That wall sits at **+107 mm of rod extension from
home**, and the extend cam stops the actuator at +77 mm — so the machine as
built is safe, with about 30 mm to spare.

Nothing in the firmware encodes this. `SOFT_LIMIT_MARGIN` is denominated in
counts off the *measured* stops, so it inherits the protection only as long
as the cam keeps stopping the rod at +77 mm. A cam adjustment, a different
actuator, or a re-drilled bracket removes it silently.

## Chain status

| link | state |
|---|---|
| counts → mm | **blocked** — 7.5× inconsistency, 4 Sep |
| mm → geometry | **measured and closed** — this document |
| geometry → degrees | **fixed and tested** — 27/27, 8 Sep |

## Next

1. Confirm what 252 and 173.5 measure, and whether it is 173.5 or 153.5.
   Neither is stroke, so nothing downstream waits on it.
2. Reed and magnet inspection. It is the only thing between here and a
   calibrated pointing model.
3. When `mm_per_count` is measured, denominate it in pin-to-pin millimetres
   per the trap above.
