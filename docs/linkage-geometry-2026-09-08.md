# Linkage geometry from CAD — 8 Sep 2026

Source: SolidWorks sketch of the actuator–antenna system at the **home
position**, supplied 8 Sep 2026. This is the `triangle` model's tape-measure
input — the half of the pointing chain that was never blocked by the reed
sensor.

## What the sketch gives directly

Taking the top-right corner of the sketch as origin, +x right, +y up:

| point | position | what it is |
|---|---|---|
| pin 1 | `(-288, 0)` | top-left pin joint |
| pin 2 | `(0, -115)` | middle-right pin joint — **taken as the antenna pivot** |
| pin 3 | `(0, -425)` | bottom-right pin joint (115 + 310 below the corner) |

Stated separately: the actuator measures **511.141 mm** pin to mount at home,
with **252** and **173.5** marked along it and **48** at the bottom.

## The one inconsistency

`288 / 115 / 310` put pins 1 and 3 exactly **513.390 mm** apart. The stated
actuator length is **511.141 mm**. The gap is **2.249 mm (0.44%)**.

That is too large to be rounding — 511.141 carries three decimals, so it is a
driven dimension and should be exact — and too small to be a misread
dimension. The unaccounted **48 mm** at the bottom of the sketch is the
obvious suspect, and it does reconcile: a point 48 mm from pin 3, offset
about 42 mm left and 23 mm down, sits exactly 511.141 mm from pin 1. That
matches the sketch, where the diagonal visibly terminates left of and below
the bottom pin before a short segment closes to it.

But it means the actuator's lower end is **not** the circled pin joint, which
contradicts "all the circles are pinjoint". Unresolved — see *Next*.

## Three candidate readings

Pivot is pin 2 in all three. `a` and `b` are symmetric in the law of cosines,
so it does not matter which end of the actuator is fixed and which moves —
that only sets `direction`, and the bench settles it.

| | a | b | L0 | θ₀ | ceiling | arc over 77 mm | °/mm at home |
|---|---:|---:|---:|---:|---:|---:|---:|
| **A** ends at circled pins | 310.111 | 310.000 | 513.390 | 111.767° | +106.7 mm | 32.611° | 0.3295 |
| **B** stated length forced | 310.111 | 310.000 | 511.141 | 111.030° | +108.970 mm | 32.014° | 0.3264 |
| **C** lower end 48 mm off pin | 310.111 | 335.741 | 511.141 | 104.567° | +134.711 mm | 26.581° | 0.2906 |

A ignores the stated length. B forces it and leaves 2.249 mm unexplained. C
reconciles everything but moves the attachment off the circled pin.

## What holds regardless of which is right

**There is a hard geometric ceiling on extension.** At `L = a + b` the three
pins go collinear and the triangle cannot close; past that the model raises
`LinkageError` and the mechanism itself would jam or invert. The ceiling sits
between **+107 mm and +135 mm** of extension from home on all three readings.
Something must stop the actuator before it. This is a mechanical limit, not a
software one, and no firmware constant currently encodes it.

**The 4 Sep bench travel is independently corroborated.** That session
measured 77 mm from home to the extend hard stop. All three readings admit
77 mm comfortably, with 30–58 mm of margin to the singularity. Two
independent methods — a ruler on the rod, and CAD geometry — now agree, and
neither depends on the reed count.

**252 mm and 173.5 mm cannot be the stroke.** Both exceed the ceiling on
every reading; the triangle cannot close at either. They must be internal
actuator dimensions — body length, exposed rod at home, or similar — not
travel. This also settles a contradiction in
[calibration-2026-09-04.md](calibration-2026-09-04.md), which recorded "full
stroke, retract limit to extend limit, is therefore about 328 mm". **That
line is wrong.** 328 mm was the exposed rod length at full extension, not the
stroke; a 328 mm stroke is geometrically impossible in this linkage. The
77 mm figure in the same document is the good one.

**Angular resolution is roughly 0.3°/mm at home**, improving to about
0.58°/mm at full extension as the linkage opens up. Over the 77 mm stroke the
dish sweeps **27–33°**, which is ample for a fixed geostationary target.

## What this does NOT unblock

`mm_per_count` is still missing, and it is still the only sensor-derived
input to the triangle model. The geometry above converts millimetres to
degrees; nothing here converts reed counts to millimetres. That remains
blocked on the reed/magnet inspection.

So the chain now stands:

| link | state |
|---|---|
| counts → mm | **blocked** — 7.5× inconsistency, 4 Sep |
| mm → geometry | **measured** — this document |
| geometry → degrees | **fixed and tested** — 27/27, 8 Sep |

## Next

1. Resolve A / B / C. The question is narrow: does the actuator's lower rod
   end attach at the circled bottom pin joint, or at a point about 48 mm from
   it? The spread is 7.2° in θ₀ and about 6° in swept arc — the first is
   absorbed by sighting `angle_at_retract_deg` on the bench, the second is
   not, and shows up as pointing error away from the calibration points.
2. Confirm what 252 and 173.5 measure. Neither is the stroke. Note the
   supplied description said "511.141 − 153.5" while the sketch reads 173.5.
3. Measure the geometric ceiling against the extend cam position, and decide
   whether `SOFT_LIMIT_MARGIN` needs to encode it.
